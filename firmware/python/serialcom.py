#!/bin/env python3
import sys
sys.path.append('generated')
from serial import Serial
from serial.threaded import Protocol, ReaderThread
import time
from enum import Enum
import messages_pb2 as pb
import json
import socket
import struct
from math import sin

plotjuggler_udp = ("127.0.0.1", 9870)


class RxState(Enum):
    IDLE = 0
    HEAD_OK = 1
    GOT_LENGTH = 2
    

class Duckoder(Protocol):
    def __init__(self):
        Protocol.__init__(self)
        self.transport = None
        self._buffer = b'  '
        self._rx_state = RxState.IDLE
        self._msg_rcv = None
        self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        for c in data:
            if self._decode(c.to_bytes(1, 'little')):
                m = pb.Message.FromString(self._msg_rcv)
                topic = m.WhichOneof('inner')
                jj = self.msg_to_json(m)
                self.so.sendto(jj.encode(), plotjuggler_udp)
                #print(jj)

    def _decode(self, c):
        ret = False
        self._buffer += c
        match self._rx_state:
            case RxState.IDLE:
                if self._buffer[-1] == 0xFF and self._buffer[-2] == 0xFF:
                    self._buffer = self._buffer[-2:]
                    self._rx_state = RxState.HEAD_OK
            case RxState.HEAD_OK:
                self._expected_bytes = ord(c)+1
                self._rx_state = RxState.GOT_LENGTH
            case  RxState.GOT_LENGTH:
                self._expected_bytes -= 1
                if self._expected_bytes == 0:
                    chk = 0
                    for c in self._buffer[3:-1]:
                        chk ^= c
                    if chk == self._buffer[-1]:
                        ret = True
                        self._msg_rcv = self._buffer[3:-1]
                    else:
                        print("chk failed")
                    self._buffer = b'  '
                    self._rx_state = RxState.IDLE
        return ret

    def msg_to_json(self, msg):
        msg_name = msg.WhichOneof('inner')
        inner = getattr(msg, msg_name)
        #popo = msg_name
        if msg_name == 'motors':
            msg_name = msg.motors.MotorDataType.Name(msg.motors.type)
        elif msg_name == 'pos':
            msg_name = msg.pos.PosObject.Name(msg.pos.obj)
        d = {msg_name: {}}
        for f in inner.DESCRIPTOR.fields:
            field_name = f.name
            d[msg_name][field_name] = getattr(inner, field_name)
        return json.dumps(d)


def serialize(msg):
    payload = msg.SerializeToString()
    chk = 0
    for c in payload:
        chk ^= c
    data = bytearray()
    data += struct.pack('BBB', 0xff, 0xff, len(payload))
    data += payload
    data += struct.pack('B', chk)
    return data

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("missing port")
        exit(1)
    port = sys.argv[1]
    baudrate = 115200

    ser=Serial(port, baudrate)
    with ReaderThread(ser, Duckoder) as p:
        start_time = time.time()
        while True:
            m = pb.Message()
            m.msg_type = pb.Message.MsgType.COMMAND
            m.pos.obj = pb.Pos.PosObject.POS_CARROT_W
            dt = time.time() - start_time
            m.pos.x = 100*sin(dt)
            m.pos.y = 0
            m.pos.theta = 0
            data = serialize(m)
            ser.write(data)
            
            time.sleep(0.005)
