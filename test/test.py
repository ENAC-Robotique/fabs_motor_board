#!/usr/bin/python3
from serial import Serial
import struct
import time
from generated import coinlang_down_pb2 as cld, coinlang_up_pb2 as clu


class Com:

    def __init__(self, port, baudrate=115200):
        self.serial = Serial(port, baudrate)
    
    def compute_chk(self, payload):
        chk = 0
        for b in payload:
         chk ^= b
        return chk
    
    def send_data(self, payload):
        header = struct.pack("<BBB", 0xFF, 0xFF, len(payload))
        chk = struct.pack("<B", self.compute_chk(payload))
        data = header + payload + chk
        print(data)
        self.serial.write(data)
    
    def send_speed_cmd(self, vx, vy, vtheta):
        msg = cld.DownMessage()
        msg.speed_command.vx = vx
        msg.speed_command.vy = vy
        msg.speed_command.vtheta = vtheta

        payload = msg.SerializeToString()
        self.send_data(payload)

    def rcv(self):
        state = 0
        payload_len = 0
        while True:
            if state == 0:
                if self.serial.read()[0] == 0xFF:
                    state = 1
                else:
                    print(b)
            if state == 1:
                if self.serial.read()[0] == 0xFF:
                    state = 2
                else:
                    state = 0
            if state == 2:
                payload_len = self.serial.read()[0]
                payload = self.serial.read(payload_len)
                chk = self.serial.read()[0]
                if chk == self.compute_chk(payload):
                    print("success")
                    u = clu.UpMessage()
                    u.ParseFromString(payload)
                    if u.HasField("battery_report"):
                        print(f"battery_report: {u.battery_report.voltage}")
                    elif u.HasField("odom_report"):
                        print("odom report!")
                else:
                    print(f"chk failed: {chk} vs {self.compute_chk(payload)}")
                state = 0


if __name__ == "__main__":
    com = Com("/dev/ttyUSB0")
    
    com.send_speed_cmd(10, 12, 16)
    com.rcv() 
        
