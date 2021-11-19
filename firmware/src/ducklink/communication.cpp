#ifdef __cplusplus
extern "C" {
#endif
    #include <ch.h>
    #include <hal.h>
    
    #include "printf.h"
    #include "globalVar.h"
    #include "speed_control.h"
    #include "stdutil.h"
#ifdef __cplusplus
}
#endif

#include "odometry.h"
#include "communication.h"
#include "BytesReadBuffer.h"
#include "BytesWriteBuffer.h"
#include "coinlang_up.h"
#include "coinlang_down.h"


BytesWriteBuffer msgBuffer[NUM_MESSAGES];

msg_t free_messages_queue[NUM_MESSAGES];
MAILBOX_DECL(mb_free_msgs, free_messages_queue, NUM_MESSAGES);

msg_t filled_messages_queue[NUM_MESSAGES];
MAILBOX_DECL(mb_filled_msgs, free_messages_queue, NUM_MESSAGES);




BytesReadBuffer read_buffer;
DownMessage msg;

/**
 *  Received message from serial. Non-blocking function.
 *  Returns COM_OK if a message is available.
 */
int check_messages(DownMessage& dmsg) {
    dmsg.clear();
    static enum RcvState _rcv_state = _RCV_START1ST;
    static uint8_t _nb_bytes_expected;
    static uint8_t msg_chk;

    if(_rcv_state == _RCV_START1ST) {
        read_buffer.clear();
        msg_chk = 0;
        uint8_t token;
        size_t ret = sdReadTimeout(&SD5, &token, 1, TIME_IMMEDIATE);
        if(ret > 0 && token == 0XFF) {
            _rcv_state = _RCV_START2ND;
            //chprintf ((BaseSequentialStream*)&SDU1, "Got start1!\r\n");
        }
    }

    if(_rcv_state == _RCV_START2ND) {
        uint8_t token;
        size_t ret = sdReadTimeout(&SD5, &token, 1, TIME_IMMEDIATE);
        if(ret > 0) {
            if(token == 0xFF) {
            //_rcv_state = _RCV_ID;
                _rcv_state = _RCV_LEN;
                //chprintf ((BaseSequentialStream*)&SDU1, "Got start2!\r\n");
            } else {
                //chprintf ((BaseSequentialStream*)&SDU1, "start2 failed! %d\r\n", token);
                _rcv_state = _RCV_START1ST;
            }
        }
    }

    // ID of the message géré par protobuf. En théorie au moins, on verra.
    // if(_rcv_state == _RCV_ID) {
    //     size_t ret = sdReadTimeout(&SD5, p_MSG_ID(_rcv_buff), 1, TIME_IMMEDIATE);
    //     if(ret > 0) {
    //         _rcv_state = _RCV_LEN;
    //     }
    // }

    if(_rcv_state == _RCV_LEN) {
        size_t ret = sdReadTimeout(&SD5, &_nb_bytes_expected, 1, TIME_IMMEDIATE);
        if(ret > 0) {
            //chprintf ((BaseSequentialStream*)&SDU1, "Got len: %d\r\n", _nb_bytes_expected);
            _rcv_state = _RCV_PAYLOAD;
        }
    }

    if(_rcv_state == _RCV_PAYLOAD) {
        uint8_t token;
        size_t ret = sdReadTimeout(&SD5, &token, 1, TIME_IMMEDIATE);
        if(ret > 0) {
            msg_chk ^= token;
            _nb_bytes_expected -= ret;
            bool buf_ok = read_buffer.push(token);
            
            if(!buf_ok) {
                chprintf ((BaseSequentialStream*)&SDU1, "read buffer put error!\r\n");
            }

            if(_nb_bytes_expected == 0) {
                _rcv_state = _RCV_CHK;
            }
        }
        
    }

    if(_rcv_state == _RCV_CHK) {
        uint8_t chk;
        size_t ret = sdReadTimeout(&SD5, &chk, 1, TIME_IMMEDIATE);
        if(ret > 0) {
            _rcv_state = _RCV_START1ST;
            // TODO control checksum
            if(chk == msg_chk) {
                auto err = dmsg.deserialize(read_buffer);
                if(err == EmbeddedProto::Error::NO_ERRORS) {
                    return COM_OK;    
                }
                else {
                    chprintf ((BaseSequentialStream*)&SDU1, "Deserialization error!\r\n\r\n");
                    return COM_ERROR;
                }
            } else {
                chprintf ((BaseSequentialStream*)&SDU1, "chk failed!\r\n\r\n");
                return COM_ERROR;
            }
        }
    }

    return COM_NO_MSG;
}



/// TODO
/// Right now, this thread wait 1ms at each loop. It better should be woken up on 3 conditions:
/// - there is something in the UART buffer (UART interruption or something like this)
/// - some other thread posted a message in the "filled" mailbox

static THD_WORKING_AREA(waCommunication, 1000);
static void el_communicator (void *arg)
{
  (void)arg;
  chRegSetThreadName("odomReport");

  systime_t lastTime = chVTGetSystemTime();

  while (true) {

    systime_t now = chVTGetSystemTime();
    float elapsed = ((float)(now - lastTime)) / CH_CFG_ST_FREQUENCY;

    // if(elapsed > 0.1) {
    //     struct UpOdomReport odom_report = {
    //         .x = get_x(),
    //         .y = get_y(),
    //         .theta = get_theta(),
    //     };
    //     uint8_t buf[SIZE_UpOdomReport];
    //     up_odom_report_to_bytes(&odom_report, buf);
    //     sdWrite(&SD5, (uint8_t *) buf, SIZE_UpOdomReport);

    //     struct UpSpeedReport speed_report = {
    //         .vx = get_vx(),
    //         .vy = get_vy(),
    //         .vtheta = get_vtheta(),
    //     };
    //     uint8_t buf_speed_report[SIZE_UpSpeedReport];
    //     up_speed_report_to_bytes(&speed_report, buf_speed_report);
    //     sdWrite(&SD5, (uint8_t *) buf_speed_report, SIZE_UpSpeedReport);

    //     lastTime = now;
    // }


    BytesWriteBuffer *buffer;
    // send throught UART all messages ready to be sent.
    // get a filled buffer. No timeout : if there is none, then there is no message to be sent
    while(chMBFetchTimeout(&mb_filled_msgs, (msg_t *)&buffer, 0) == MSG_OK) {
        uint32_t buf_size = buffer->get_size();
        uint8_t* data = buffer->get_data();

        uint8_t header[3] = {0xFF, 0xFF, 0};
        header[2] =  static_cast<uint8_t>(buf_size);

        uint8_t chk = 0;
        for(size_t i=0; i<buf_size; i++) {
            chk ^= data[i];
        }

        sdWrite(&SD5, header, 3);
        sdWrite(&SD5, buffer->get_data(), (size_t)buf_size);
        sdWrite(&SD5, &chk, 1);

        buffer->clear();
        // return buffer to the free buffers pool.
        // can't fail right ? Just fetched a message, and filled and free have the same size
        (void)chMBPostTimeout(&mb_free_msgs, (msg_t)buffer, 0);
    }
    
    int ret = check_messages(msg);
    if(ret == COM_OK) {
        if (msg.has_speed_command()) {
            auto vx = msg.speed_command().vx();
            auto vy = msg.speed_command().vy();
            auto vtheta = msg.speed_command().vtheta();
            //chprintf ((BaseSequentialStream*)&SDU1, "Speed cmd: %f, %f, %f\r\n\r\n", vx, vy, vtheta);
            // acquire lock ?!
            set_speed_setPoint(vx, vy, vtheta);
        } else if(msg.has_pid_gains()) {
            auto ng = msg.pid_gains().ng();
            auto kp = msg.pid_gains().kp();
            auto ki = msg.pid_gains().ki();
            auto kd = msg.pid_gains().kd();
            // acquire lock ?!
            set_pid_gains(ng, kp, ki, kd);
        }
        
    }
    
      palToggleLine(LINE_LED_UART);
      chThdSleepMilliseconds(1);

  }
}

void start_communication() {
  // Pre-filling the free buffers pool with the available buffers, the post
  // will not stop because the mailbox is large enough.
  for (int i = 0; i < NUM_MESSAGES; i++) {
    (void)chMBPostTimeout(&mb_free_msgs, (msg_t)&msgBuffer[i], 0);
  }
  chThdCreateStatic(waCommunication, sizeof(waCommunication), NORMALPRIO, &el_communicator, NULL);
}
