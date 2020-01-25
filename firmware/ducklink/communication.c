
#include <ch.h>
#include <hal.h>
#include "communication.h"
#include "messages.h"
#include "odometry.h"
#include "printf.h"
#include "globalVar.h"
#include "speed_control.h"
#include "stdutil.h"


//#define SPI_SELECT_MODE SPI_SELECT_MODE_LINE
// SPIConfig spi_config = {
//     .circular = true,
//     .end_cb = NULL,
//     .ssline = LINE_SPI_NSS,
// };

static THD_WORKING_AREA(waOdomReport, 304);
static void odomReporter (void *arg)
{
  (void)arg;
  chRegSetThreadName("odomReport");

  struct TagMessage tmsg;;

  systime_t lastTime = chVTGetSystemTime();

  while (true) {

    systime_t now = chVTGetSystemTime();
    float elapsed = ((float)(now - lastTime)) / CH_CFG_ST_FREQUENCY;

    if(elapsed > 0.1) {
        struct UpOdomReport odom_report = {
            .x = get_x(),
            .y = get_y(),
            .theta = get_theta(),
        };

        uint8_t buf[SIZE_UpOdomReport];
        up_odom_report_to_bytes(&odom_report, buf);
        sdWrite(&SD5, (uint8_t *) buf, SIZE_UpOdomReport);

        

        struct UpSpeedReport speed_report = {
            .vx = get_vx(),
            .vy = get_vy(),
            .vtheta = get_vtheta(),
        };
        uint8_t buf_speed_report[SIZE_UpSpeedReport];
        up_speed_report_to_bytes(&speed_report, buf_speed_report);
        sdWrite(&SD5, (uint8_t *) buf_speed_report, SIZE_UpSpeedReport);

        lastTime = now;
    }



    int ret = check_messages(&tmsg);
    if(ret == COM_OK) {
        if(tmsg.tag == ID_DownSpeedCommand) {

            float32_t vx = tmsg.msg.down_speed_command.vx;
            float32_t vy = tmsg.msg.down_speed_command.vy;
            float32_t vtheta = tmsg.msg.down_speed_command.vtheta;

            set_speed_setPoint(vx, vy, vtheta);
        }
        else if(tmsg.tag == ID_DownPidgain) {
            //print_msg(&tmsg.msg, tmsg.tag);
            set_pid_gains(tmsg.msg.down_pidgain.kp, tmsg.msg.down_pidgain.ki, tmsg.msg.down_pidgain.kd);
        }
    }
    
      palToggleLine(LINE_LED_UART);
      chThdSleepMilliseconds(1);

  }
}

/**
 *  Received message from serial. Non-blocking function.
 *  Returns COM_OK if a message is available.
 */
int check_messages(struct TagMessage* tmsg) {
    static enum RcvState _rcv_state = _RCV_START1ST;
    static int _nb_bytes_expected;
    static uint8_t _rcv_buff[MAX_MSG_BUFFER_SIZE];

    

    // Use of multiple if statement instead of switch is motivated by the fact that
    // if a multiple bytes are available in the reception buffer, the function will
    // read all of them without wainting for the call of this function.

    if(_rcv_state == _RCV_START1ST) {
        uint8_t token;
        size_t ret = sdReadTimeout(&SD5, &token, 1, TIME_IMMEDIATE);
        if(ret > 0 && token == 0XFF) {
            _rcv_state = _RCV_START2ND;
        }
    }

    if(_rcv_state == _RCV_START2ND) {
        uint8_t token;
        size_t ret = sdReadTimeout(&SD5, &token, 1, TIME_IMMEDIATE);
        if(ret > 0 && token == 0XFF) {
            _rcv_state = _RCV_ID;
        } else {
            _rcv_state = _RCV_START1ST;
        }
    }

    if(_rcv_state == _RCV_ID) {
        size_t ret = sdReadTimeout(&SD5, p_MSG_ID(_rcv_buff), 1, TIME_IMMEDIATE);
        if(ret > 0) {
            _rcv_state = _RCV_LEN;
        }
    }

    if(_rcv_state == _RCV_LEN) {
        size_t ret = sdReadTimeout(&SD5, p_MSG_LEN(_rcv_buff), 1, TIME_IMMEDIATE);
        if(ret > 0) {
            _nb_bytes_expected = MSG_LEN(_rcv_buff);
            _rcv_state = _RCV_PAYLOAD;
        }
    }

    if(_rcv_state == _RCV_PAYLOAD) {
        // if msg lenght is _msg_len and we expect _nb_bytes_expected, then it means that we already received (_msg_len - _nb_bytes_expected) bytes.
        int buff_offset = MSG_LEN(_rcv_buff) - _nb_bytes_expected;    // +2 for the header : 0xFF, 0xFF, msg_id, msg_len
        size_t ret = sdReadTimeout(&SD5, p_MSG_PAYLOAD(_rcv_buff) + buff_offset, _nb_bytes_expected, TIME_IMMEDIATE);
        _nb_bytes_expected -= ret;
        if(_nb_bytes_expected == 0) {   //no more bytes expected, the message should be complete !
            _rcv_state = _RCV_START1ST;
            // TODO control checksum
            msg_from_bytes(tmsg, p_MSG_PAYLOAD(_rcv_buff), MSG_ID(_rcv_buff));
            uint16_t ck = compute_cheksum(p_MSG_ID(_rcv_buff), MSG_LEN(_rcv_buff));
            uint16_t rcv_ck = MSG_CK(_rcv_buff);
            if(ck == rcv_ck) {
                return COM_OK;
            } else {
                return COM_ERROR;
            }
        }
    }

    return COM_NO_MSG;
}

void print_msg(union Message_t* msg_u, uint8_t msg_id) {
    switch (msg_id)
    {
    case ID_DownSpeedCommand:
        chprintf ((BaseSequentialStream*)&SDU1, "vx = %f\tvy = %f\tvtheta = %f\r\n",
            msg_u->down_speed_command.vx,
            msg_u->down_speed_command.vy,
            msg_u->down_speed_command.vtheta);
        break;
    case ID_UpOdomReport:
        chprintf ((BaseSequentialStream*)&SDU1, "x = %f\ty = %f\ttheta = %f\r\n",
            msg_u->up_odom_report.x,
            msg_u->up_odom_report.y,
            msg_u->up_odom_report.theta);
        break;
    case ID_DownPidgain:
        chprintf ((BaseSequentialStream*)&SDU1, "kp = %f\tki = %f\tkd = %f\r\n",
            msg_u->down_pidgain.kp,
            msg_u->down_pidgain.ki,
            msg_u->down_pidgain.kd);
    default:
        break;
    }
}

void start_odom_report() {
  chThdCreateStatic(waOdomReport, sizeof(waOdomReport), NORMALPRIO, &odomReporter, NULL);
}