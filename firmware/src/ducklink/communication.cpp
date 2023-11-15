#ifdef __cplusplus
extern "C" {
#endif
    #include <ch.h>
    #include <hal.h>
    #include "printf.h"
    #include "stdutil.h"
#ifdef __cplusplus
}
#endif
#include "locomotion.h"
#include "globalVar.h"
#include "communication.h"
#include "BytesReadBuffer.h"
#include "BytesWriteBuffer.h"



BytesWriteBuffer msgBuffer[NUM_MESSAGES];

msg_t free_messages_queue[NUM_MESSAGES];
MAILBOX_DECL(mb_free_msgs, free_messages_queue, NUM_MESSAGES);

msg_t filled_messages_queue[NUM_MESSAGES];
MAILBOX_DECL(mb_filled_msgs, free_messages_queue, NUM_MESSAGES);


constexpr size_t NUM_CALLBACKS = 10;
msg_callback_t callbacks[NUM_CALLBACKS] = {0};

static const SerialConfig serialConfig =  {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

void comm_init() {
    sdStart(&SD5, &serialConfig);
}


void register_callback(msg_callback_t cb) {
    for(size_t i=0; i<NUM_CALLBACKS; i++) {
        if(!callbacks[i]) {
            callbacks[i] = cb;
            break;
        }
    }
}


/**
 *  Received message from serial. Non-blocking function.
 *  Returns COM_OK if a message is available.
 */
int check_messages(Message& dmsg, BytesReadBuffer& read_buffer) {
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
            if(_nb_bytes_expected == 0) {
                return COM_ERROR;
            }
            _rcv_state = _RCV_PAYLOAD;
        }
    }

    if(_rcv_state == _RCV_PAYLOAD) {
        uint8_t tokens[50];
        uint8_t nb_bytes_to_read = _nb_bytes_expected;
        if(nb_bytes_to_read >= 50) {nb_bytes_to_read = 50;}

        size_t ret = sdReadTimeout(&SD5, tokens, _nb_bytes_expected, TIME_IMMEDIATE);
        for(size_t i=0; i<ret; i++) {
            msg_chk ^= tokens[i];
            _nb_bytes_expected -= 1;
            bool buf_ok = read_buffer.push(tokens[i]);
            
            if(!buf_ok) {
                chprintf ((BaseSequentialStream*)&SDU1, "read buffer put error!\r\n");
            }

            if(_nb_bytes_expected == 0) {
                _rcv_state = _RCV_CHK;
                break;
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

  BytesReadBuffer read_buffer;

  Message msg;

  while (true) {

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
    
    int ret = check_messages(msg, read_buffer);
    if(ret == COM_OK) {
      for(size_t i=0; i<NUM_CALLBACKS; i++) {
          if(callbacks[i]) {
              callbacks[i](msg);
          }
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
