#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <functional>
#include "messages.h"
#include <hal.h>

using namespace protoduck;

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MESSAGES 10
extern mailbox_t mb_free_msgs;
extern mailbox_t mb_filled_msgs;

enum RcvState {
    _RCV_START1ST,
    _RCV_START2ND,
    //_RCV_ID,
    _RCV_LEN,
    _RCV_PAYLOAD,
    _RCV_CHK,
};

enum MessagesStates {
    COM_OK,
    COM_NO_MSG,
    COM_ERROR,
};

typedef std::function<void(Message&)> msg_callback_t;

void comm_init();
void register_callback(msg_callback_t cb);

void start_communication(void);
int check_messages();

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_H */
