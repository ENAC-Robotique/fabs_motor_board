#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "messages.h"


#ifdef __cplusplus
extern "C" {
#endif

#define p_MSG_ID(buffer) (buffer + 2)
#define MSG_ID(buffer) (buffer[2])

#define p_MSG_LEN(buffer) (buffer + 3)
#define MSG_LEN(buffer) (buffer[3])

#define p_MSG_PAYLOAD(buffer) (buffer + 4)

#define MSG_CK(buffer) (buffer[MSG_LEN(buffer)+2] | buffer[MSG_LEN(buffer)+3] << 8)

enum RcvState {
    _RCV_START1ST,
    _RCV_START2ND,
    _RCV_ID,
    _RCV_LEN,
    _RCV_PAYLOAD
};

enum MessagesStates {
    COM_OK,
    COM_NO_MSG,
    COM_ERROR,
};

void start_odom_report(void);
int check_messages(struct TagMessage* tmsg);
void print_msg(union Message_t* msg_u, uint8_t msg_id);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_H */
