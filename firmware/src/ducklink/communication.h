#ifndef COMMUNICATION_H
#define COMMUNICATION_H


#ifdef __cplusplus
extern "C" {
#endif

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

void start_communication(void);
int check_messages();

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_H */
