#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <string.h>

#define UID 2190931669

union Message_t;

struct TagMessage;

uint16_t compute_cheksum(uint8_t *buffer, int len);

#define MAX_MSG_BUFFER_SIZE 18

#define SIZE_DownPidgain 18
#define  ID_DownPidgain 0

struct DownPidgain{
  float kd;
  float ki;
  float kp;
};

void down_pidgain_from_bytes(union Message_t* msg_u, uint8_t *buffer);
void down_pidgain_to_bytes(struct DownPidgain* msg, uint8_t *buffer);




#define SIZE_DownSpeedCommand 18
#define  ID_DownSpeedCommand 1

struct DownSpeedCommand{
  float vtheta;
  float vx;
  float vy;
};

void down_speed_command_from_bytes(union Message_t* msg_u, uint8_t *buffer);
void down_speed_command_to_bytes(struct DownSpeedCommand* msg, uint8_t *buffer);




#define SIZE_InterMCUUid 10
#define  ID_InterMCUUid 2

struct InterMCUUid{
  uint32_t uid;
};

void inter_mcu_uid_from_bytes(union Message_t* msg_u, uint8_t *buffer);
void inter_mcu_uid_to_bytes(struct InterMCUUid* msg, uint8_t *buffer);




#define SIZE_UpMotorsSpeedReport 18
#define  ID_UpMotorsSpeedReport 3

struct UpMotorsSpeedReport{
  float v1;
  float v2;
  float v3;
};

void up_motors_speed_report_from_bytes(union Message_t* msg_u, uint8_t *buffer);
void up_motors_speed_report_to_bytes(struct UpMotorsSpeedReport* msg, uint8_t *buffer);




#define SIZE_UpOdomReport 18
#define  ID_UpOdomReport 4

struct UpOdomReport{
  float theta;
  float x;
  float y;
};

void up_odom_report_from_bytes(union Message_t* msg_u, uint8_t *buffer);
void up_odom_report_to_bytes(struct UpOdomReport* msg, uint8_t *buffer);




#define SIZE_UpSpeedReport 18
#define  ID_UpSpeedReport 5

struct UpSpeedReport{
  float vtheta;
  float vx;
  float vy;
};

void up_speed_report_from_bytes(union Message_t* msg_u, uint8_t *buffer);
void up_speed_report_to_bytes(struct UpSpeedReport* msg, uint8_t *buffer);



union Message_t {
  struct DownPidgain down_pidgain;
  struct DownSpeedCommand down_speed_command;
  struct InterMCUUid inter_mcu_uid;
  struct UpMotorsSpeedReport up_motors_speed_report;
  struct UpOdomReport up_odom_report;
  struct UpSpeedReport up_speed_report;
};

struct TagMessage {
  uint8_t tag;
  union Message_t msg;
};

void msg_from_bytes(struct TagMessage* tmsg, uint8_t* buffer, uint8_t id);

#endif    // MESSAGES_H