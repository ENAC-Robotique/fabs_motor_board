#include "messages.h"

uint16_t compute_cheksum(uint8_t *buffer, int len) {
  uint8_t ck_a, ck_b = 0;
  for(int i=0; i<len; i++) {
    ck_a = (ck_a + buffer[i]);       // % 256 by overflow
    ck_b = (ck_b + ck_a);    // % 256 by overflow
  }
  uint16_t ck = (ck_a << 8) | ck_b;
  return ck;
}

void msg_from_bytes(struct TagMessage* tmsg, uint8_t* buffer, uint8_t id) {
  if(id==0) {
    down_pidgain_from_bytes(&tmsg->msg, buffer);
  }
  if(id==1) {
    down_speed_command_from_bytes(&tmsg->msg, buffer);
  }
  if(id==2) {
    inter_mcu_uid_from_bytes(&tmsg->msg, buffer);
  }
  if(id==3) {
    up_motors_speed_report_from_bytes(&tmsg->msg, buffer);
  }
  if(id==4) {
    up_odom_report_from_bytes(&tmsg->msg, buffer);
  }
  if(id==5) {
    up_speed_report_from_bytes(&tmsg->msg, buffer);
  }
  tmsg->tag = id;
}

void down_pidgain_to_bytes(struct DownPidgain* msg, uint8_t *buffer) {
  int offset = 0;
  buffer[offset++] = 0xFF;
  buffer[offset++] = 0xFF;
  buffer[offset++] = ID_DownPidgain;
  buffer[offset++] = SIZE_DownPidgain - 4;
  memcpy(buffer+offset, &msg->kd, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->ki, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->kp, 4);
  offset += 4;
  int16_t checksum = compute_cheksum(buffer+2, SIZE_DownPidgain - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

void down_pidgain_from_bytes(union Message_t* msg_u, uint8_t *buffer) {
  struct DownPidgain* msg = (struct DownPidgain*)msg_u;
  int offset = 0;
  memcpy(&msg->kd, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->ki, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->kp, buffer+offset, 4);
  offset += 4;
}


void down_speed_command_to_bytes(struct DownSpeedCommand* msg, uint8_t *buffer) {
  int offset = 0;
  buffer[offset++] = 0xFF;
  buffer[offset++] = 0xFF;
  buffer[offset++] = ID_DownSpeedCommand;
  buffer[offset++] = SIZE_DownSpeedCommand - 4;
  memcpy(buffer+offset, &msg->vtheta, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->vx, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->vy, 4);
  offset += 4;
  int16_t checksum = compute_cheksum(buffer+2, SIZE_DownSpeedCommand - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

void down_speed_command_from_bytes(union Message_t* msg_u, uint8_t *buffer) {
  struct DownSpeedCommand* msg = (struct DownSpeedCommand*)msg_u;
  int offset = 0;
  memcpy(&msg->vtheta, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->vx, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->vy, buffer+offset, 4);
  offset += 4;
}


void inter_mcu_uid_to_bytes(struct InterMCUUid* msg, uint8_t *buffer) {
  int offset = 0;
  buffer[offset++] = 0xFF;
  buffer[offset++] = 0xFF;
  buffer[offset++] = ID_InterMCUUid;
  buffer[offset++] = SIZE_InterMCUUid - 4;
  memcpy(buffer+offset, &msg->uid, 4);
  offset += 4;
  int16_t checksum = compute_cheksum(buffer+2, SIZE_InterMCUUid - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

void inter_mcu_uid_from_bytes(union Message_t* msg_u, uint8_t *buffer) {
  struct InterMCUUid* msg = (struct InterMCUUid*)msg_u;
  int offset = 0;
  memcpy(&msg->uid, buffer+offset, 4);
  offset += 4;
}


void up_motors_speed_report_to_bytes(struct UpMotorsSpeedReport* msg, uint8_t *buffer) {
  int offset = 0;
  buffer[offset++] = 0xFF;
  buffer[offset++] = 0xFF;
  buffer[offset++] = ID_UpMotorsSpeedReport;
  buffer[offset++] = SIZE_UpMotorsSpeedReport - 4;
  memcpy(buffer+offset, &msg->v1, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->v2, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->v3, 4);
  offset += 4;
  int16_t checksum = compute_cheksum(buffer+2, SIZE_UpMotorsSpeedReport - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

void up_motors_speed_report_from_bytes(union Message_t* msg_u, uint8_t *buffer) {
  struct UpMotorsSpeedReport* msg = (struct UpMotorsSpeedReport*)msg_u;
  int offset = 0;
  memcpy(&msg->v1, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->v2, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->v3, buffer+offset, 4);
  offset += 4;
}


void up_odom_report_to_bytes(struct UpOdomReport* msg, uint8_t *buffer) {
  int offset = 0;
  buffer[offset++] = 0xFF;
  buffer[offset++] = 0xFF;
  buffer[offset++] = ID_UpOdomReport;
  buffer[offset++] = SIZE_UpOdomReport - 4;
  memcpy(buffer+offset, &msg->theta, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->x, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->y, 4);
  offset += 4;
  int16_t checksum = compute_cheksum(buffer+2, SIZE_UpOdomReport - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

void up_odom_report_from_bytes(union Message_t* msg_u, uint8_t *buffer) {
  struct UpOdomReport* msg = (struct UpOdomReport*)msg_u;
  int offset = 0;
  memcpy(&msg->theta, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->x, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->y, buffer+offset, 4);
  offset += 4;
}


void up_speed_report_to_bytes(struct UpSpeedReport* msg, uint8_t *buffer) {
  int offset = 0;
  buffer[offset++] = 0xFF;
  buffer[offset++] = 0xFF;
  buffer[offset++] = ID_UpSpeedReport;
  buffer[offset++] = SIZE_UpSpeedReport - 4;
  memcpy(buffer+offset, &msg->vtheta, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->vx, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->vy, 4);
  offset += 4;
  int16_t checksum = compute_cheksum(buffer+2, SIZE_UpSpeedReport - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

void up_speed_report_from_bytes(union Message_t* msg_u, uint8_t *buffer) {
  struct UpSpeedReport* msg = (struct UpSpeedReport*)msg_u;
  int offset = 0;
  memcpy(&msg->vtheta, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->vx, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->vy, buffer+offset, 4);
  offset += 4;
}

