#include "odometry.h"
#include "math.h"
#include "encoders.h"

extern "C" {
#include "printf.h"
#include "globalVar.h"
#include "stdutil.h"
}
#include "utils.h"
#include "communication.h"
#include "BytesWriteBuffer.h"
#include "coinlang_up.h"

OdometryDiff odometry;

void OdometryDiff::update_pos(double elapsed) {
    static systime_t lastOdomReportTime = 0;
    int32_t delta_left = get_delta_enc3();
    int32_t delta_right = get_delta_enc4();

    double length = ((double)(delta_left + delta_right)/2.0)/CODING_INC_PER_MM;
    double angle = ((double)(delta_right - delta_left)/CODING_INC_PER_MM)/CODING_WHEELBASE;

    speed = length / elapsed;
    omega = angle / elapsed;
    _x = _x + length*cos(_theta + angle/2.0);
    _y = _y + length*sin(_theta + angle/2.0);
    _theta = center_radians(_theta + angle);


    if(chVTTimeElapsedSinceX(lastOdomReportTime) > TIME_MS2I(PERIOD_ODOM_REPORT)) {
      sendOdomReport();
      lastOdomReportTime = chVTGetSystemTime();
    }
}

void OdometryDiff::update_mot(double elapsed) {
  int32_t delta_mot_left = get_delta_enc1();
  int32_t delta_mot_right = get_delta_enc2();

  speed_left  = static_cast<double>(delta_mot_left) / (MOTOR_INC_PER_MM*elapsed);
  speed_right = static_cast<double>(delta_mot_right)/ (MOTOR_INC_PER_MM*elapsed);
}


msg_t OdometryDiff::sendOdomReport() {
  BytesWriteBuffer* buffer_pos;
  // get a free buffer. no timeout.
  msg_t ret = chMBFetchTimeout(&mb_free_msgs, (msg_t *)&buffer_pos, TIME_IMMEDIATE);
  if(ret != MSG_OK) {
    return ret;
  }

  UpMessage msg;
  auto& pos_report = msg.mutable_pos_report();
  pos_report.set_pos_x(_x);
  pos_report.set_pos_y(_y);
  pos_report.set_pos_theta(_theta);
  msg.serialize(*buffer_pos);
  // post the new message for the communication thread. no timeout.
  (void)chMBPostTimeout(&mb_filled_msgs, (msg_t)buffer_pos, TIME_IMMEDIATE);


  BytesWriteBuffer* buffer_speed;
  // get a free buffer. timeout of 5ms
  ret = chMBFetchTimeout(&mb_free_msgs, (msg_t *)&buffer_speed, TIME_MS2I(5));
  if(ret != MSG_OK) {
    return ret;
  }

  auto& speed_report = msg.mutable_speed_report();
  speed_report.set_vx(speed);
  speed_report.set_vy(0);
  speed_report.set_vtheta(omega);
  msg.serialize(*buffer_speed);
  // post the new message for the communication thread. timeout of 10 ms
  (void)chMBPostTimeout(&mb_filled_msgs, (msg_t)buffer_speed, TIME_MS2I(10));

  return ret;
}
