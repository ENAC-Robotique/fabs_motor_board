#include "DifferentialControl.h"
#include "odometry.h"
#include "motors.h"
#include "encoders.h"
#include "hal.h"
#include "communication.h"
#include "BytesWriteBuffer.h"
#include "coinlang_up.h"

extern "C" {
  #include "printf.h"
  #include "globalVar.h"
  #include "stdutil.h"
    
}


#define SETPOINT_VALIDITY 1
#define MOTOR_CONTROL_PERIOD 0.1
#define ODOMETRY_PERIOD 0.1



void DifferentialControl::set_speed_setPoint(double vx, double vy, double vtheta) {
    speed_setPoint = vx;
    omega_setPoint = vtheta;

    double spl = speed_setPoint - WHEELBASE*omega_setPoint/2;
    double spr = speed_setPoint + WHEELBASE*omega_setPoint/2;

    l_pid.set_setpoint(spl);
    r_pid.set_setpoint(spr);

    setpoint_time = chVTGetSystemTime();
}

// TODO omega ?
// message en parametre ?
void DifferentialControl::set_pid_gains(double ng, double kp, double ki, double kd) {
    l_pid.set_gains(ng, kp, ki, kd);
    r_pid.set_gains(ng, kp, ki, kd);
}

void DifferentialControl::speed_control(void *arg) {
  (void)arg;

  systime_t lastTime_odometry = chVTGetSystemTime();
  systime_t lastTime_motors = chVTGetSystemTime();
  setpoint_time = chVTGetSystemTime();

  l_pid.init(20);
  r_pid.init(20);

  set_pid_gains(1.5, 0, 0.4, 0);

  while(true) {

    systime_t now = chVTGetSystemTime();
    double elapsed_odometry = ((double)(now - lastTime_odometry)) / CH_CFG_ST_FREQUENCY;
    double elapsed_motors = ((double)(now - lastTime_motors)) / CH_CFG_ST_FREQUENCY;
    double elapsed_setpoint = ((double)(now - setpoint_time)) / CH_CFG_ST_FREQUENCY;


    // set speed setpoint to 0 is no speed command has been received since a while.
    if(elapsed_setpoint > SETPOINT_VALIDITY) {
      set_speed_setPoint(0, 0, 0);
    }

    if(elapsed_odometry > ODOMETRY_PERIOD) {
      odometry.update_pos(elapsed_odometry);
      lastTime_odometry = now;
    }

    if(elapsed_motors > MOTOR_CONTROL_PERIOD) {
      

      odometry.update_mot(elapsed_motors);

      double speed_left = odometry.get_speed_left();
      double speed_right = odometry.get_speed_right();
      double cmd_left =  l_pid.update(speed_left);
      double cmd_right = r_pid.update(speed_right);
      setMot1(cmd_left);
      setMot2(-cmd_right);

      sendMotorsSpeed(speed_left, speed_right, 0);
      sendMotorsCmd(l_pid.get_setpoint(), r_pid.get_setpoint(), 0);

      lastTime_motors = now;
    }
  }
}

msg_t DifferentialControl::sendMotorsSpeed(double v1, double v2, double v3) {
  BytesWriteBuffer* buffer_pos;
  // get a free buffer. no timeout.
  msg_t ret = chMBFetchTimeout(&mb_free_msgs, (msg_t *)&buffer_pos, TIME_IMMEDIATE);
  if(ret != MSG_OK) {
    return ret;
  }

  UpMessage msg;
  auto& motors_speed = msg.mutable_motor_speed_report();
  motors_speed.set_v1(v1);
  motors_speed.set_v2(v2);
  motors_speed.set_v3(v3);
  msg.serialize(*buffer_pos);
  // post the new message for the communication thread. no timeout.
  (void)chMBPostTimeout(&mb_filled_msgs, (msg_t)buffer_pos, TIME_IMMEDIATE);

  return ret;
}

msg_t DifferentialControl::sendMotorsCmd(double cmd1, double cmd2, double cmd3) {
  BytesWriteBuffer* buffer_pos;
  // get a free buffer. no timeout.
  msg_t ret = chMBFetchTimeout(&mb_free_msgs, (msg_t *)&buffer_pos, TIME_IMMEDIATE);
  if(ret != MSG_OK) {
    return ret;
  }

  UpMessage msg;
  auto& motors_cmd= msg.mutable_motor_cmd_report();
  motors_cmd.set_cmd1(cmd1);
  motors_cmd.set_cmd2(cmd2);
  motors_cmd.set_cmd3(cmd3);
  msg.serialize(*buffer_pos);
  // post the new message for the communication thread. no timeout.
  (void)chMBPostTimeout(&mb_filled_msgs, (msg_t)buffer_pos, TIME_IMMEDIATE);

  return ret;
}
