#include "DifferentialControl.h"
#include "odometry.h"
#include "motors.h"
#include "encoders.h"
#include "hal.h"
#include "communication.h"
#include "BytesWriteBuffer.h"
#include "messages.h"
#include "utils.h"

extern "C" {
  #include "printf.h"
  #include "globalVar.h"
  #include "stdutil.h"
    
}

using namespace protoduck;

#define SETPOINT_VALIDITY 1000  //ms
#define MOTOR_CONTROL_PERIOD 0.05
#define ODOMETRY_PERIOD 0.05



void DifferentialControl::set_speed_setPoint(double vx, double vy, double vtheta) {
    speed_setPoint = vx;
    omega_setPoint = vtheta;


    double spl = speed_setPoint + WHEELBASE*omega_setPoint/2;
    double spr = speed_setPoint - WHEELBASE*omega_setPoint/2;


    l_pid.set_setpoint(spl);
    r_pid.set_setpoint(spr);

    setpoint_time = chVTGetSystemTime();
}

// TODO omega ?
// message en parametre ?
void DifferentialControl::set_pid_gains(uint32_t motor_no, double feedforward, double kp, double ki, double kd) {
    if(motor_no == 0) {
      l_pid.set_gains(feedforward, kp, ki, kd);
      r_pid.set_gains(feedforward, kp, ki, kd);
    } else if(motor_no == 1) {
      l_pid.set_gains(feedforward, kp, ki, kd);
    } else if(motor_no == 2) {
      r_pid.set_gains(feedforward, kp, ki, kd);
    }
}

void DifferentialControl::speed_control(void *arg) {
  (void)arg;

  systime_t lastTime_odometry = chVTGetSystemTime();
  systime_t lastTime_motors = chVTGetSystemTime();
  setpoint_time = chVTGetSystemTime();

  l_pid.init(30, 1000);
  r_pid.init(30, 1000);

  set_pid_gains(0, 0.14, 0.2, 0.1, 0);

  while(true) {

    systime_t now = chVTGetSystemTime();
    double elapsed_odometry = chTimeMS2I(chVTTimeElapsedSinceX(lastTime_odometry))/1000.0;
    double elapsed_motors = chTimeMS2I(chVTTimeElapsedSinceX(lastTime_motors))/1000.0;
   
    // set speed setpoint to 0 is no speed command has been received since a while.
    if(chVTTimeElapsedSinceX(setpoint_time) > chTimeMS2I(SETPOINT_VALIDITY)) {
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

      //chprintf ((BaseSequentialStream*)&SDU1, "speeds = %f\t%f\r\n", speed_left, speed_right);

      //sendMotorsSpeed(speed_left, speed_right, 0);
      //sendMotorsCmd(l_pid.get_setpoint(), l_pid.get_precommand(), 0);

      lastTime_motors = now;
    }
  }
}

msg_t DifferentialControl::sendMotorsSpeed(double v1, double v2, double v3) {
  Message msg;
  auto& motors_speed = msg.mutable_motors_speed();
  motors_speed.set_v1(v1);
  motors_speed.set_v2(v2);
  motors_speed.set_v3(v3);
  return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
}

msg_t DifferentialControl::sendMotorsCmd(double cmd1, double cmd2, double cmd3) {
  Message msg;
  auto& motors_cmd= msg.mutable_motors_cmd();
  motors_cmd.set_cmd1(cmd1);
  motors_cmd.set_cmd2(cmd2);
  motors_cmd.set_cmd3(cmd3);
  return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
}
