#include "DifferentialControl.h"
#include "OdometryDiff.h"
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
#define MOTOR_CONTROL_PERIOD 50 //ms
#define ODOMETRY_PERIOD 50 //ms

void DifferentialControl::init() {

  initPwms();

  setMot1(0);
  setMot2(0);
  setMot3(0);

  lastTime_odometry = chVTGetSystemTime();
  lastTime_motors = chVTGetSystemTime();
  setpoint_time = chVTGetSystemTime();

  l_pid.init(30, 1000);
  r_pid.init(30, 1000);

  auto set_setpoint_cb = [this](Message& msg) {
    if (msg.has_speed() && msg.msg_type() == Message::MsgType::COMMAND) {
          auto vx = msg.speed().vx();
          auto vy = msg.speed().vy();
          auto vtheta = msg.speed().vtheta();
          //chprintf ((BaseSequentialStream*)&SDU1, "Speed cmd: %f, %f, %f\r\n\r\n", vx, vy, vtheta);
          // acquire lock ?!
          set_speed_setPoint(vx, vy, vtheta);
      }
  };

  auto set_pid_gains_cb = [this](Message& msg) {
    if(msg.has_motor_pid() && msg.msg_type() == Message::MsgType::COMMAND) {
          auto motor_no = msg.motor_pid().motor_no();
          auto feedforward = msg.motor_pid().feedforward();
          auto kp = msg.motor_pid().kp();
          auto ki = msg.motor_pid().ki();
          auto kd = msg.motor_pid().kd();
          // acquire lock ?!
          set_pid_gains(motor_no, feedforward, kp, ki, kd);
      }
  };

  register_callback(set_setpoint_cb);
  register_callback(set_pid_gains_cb);
}

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

void DifferentialControl::speed_control(OdometryDiff* odometry) {
  systime_t now = chVTGetSystemTime();
  
  
  
  // set speed setpoint to 0 is no speed command has been received since a while.
  if(chVTTimeElapsedSinceX(setpoint_time) > chTimeMS2I(SETPOINT_VALIDITY)) {
    set_speed_setPoint(0, 0, 0);
  }

  if(chVTTimeElapsedSinceX(lastTime_odometry) > chTimeMS2I(ODOMETRY_PERIOD)) {
    double elapsed_odometry = chTimeMS2I(chVTTimeElapsedSinceX(lastTime_odometry))/1000.0;
    odometry->update_pos(elapsed_odometry);
    lastTime_odometry = now;
  }

  if(chVTTimeElapsedSinceX(lastTime_motors) > chTimeMS2I(MOTOR_CONTROL_PERIOD)) {
    double elapsed_motors = chTimeMS2I(chVTTimeElapsedSinceX(lastTime_motors))/1000.0;

    odometry->update_mot(elapsed_motors);

    double speed_left = odometry->get_speed_left();
    double speed_right = odometry->get_speed_right();
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
