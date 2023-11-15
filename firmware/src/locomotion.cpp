
#include "locomotion.h"
#include <ch.h>
#include "HolonomicControl.h"
#include "OdometryHolo.h"
#include "config.h"
#include "globalVar.h"
#include "messages.h"

using namespace protoduck;

static msg_t sendMotorReport(void);
static msg_t sendOdomReport(void);

static THD_WORKING_AREA(waEncoders, 2000);

static void encoders_thd(void* arg) {
  (void)arg;
  chRegSetThreadName("Encoders update");

  while (true)
  {
    systime_t now = chVTGetSystemTime();
    odometry.update_filters();
    chThdSleepUntil(chTimeAddX(now, chTimeMS2I(ENCODERS_PERIOD)));
  }
}


static THD_WORKING_AREA(waControl, 4000);

/**
 * Odometry, guidance, control
*/
static void control_thd(void* arg) {
  (void)arg;
  chRegSetThreadName("Odometry guidance control");

  // tmp
  Eigen::Vector3d pos_cons = {0, 0, 0};
  Eigen::Vector3d speed_cons = {100, 0, 0};
  systime_t start = chVTGetSystemTime();
  
  while (true)
  {
    systime_t now = chVTGetSystemTime();

    // odometry (sensing)
    odometry.update();

    // guidance TODO
    time_msecs_t t = chTimeI2MS(chTimeDiffX(start, now));
    double speed_x = 200 * sin(0.2 * 6.28*t/1000.0);
    speed_cons[0] = speed_x;
    pos_cons += speed_cons * CONTROL_PERIOD/1000.0;
    control.set_setPoints(pos_cons, speed_cons);

    // control (acting)
    control.update();

    chThdSleepUntil(chTimeAddX(now, chTimeMS2I(CONTROL_PERIOD)));
  }
}

static THD_WORKING_AREA(waReport, 2000);
void reporter_thd(void*) {
  chRegSetThreadName("Report");
  while(true) {
    systime_t now = chVTGetSystemTime();
    sendOdomReport();
    sendMotorReport();
    chThdSleepUntil(chTimeAddX(now, chTimeMS2I(PERIOD_CONTROL_REPORT)));
  }
}

void start_locomotion() {
  odometry.init();
  //guidance.init();
  control.init();
  chThdCreateStatic(waEncoders, sizeof(waEncoders), NORMALPRIO, &encoders_thd, NULL);
  chThdCreateStatic(waControl, sizeof(waControl), NORMALPRIO, &control_thd, NULL);
  chThdCreateStatic(waReport, sizeof(waReport), NORMALPRIO-1, &reporter_thd, NULL);
}



static msg_t sendMotorReport() {

  auto send_pos = [=]() {
    Message msg;
    Eigen::Vector3d  motors_pos = odometry.get_motors_pos();
    auto& msg_motors = msg.mutable_motors();
    msg_motors.set_type(Motors::MotorDataType::MOTORS_POS);
    msg_motors.set_m1(motors_pos[0]);
    msg_motors.set_m2(motors_pos[1]);
    msg_motors.set_m3(motors_pos[2]);
    return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  };

  auto send_speed = [=]() {
    Message msg;
    Eigen::Vector3d  motors_speed = odometry.get_motors_speed();
    auto& msg_motors = msg.mutable_motors();
    msg_motors.set_type(Motors::MotorDataType::MOTORS_SPEED);
    msg_motors.set_m1(motors_speed[0]);
    msg_motors.set_m2(motors_speed[1]);
    msg_motors.set_m3(motors_speed[2]);
    return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  };

  auto send_cmds= [=]() {
    Message msg;
    Eigen::Vector3d  cmds = control.get_cmds();
    auto& msg_motors = msg.mutable_motors();
    msg_motors.set_type(Motors::MotorDataType::MOTORS_CMD);
    msg_motors.set_m1(cmds[0]);
    msg_motors.set_m2(cmds[1]);
    msg_motors.set_m3(cmds[2]);
    return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  };

  auto send_pos_cons= [=]() {
    Message msg;
    Eigen::Vector3d  pos_cons = control.get_pos_cons();
    auto& msg_motors = msg.mutable_motors();
    msg_motors.set_type(Motors::MotorDataType::MOTORS_POS_CONS);
    msg_motors.set_m1(pos_cons[0]);
    msg_motors.set_m2(pos_cons[1]);
    msg_motors.set_m3(pos_cons[2]);
    return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  };


  msg_t ret;
  if((ret = send_pos()) != MSG_OK) {
    return ret;
  }

  if((ret = send_speed()) != MSG_OK) {
    return ret;
  }

  if((ret = send_cmds()) != MSG_OK) {
    return ret;
  }

  if((ret = send_pos_cons()) != MSG_OK) {
    return ret;
  }

  return MSG_OK;
}



static msg_t sendOdomReport() {

  auto send_pos = [=]() {
    Message msg;
    auto pos = odometry.get_pos();
    auto& pos_report = msg.mutable_pos();
    pos_report.set_x(pos[0]);
    pos_report.set_y(pos[1]);
    pos_report.set_theta(pos[2]);
    return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  };

  auto send_speed = [=]() {
    Message msg;
    auto speed = odometry.get_speed();
    auto& speed_report = msg.mutable_speed();
    speed_report.set_vx(speed[0]);
    speed_report.set_vy(speed[1]);
    speed_report.set_vtheta(speed[2]);
    return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  };

  msg_t ret;
  if((ret = send_pos()) != MSG_OK) {
    return ret;
  }

  if((ret = send_speed()) != MSG_OK) {
    return ret;
  }

  return MSG_OK;

}
