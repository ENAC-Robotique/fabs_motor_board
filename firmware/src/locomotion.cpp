
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


static THD_WORKING_AREA(waControl, 40000);

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

  int c=0;
  
  while (true)
  {

    if(c++ >= 1000) {
      c = 0;
      guidance.setTarget({500, 0, 0});
    }

    systime_t now = chVTGetSystemTime();

    // odometry (sensing)
    odometry.update();

    // guidance TODO
    guidance.update();

    // time_msecs_t t = chTimeI2MS(chTimeDiffX(start, now));
    // double speed_x = 200 * sin(0.2 * 6.28*t/1000.0);
    // speed_cons[0] = speed_x;
    // pos_cons += speed_cons * CONTROL_PERIOD/1000.0;
    // control.set_setPoints(pos_cons, speed_cons);

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
  guidance.init(CONTROL_RATE);
  control.init();
  guidance.setTarget({500, 0, 0});
  chThdCreateStatic(waEncoders, sizeof(waEncoders), NORMALPRIO, &encoders_thd, NULL);
  chThdCreateStatic(waControl, sizeof(waControl), NORMALPRIO, &control_thd, NULL);
  chThdCreateStatic(waReport, sizeof(waReport), NORMALPRIO-1, &reporter_thd, NULL);
}



static msg_t sendMotorReport() {
  msg_t ret;

  ret = msg_send_motors(odometry.get_motors_pos(), Motors::MotorDataType::MOTORS_POS);
  if(ret != MSG_OK) { return ret; }

  ret = msg_send_motors(odometry.get_motors_speed(), Motors::MotorDataType::MOTORS_SPEED);
  if(ret != MSG_OK) { return ret; }

  ret = msg_send_motors(control.get_cmds(), Motors::MotorDataType::MOTORS_CMD);
  if(ret != MSG_OK) { return ret; }
  
  ret = msg_send_motors(control.get_pos_cons(), Motors::MotorDataType::MOTORS_POS_CONS);
  if(ret != MSG_OK) { return ret; }
  
  return MSG_OK;
}



static msg_t sendOdomReport() {
  auto send_speed = [=]() {
    Message msg;
    auto speed = odometry.get_speed();
    auto& speed_report = msg.mutable_speed();
    speed_report.set_vx(speed[0]);
    speed_report.set_vy(speed[1]);
    speed_report.set_vtheta(speed[2]);
    return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  };

  
  msg_t ret = msg_send_pos(odometry.get_pos(), Pos::PosObject::POS_ROBOT_W);
  if(ret != MSG_OK) {
    return ret;
  }

  if((ret = send_speed()) != MSG_OK) {
    return ret;
  }

  return MSG_OK;

}
