#include "OdometryHolo.h"
#include "encoders.h"
#include "utils.h"
#include "communication.h"
#include "BytesWriteBuffer.h"
#include "messages.h"
#include <cmath>
#include <Eigen/LU>

#include "globalVar.h"
#include "stdutil.h"

/*
 *  |v1|   |-sin(O1)  cos(O1)  1|   |vx|
 *  |v2| = |-sin(O2)  cos(O2)  1| . |vy|
 *  |v3|   |-sin(O3)  cos(O3)  1|   |Rw|
 *
 *    m  =           D            .   v
 */

// Euclidean speeds into motor speeds: m = Dv


constexpr double THETA1 = 0;
constexpr double THETA2 = -2.0*M_PI/3.0;
constexpr double THETA3 = 2.0*M_PI/3.0;

const double F = 0.8750322081937646;

// Euclidean speeds into motor speeds: m = Dv
const Eigen::Matrix<double, 3, 3> D {
  {-sin(THETA1)*F, cos(THETA1)*F, ROBOT_RADIUS*F},
  {-sin(THETA2), cos(THETA2), ROBOT_RADIUS},
  {-sin(THETA3), cos(THETA3), ROBOT_RADIUS}
};

//motor speeds into Euclidean speeds: v = Dinv m
const Eigen::Matrix<double, 3, 3> Dinv = D.inverse();



constexpr float POS_ALPHA0 = 2.4f;
constexpr float POS_ALPHA1 = 2.08f;
constexpr float POS_ALPHA2 = 0.64f;
constexpr float POS_EPS = 0.04f; // 0.02

std::array<double, 3> alphas = {POS_ALPHA0, POS_ALPHA1, POS_ALPHA2};



void OdometryHolo::init() {
  enc1.init(true);
  enc2.init(true);
  enc3.init(true);
  //enc4.init(true);
  
  for(int i=0; i<3; i++) {
    pos_filters[i].init(alphas, POS_EPS, 40);
    prev_pos[i] = pos_filters[i].get_pos();
  }

  _position = {0, 0, 0};
  _speed_r = {0, 0, 0};

  // auto cb_recalage = [this](Message& msg) {
  //     if(msg.has_pos() && msg.msg_type() == Message::MsgType::COMMAND) {
  //       double x = msg.pos().get_x();
  //       double y = msg.pos().get_y();
  //       double theta = msg.pos().get_theta();
  //       set_pos(x, y, theta);
  //   }
  // };
  // register_callback(cb_recalage);
}

void OdometryHolo::set_pos(double x, double y, double theta) {
  _position = {x, y, theta};
}

void OdometryHolo::update() {

  // TODO update filter at higher rate

  // static systime_t lastOdomReportTime = 0;

  chSysLock();
  double pos0 = pos_filters[0].get_pos();
  double pos1 = pos_filters[1].get_pos();
  double pos2 = pos_filters[2].get_pos();
  double speed0 = pos_filters[0].get_speed();
  double speed1 = pos_filters[1].get_speed();
  double speed2 = pos_filters[2].get_speed();
  chSysUnlock();

  // get encoders increments
  Eigen::Vector3d motors_deltas = {
    pos0 - prev_pos[0],
    pos1 - prev_pos[1],
    pos2 - prev_pos[2],
  };

  Eigen::Vector3d motors_speeds = {
    speed0,
    speed1,
    speed2,
  };

  // robot move in robot frame
  Eigen::Vector3d robot_move_r = Dinv * (motors_deltas / INC_PER_MM);

  _speed_r = Dinv * (motors_speeds / INC_PER_MM);

  // hypothesis: the movement is approximated as a straight line at heading [ oldTheta + dTheta/2 ]
  double theta_mean = _position[2] + robot_move_r[2]/2;

  // rotation matrix from robot frame to table frame
  const Eigen::Matrix<double, 3, 3> R {
    {cos(theta_mean), -sin(theta_mean), 0},
    {sin(theta_mean), cos(theta_mean) , 0},
    {0              , 0               , 1}
  };

  // change frame from robot frame to table frame
  Eigen::Vector3d robot_move_table = R * robot_move_r;

  _position += robot_move_table;

  _position(2) = center_radians(_position(2));

  // if(chVTTimeElapsedSinceX(lastOdomReportTime) > chTimeMS2I(PERIOD_ODOM_REPORT)) {
  //   sendOdomReport();
  //   lastOdomReportTime = chVTGetSystemTime();
  // }
}

msg_t OdometryHolo::sendOdomReport() {
  Message msg;
  auto& pos_report = msg.mutable_pos();
  pos_report.set_x(get_x());
  pos_report.set_y(get_y());
  pos_report.set_theta(get_theta());
  msg_t ret = post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);

  chprintf ((BaseSequentialStream*)&SDU1, "%lf %lf %lf\r\n", get_x(), get_y(), get_theta());

  if(ret != MSG_OK) {
    return ret;
  }


  auto& speed_report = msg.mutable_speed();
  speed_report.set_vx(_speed_r[0]);
  speed_report.set_vy(_speed_r[1]);
  speed_report.set_vtheta(_speed_r[2]);
  ret = post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  return ret;
}
