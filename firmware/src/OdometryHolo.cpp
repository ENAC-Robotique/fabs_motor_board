#include "OdometryHolo.h"
#include "encoders.h"
#include "utils.h"
#include "communication.h"
#include "BytesWriteBuffer.h"
#include "messages.h"
#include <cmath>
#include <Eigen/LU>

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
const Eigen::Matrix<float, 3, 3> D {
  {-sin(THETA1)*F, cos(THETA1)*F, ROBOT_RADIUS*F},
  {-sin(THETA2), cos(THETA2), ROBOT_RADIUS},
  {-sin(THETA3), cos(THETA3), ROBOT_RADIUS}
};

//motor speeds into Euclidean speeds: v = Dinv m
const Eigen::Matrix<float, 3, 3> Dinv = D.inverse();

void OdometryHolo::init() {
  initEnc1(true);
  initEnc2(true);
  initEnc3(true);
  //initEnc4(true);

  _position = {0, 0, 0};
  _speed_r = {0, 0, 0};

  auto cb_recalage = [this](Message& msg) {
      if(msg.has_pos() && msg.msg_type() == Message::MsgType::COMMAND) {
        double x = msg.pos().get_x();
        double y = msg.pos().get_y();
        double theta = msg.pos().get_theta();
        set_pos(x, y, theta);
    }
  };
  register_callback(cb_recalage);
}

void OdometryHolo::set_pos(double x, double y, double theta) {
  _position = {x, y, theta};
}

void OdometryHolo::update(double elapsed) {
  static systime_t lastOdomReportTime = 0;

  // get encoders increments
  Eigen::Vector3f motors_deltas = {
    static_cast<float>(get_delta_enc1()),
    static_cast<float>(get_delta_enc2()),
    static_cast<float>(get_delta_enc3())
  };

  // motors move in mm
  Eigen::Vector3f motors_move = motors_deltas / INC_PER_MM;
  
  // robot move in robot frame
  Eigen::Vector3f robot_move_r = Dinv * motors_move;
  _speed_r = robot_move_r / elapsed;

  // hypothesis: the movement is approximated as a straight line at heading [ oldTheta + dTheta/2 ]
  double theta_mean = _position[2] + robot_move_r[2]/2;

  // rotation matrix from robot frame to table frame
  const Eigen::Matrix<float, 3, 3> R {
    {cos(theta_mean), -sin(theta_mean), 0},
    {sin(theta_mean), cos(theta_mean) , 0},
    {0              , 0               , 1}
  };

  // change frame from robot frame to table frame
  Eigen::Vector3f robot_move_table = R * robot_move_r;

  _position += robot_move_table;

  _position(2) = center_radians(_position(2));

  if(chVTTimeElapsedSinceX(lastOdomReportTime) > chTimeMS2I(PERIOD_ODOM_REPORT)) {
    sendOdomReport();
    lastOdomReportTime = chVTGetSystemTime();
  }
}

msg_t OdometryHolo::sendOdomReport() {
  Message msg;
  auto& pos_report = msg.mutable_pos();
  pos_report.set_x(get_x());
  pos_report.set_y(get_y());
  pos_report.set_theta(get_theta());
  msg_t ret = post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);

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
