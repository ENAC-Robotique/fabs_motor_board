
#pragma once

#include "messages.h"
#include "ch.h"
#include <Eigen/LU>

#define M_PI 3.14159265358979323846

#ifndef ABS
#define ABS(val) ((val) < 0 ? -(val) : (val))
#endif


/**
 * Centers an angle in radians to [-pi, pi[
 */
double center_radians(double angle);

double clamp(double lo, double val, double hi);

msg_t post_message(protoduck::Message& msg, protoduck::Message::MsgType msg_type, sysinterval_t timeout);

msg_t msg_send_pos(Eigen::Vector3d pos, protoduck::Pos::PosObject obj);
msg_t msg_send_motors(Eigen::Vector3d mot, protoduck::Motors::MotorDataType obj);
