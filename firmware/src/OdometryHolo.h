#pragma once

#include "ch.h"
#include <Eigen/Core>


#define PERIOD_ODOM_REPORT 100  // ms

constexpr double ROBOT_RADIUS = 113.0;
constexpr double INC_PER_MM = 18.12;


extern const Eigen::Matrix<float, 3, 3> D;


class OdometryHolo {
public:
    void init();

    void set_pos(double x, double y, double theta);

    double get_x(void) {return _position[0];}
    double get_y(void) {return _position[1];}
    double get_theta(void) {return _position[2];}

    double get_vx(void) {return _speed[0];}
    double get_vy(void) {return _speed[1];}
    double get_vtheta(void) {return _speed[2];}

    void update(double elapsed);

private:
    msg_t sendOdomReport();

    Eigen::Vector3f _position;
    Eigen::Vector3f _speed;

};

