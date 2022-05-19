#pragma once

#include "ch.h"
#include <Eigen/Core>


#define PERIOD_ODOM_REPORT 100  // ms

constexpr double ROBOT_RADIUS = 108.54341185440282;
constexpr double INC_PER_MM = 17.753023791070714;


extern const Eigen::Matrix<float, 3, 3> D;


class OdometryHolo {
public:
    void init();

    void set_pos(double x, double y, double theta);

    double get_x(void) {return _position[0];}
    double get_y(void) {return _position[1];}
    double get_theta(void) {return _position[2];}
    Eigen::Vector3f get_pos() {return _position;}

    /**
     * Speed in the robot frame
     */
    Eigen::Vector3f get_speed() {return _speed_r;}

    void update(double elapsed);

private:
    msg_t sendOdomReport();

    Eigen::Vector3f _position;
    Eigen::Vector3f _speed_r;

};

