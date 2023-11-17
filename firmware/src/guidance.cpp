#include "guidance.h"
#include "OdometryHolo.h"
#include "globalVar.h"
#include "config.h"
#include "messages.h"
#include "utils.h"
using namespace protoduck;

const Eigen::Vector3d ACCEL = {200.0, 200.0, 2.};
const Eigen::Vector3d MAX_SPEED = {50.0, 50.0, 2.};

void Guidance::init(double rate)
{
    target = odometry.get_pos();
    dt = 1/rate;
}

void Guidance::update()
{
    static systime_t last = chVTGetSystemTime();

    auto pRobotW = odometry.get_pos();
    //auto robot_speed = odometry.get_speed();
    //auto diff_table = target - pRobotW;
    // rotate diff in robot frame

    double theta = pRobotW[2];
    const Eigen::Matrix<double, 3, 3> rot {
        {cos(theta) , sin(theta), 0},
        {-sin(theta), cos(theta), 0},
        {0          ,          0, 1}
    };
    
    

    systime_t now = chVTGetSystemTime();
    double t = chTimeI2MS(chTimeDiffX(start_time, now)) / 1000.0;
    double alpha = t/travel_time;
    Eigen::Vector3d pCarrotW;
    Eigen::Vector3d vCarrotW;
    if(alpha < 1) {    
        pCarrotW = start_point + alpha * (target - start_point);
        Eigen::Vector3d dir = (target - start_point).normalized();
        vCarrotW = dir.cwiseProduct(MAX_SPEED);
    } else {
        pCarrotW = target;
        vCarrotW = {0, 0, 0};
    }

    // Eigen::Vector3d pCarrotW = {200, 0, 0};
    // Eigen::Vector3d vCarrotW = {0, 0, 0};

    Eigen::Vector3d pCarrotR = rot * (pCarrotW - pRobotW);
    Eigen::Vector3d vCarrotR = rot * vCarrotW;
    
    Eigen::Vector3d vConsR = vCarrotR + 0.1 * pCarrotR; // P(id)

    control.set_setPoints(pCarrotR, vConsR);


    if(chTimeI2MS(chTimeDiffX(last, now)) > 100) {
        last = now;

        msg_send_pos(pCarrotW, Pos::PosObject::POS_CARROT_W);
        msg_send_pos(pCarrotR, Pos::PosObject::POS_CARROT_R);
    }

}

void Guidance::setTarget(Eigen::Vector3d target)
{
    // TODO si en cours de route ?
    //      si le point est très proche ?

    this->target = target;
    this->start_point = odometry.get_pos();

    auto diff = target-start_point;


    Eigen::Vector3d d1 = MAX_SPEED.cwiseProduct(MAX_SPEED).cwiseQuotient(2*ACCEL);


    auto T = (ACCEL.cwiseProduct(diff) - MAX_SPEED.cwiseProduct(MAX_SPEED)).cwiseQuotient(ACCEL.cwiseProduct(MAX_SPEED));
    factors = T / T.maxCoeff();

    travel_accel = ACCEL.cwiseProduct(factors);
    // Eigen::Vector3d travel_max_speed = MAX_SPEED.cwiseProduct(factors);

    // Eigen::Vector3d t1 = travel_max_speed.cwiseQuotient(travel_accel);
    // Eigen::Vector3d t2 = T - t1;


    travel_time = T.maxCoeff();

    start_time =chVTGetSystemTime();
}
