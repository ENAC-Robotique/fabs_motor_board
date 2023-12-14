#include "guidance.h"
#include "OdometryHolo.h"
#include "globalVar.h"
#include "config.h"
#include "messages.h"
#include "utils.h"
#include "communication.h"

using namespace protoduck;

const Eigen::Vector3d ACCEL = {200.0, 200.0, 2.};
const Eigen::Vector3d MAX_SPEED = {50.0, 50.0, 2.};

void Guidance::init(double rate)
{
    target = odometry.get_pos();
    dt = 1/rate;

    auto set_pos_target = [this](Message& msg) {
        if(msg.has_pos() && msg.msg_type() == Message::MsgType::COMMAND) {
            auto obj = msg.pos().get_obj();
            if(obj == Pos::PosObject::POS_CARROT_W) {
                auto x_target = msg.pos().get_x();
                auto y_target = msg.pos().get_y();
                auto theta_target = msg.pos().get_theta();
                //msg_send_pos({x_target, y_target, theta_target}, Pos::PosObject::POS_CARROT_R);
                palToggleLine(LINE_LED_ORANGE);
                setTarget({x_target, y_target, theta_target});
            }
        }
  };

  register_callback(set_pos_target);
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

    // if(t > 0.2) {

    // }


    Eigen::Vector3d pCarrotR = rot * (target - pRobotW);
    //Eigen::Vector3d vCarrotR = rot * vCarrotW;
    
    // Eigen::Vector3d vConsR = vCarrotR + 0.1 * pCarrotR; // P(id)

    control.set_cons(pCarrotR, {0, 0, 0});


    // if(chTimeI2MS(chTimeDiffX(last, now)) > 100) {
    //     last = now;

    //     msg_send_pos(pCarrotW, Pos::PosObject::POS_CARROT_W);
    //     msg_send_pos(pCarrotR, Pos::PosObject::POS_CARROT_R);
    // }

}

void Guidance::setTarget(Eigen::Vector3d target)
{
    this->target = target;
    // this->start_point = odometry.get_pos();
    start_time =chVTGetSystemTime();
}
