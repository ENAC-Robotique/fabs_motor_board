#include "HolonomicControl.h"
#ifdef __cplusplus
extern "C" {
#endif
  
  #include <ch.h>
  #include <hal.h>
   
  #include "stdutil.h"
  #include "printf.h"
  #include "globalVar.h"
  #include "utils.h"
#ifdef __cplusplus
}
#endif

#include <cmath>
#include "OdometryHolo.h" 
#include "motors.h"
#include "BytesWriteBuffer.h"
#include "utils.h"
#include "messages.h"
#include "communication.h"

constexpr double THETA0 = 0;
constexpr double THETA2 = -2.0*M_PI/3.0;
constexpr double THETA1 = 2.0*M_PI/3.0;


const Eigen::Matrix<float, 3, 3> D {
  {sin(THETA0), cos(THETA0), 1.0},
  {sin(THETA1), cos(THETA1), 1.0},
  {sin(THETA2), cos(THETA2), 1.0}
};



using namespace protoduck;

void HolonomicControl::init() {

  initPwms();

  setMot1(0);
  setMot2(0);
  setMot3(0);

  set_speed_setPoint(0, 0, 0);

  set_pid_gains(0.1, 0, 0);

  control_time = chVTGetSystemTime();
  setpoint_time = chVTGetSystemTime();

  auto set_setpoint_cb = [this](Message& msg) {
    if (msg.has_speed() && msg.msg_type() == Message::MsgType::COMMAND) {
          auto vx = msg.speed().vx();
          auto vy = msg.speed().vy();
          auto vtheta = msg.speed().vtheta();
          //chprintf ((BaseSequentialStream*)&SDU1, "Speed cmd: %f, %f, %f\r\n\r\n", vx, vy, vtheta);
          set_speed_setPoint(vx, vy, vtheta);
      }
  };

  auto set_pid_gains_cb = [this](Message& msg) {
    if(msg.has_motor_pid() && msg.msg_type() == Message::MsgType::COMMAND) {
        //auto motor_no = msg.motor_pid().motor_no();
        //auto feedforward = msg.motor_pid().feedforward();
        auto kp = msg.motor_pid().kp();
        auto ki = msg.motor_pid().ki();
        auto kd = msg.motor_pid().kd();
        // acquire lock ?!
        set_pid_gains(kp, ki, kd);
      }
  };

  register_callback(set_setpoint_cb);
  register_callback(set_pid_gains_cb);
}

msg_t sendMotorReport(float m1, float m2, float m3) {
  Message msg;
  auto& motors_speed = msg.mutable_motors_speed();
  motors_speed.set_v1(m1);
  motors_speed.set_v2(m2);
  motors_speed.set_v3(m3);
  return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
}


//void clamping(arm_matrix_instance_f32* cmd, arm_matrix_instance_f32* error, const float32_t *mins, const float32_t *maxs, bool* clamps);

/**
 * Set speed setPoint
 * vx: mm/s
 * vy: mm/s
 * vtheta: rad/s
 */
void HolonomicControl::set_speed_setPoint(float32_t vx, float32_t vy, float32_t vtheta) {
  chMtxLock(&(mut_speed_set_point));
  _speed_setPoint(0) = vx;
  _speed_setPoint(1) = vy;
  _speed_setPoint(2) = W_to_RW(vtheta);
  setpoint_time = chVTGetSystemTime();
  chMtxUnlock(&(mut_speed_set_point));
}


/**
 * Set speed setPoint from norm and direction.
 * speed: robot speed in mm/s
 * direction: speed direction in radians, [0, 2*pi]
 * omega: rotation speed in rad/s
 */
void HolonomicControl::set_speed_setPoint_norm_dir(float32_t speed, float32_t direction, float32_t omega) {
  chMtxLock(&(mut_speed_set_point));
  _speed_setPoint(0) = speed * cos(direction);
  _speed_setPoint(1) = speed * cos(direction);;
  _speed_setPoint(2) = omega;
  setpoint_time = chVTGetSystemTime();
  chMtxUnlock(&(mut_speed_set_point));
}

void HolonomicControl::set_pid_gains(float32_t kp, float32_t ki, float32_t kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  chprintf ((BaseSequentialStream*)&SDU1, "kp = %f\tki = %f\tkd = %f\r\n\r\n", kp, ki, kd);
}


// static void clamping(Eigen::Vector3f cmd, Eigen::Vector3f error,
//                       const Eigen::Vector3f mins, const Eigen::Vector3f maxs){
//   for(int i=0;i<3;i++) {
//     bool saturation, sign_eq = false;
//     if(cmd->pData[i] <= mins[i] || cmd->pData[i] >= maxs[i]) { // saturate
//       saturation = true;
//     }

//     if((cmd->pData[i] >= 0 && error->pData[i] >= 0) ||
//        (cmd->pData[i] <= 0 && error->pData[i] <= 0)) {
//       sign_eq = true;
//     }

//     if(saturation && sign_eq) {
//       clamps[i] = true;
//     } else {
//       clamps[i] = false;
//     }
    
//   }
// }


void HolonomicControl::speed_control(OdometryHolo* odometry)
{
  systime_t now = chVTGetSystemTime();

    // set speed setpoint to 0 is no speed command has been received since a while.
  if(chVTTimeElapsedSinceX(setpoint_time) > chTimeMS2I(SETPOINT_VALIDITY)) {
    set_speed_setPoint(0, 0, 0);
  }

  if(chVTTimeElapsedSinceX(control_time) > chTimeMS2I(ODOMETRY_PERIOD)) {
    double elapsed = chTimeMS2I(chVTTimeElapsedSinceX(control_time))/1000.0;
    //odometry->update_pos(elapsed_odometry);

    Eigen::Vector3f m_speeds = {0, 0, 0};// = odometry.get_motor_speeds();    // get current motors speeds


    chMtxLock(&(mut_speed_set_point));
    // compute desired motors speed
    Eigen::Vector3f m_setpoints = D * _speed_setPoint;
    chMtxUnlock(&(mut_speed_set_point));

    // compute errors
    Eigen::Vector3f m_errors = m_setpoints - m_speeds;

    //   // chMtxLock(&(mut_speed_set_point));
    //   // chprintf ((BaseSequentialStream*)&SDU1, "sSp: %.2f %.2f %.2f  \r\n", 
    //   //           _speed_setPoint.pData[0], _speed_setPoint.pData[1], _speed_setPoint.pData[2]);
    //   // chMtxUnlock(&(mut_speed_set_point));
    //   // chprintf ((BaseSequentialStream*)&SDU1, "mSp: %.2f %.2f %.2f  \r\n", 
    //   //           m_setPoints.pData[0], m_setPoints.pData[1], m_setPoints.pData[2]);
    //   // chprintf ((BaseSequentialStream*)&SDU1, "merr: %.2f %.2f %.2f  \r\ncmd: %.2f %.2f %.2f\r\n\r\n",
    //   //           m_errors.pData[0], m_errors.pData[1], m_errors.pData[2],
    //   //           m_cmd.pData[0], m_cmd.pData[1], m_cmd.pData[2]);

    //   bool clamps[3];
    //   clamping(&m_cmd, &m_errors, mins_cmd, maxs_cmd, clamps); // should be integrate ?
    //   for(int i=0; i<3; i++) {                                    //integrate if allowed
    //     if(!clamps[i]) {
    //       m_Ierr.pData[i] += m_errors.pData[i];
    //     }
    //   }


    Eigen::Vector3f m_cmd = m_errors * _kp + m_Ierr * _ki;

    chprintf ((BaseSequentialStream*)&SDU1, "cmd: %f, %f, %f\r\n\r\n", m_cmd[0], m_cmd[1], m_cmd[2]);
    
    setMot1(m_cmd[0]);
    setMot2(m_cmd[1]);
    setMot3(m_cmd[2]);

    sendMotorReport(m_cmd[0], m_cmd[1], m_cmd[2]);


    control_time = now;
  }
}
