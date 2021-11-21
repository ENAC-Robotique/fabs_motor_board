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

#include "odometry.h" 
#include "motors.h"
#include "BytesWriteBuffer.h"
#include "utils.h"
#include "messages.h"

using namespace protoduck;

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
  _speed_setPoint.pData[0] = vx;
  _speed_setPoint.pData[1] = vy;
  _speed_setPoint.pData[2] = W_to_RW(vtheta);
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
  _speed_setPoint.pData[0] = speed * arm_cos_f32(direction);
  _speed_setPoint.pData[1] = speed * arm_sin_f32(direction);;
  _speed_setPoint.pData[2] = omega;
  chMtxUnlock(&(mut_speed_set_point));
}

void HolonomicControl::set_pid_gains(float32_t kp, float32_t ki, float32_t kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  chprintf ((BaseSequentialStream*)&SDU1, "kp = %f\tki = %f\tkd = %f\r\n\r\n", kp, ki, kd);
}


static void clamping(arm_matrix_instance_f32* cmd, arm_matrix_instance_f32* error, const float32_t *mins, const float32_t *maxs, bool* clamps){
  for(int i=0;i<3;i++) {
    bool saturation, sign_eq = false;
    if(cmd->pData[i] <= mins[i] || cmd->pData[i] >= maxs[i]) { // saturate
      saturation = true;
    }

    if((cmd->pData[i] >= 0 && error->pData[i] >= 0) ||
       (cmd->pData[i] <= 0 && error->pData[i] <= 0)) {
      sign_eq = true;
    }

    if(saturation && sign_eq) {
      clamps[i] = true;
    } else {
      clamps[i] = false;
    }
    
  }
}


void HolonomicControl::speed_control(void *arg)
{
  (void)arg;

  // systime_t lastTime_odometry = chVTGetSystemTime();
  // systime_t lastTime = chVTGetSystemTime();

  while (true) {
    // systime_t now = chVTGetSystemTime();
    // float32_t elapsed = ((float)(now - lastTime)) / CH_CFG_ST_FREQUENCY;
    // float32_t elapsed_odometry = ((float)(now - lastTime_odometry)) / CH_CFG_ST_FREQUENCY;

    // if(elapsed_odometry > ODOMETRY_PERIOD) {
    //   odometry.update_odometry(elapsed_odometry);
    //   //update_odometry(elapsed_odometry);
    //   lastTime_odometry = now;
    // }

    // if(elapsed > SPEED_CONTROL_PERIOD) {
    //   MAKE_VECTOR3(m_setPoints);
    //   MAKE_VECTOR3(motor_speeds);
    //   MAKE_VECTOR3(m_errors);
    //   MAKE_VECTOR3(m_cmd_p);
    //   MAKE_VECTOR3(m_cmd_i);
      

    //   //auto speeds = odometry.get_motor_speeds();
      
    //   //get_motor_speeds(&motor_speeds); // get current motors speeds

    //   chMtxLock(&(mut_speed_set_point));
    //   arm_mat_mult_f32(&D, &_speed_setPoint, &m_setPoints);       // compute desired motors speed
    //   chMtxUnlock(&(mut_speed_set_point));

    //   arm_mat_sub_f32(&m_setPoints, &motor_speeds, &m_errors);     // compute error

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


      

    //   arm_mat_scale_f32(&m_errors, _kp, &m_cmd_p);                 // P command
    //   arm_mat_scale_f32(&m_Ierr,   _ki, &m_cmd_i);                 // I command
      
    //   arm_mat_add_f32(&m_cmd_p, &m_cmd_i, &m_cmd);



    //   //if(status == ARM_MATH_SUCCESS) {
    //   setMot1(m_cmd.pData[0]);
    //   setMot2(m_cmd.pData[1]);
    //   setMot3(m_cmd.pData[2]);

    //   sendMotorReport(m_cmd.pData[0], m_cmd.pData[1], m_cmd.pData[2]);

    //   lastTime = now;
    // }


    chThdSleepMilliseconds(1);
  }
}
