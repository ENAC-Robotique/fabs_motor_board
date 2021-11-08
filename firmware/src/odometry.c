#include "odometry.h"
#include "encoders.h"
#include "utils.h"
#include "arm_math.h"
#include "printf.h"
#include "globalVar.h"
#include "utils.h"


float speed_l, speed_theta = 0;
float _x, _y, _theta = 0;

#if defined(HOLONOMIC)

//motor speeds into Euclidean speeds: v = D+ m
float32_t Dplus_data[] =
  {2.56395025e-16,  -5.77350269e-01,   5.77350269e-01,
  6.66666667e-01,  -3.33333333e-01,  -3.33333333e-01,
  3.33333333e-01,   3.33333333e-01,   3.33333333e-01};
arm_matrix_instance_f32 Dplus = {
  .numRows = 3,
  .numCols = 3,
  .pData = Dplus_data
};

// speed in the robot reference frame : {vx, vy, R*vtheta}
MAKE_VECTOR3(_speed);

MUTEX_DECL(_mut_m_speeds);
MAKE_VECTOR3(_motorSpeeds);

void get_motor_speeds(arm_matrix_instance_f32* speeds) {
  chMtxLock(&(_mut_m_speeds));
  memcpy(speeds->pData, _motorSpeeds.pData, _motorSpeeds.numCols*_motorSpeeds.numRows*sizeof(float32_t));
  chMtxUnlock(&(_mut_m_speeds));
}

MAKE_VECTOR3(_robotPos)

inline float32_t RW_to_W(float32_t rw) {
    return rw/ROBOT_RADIUS;
}

inline float32_t W_to_RW(float32_t w) {
    return w * ROBOT_RADIUS;
}

#endif

void update_odometry(float32_t elapsed) {

    #if defined(DIFF_DRIVE) && !defined(CODING_WHEELS) && !defined(HOLONOMIC)
    int32_t delta_left = get_delta_enc1();
    int32_t delta_right = get_delta_enc2();

    float length = ((float)(delta_left + delta_right)/2.0)/INC_PER_MM;
    float angle = ((float)(delta_right - delta_left)/INC_PER_MM)/WHEELBASE;

    speed_l = length / elapsed;
    speed_theta = angle / elapsed;
    //speed_l = 0.5*speed_l + 0.5*new_odoSpeed;
    //speed_theta = 0.5*speed_theta + 0.5*new_odoOmega;
    _x = _x + length*cos(_theta + angle/2.0);
    _y = _y + length*sin(_theta + angle/2.0);
    _theta = center_radians(_theta + angle);

    #elif defined(DIFF_DRIVE) && defined(CODING_WHEELS) && !defined(HOLONOMIC)
    int32_t mot_left = get_delta_enc1();
    int32_t mot_right = get_delta_enc2();

    #elif defined(HOLONOMIC) && !defined(DIFF_DRIVE) && !defined(CODING_WHEELS)

    MAKE_VECTOR3(_motorsDisplacement)

    // robot displacement in its reference frame : {dx, dy, R*dtheta}
    MAKE_VECTOR3(_robotDisplacement)

    // get encoders
    int32_t inc1 = get_delta_enc1();
    int32_t inc2 = get_delta_enc2();
    int32_t inc3 = get_delta_enc3();

    // get motors displacements
	_motorsDisplacement.pData[0] = (float32_t)inc1 / INC_PER_MM;
	_motorsDisplacement.pData[1] = (float32_t)inc2 / INC_PER_MM;
	_motorsDisplacement.pData[2] = (float32_t)inc3 / INC_PER_MM;

	// compute motors speeds.
    chMtxLock(&(_mut_m_speeds));
    arm_mat_scale_f32(&_motorsDisplacement, 1/elapsed, &_motorSpeeds);
    chMtxUnlock(&(_mut_m_speeds));

	//compute robot displacement, according to robot fame
	arm_mat_mult_f32(&Dplus, &_motorsDisplacement, &_robotDisplacement);

    // compute robot speed in its reference frame.  arm_mat_mult_f32(&Dplus, &_motorSpeeds, &_speed); should also do the trick ?
    arm_mat_scale_f32(&_robotDisplacement, 1/elapsed, &_speed);


    float32_t d_theta = RW_to_W(_robotDisplacement.pData[2]);		//transforms speed in dTheta
    float32_t theta = d_theta + _robotPos.pData[2];
    float32_t dx = _robotDisplacement.pData[0] * cos(theta) - _robotDisplacement.pData[1] * sin(theta);
	float32_t dy = _robotDisplacement.pData[1] * cos(theta) + _robotDisplacement.pData[0] * sin(theta);

    _robotPos.pData[0] += dx;
    _robotPos.pData[1] += dy;
    _robotPos.pData[2] = theta;

    #else
    #error "Make your mind bro! Do you have an holonomic robot or a diff drive ? If you have an holonomic you can't have coding wheels!"
    #endif
    
}

float get_speed() {
#if defined(DIFF_DRIVE)
    return speed_l;
#else
    return 0.0;
#endif
}

float get_omega() {
    return speed_theta;
}

float get_x() {
#if defined(DIFF_DRIVE)
    return _x;
#else
    return _robotPos.pData[0];
#endif
}

float get_y() {
#if defined(DIFF_DRIVE)
    return _y;
#else
    return _robotPos.pData[1];
#endif
}

float get_theta() {
#if defined(DIFF_DRIVE)
    return _theta;
#else
    return _robotPos.pData[2];
#endif
}

float get_vx() {
    return _speed.pData[0];
}

float get_vy() {
    return _speed.pData[1];
}

float get_vtheta() {
    return RW_to_W(_speed.pData[2]);
}
