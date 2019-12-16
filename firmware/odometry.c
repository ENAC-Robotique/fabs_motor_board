#include "odometry.h"
#include "encoders.h"
#include "utils.h"
#include "math.h"

#define DIFF_DRIVE
//#define CODING_WHEELS
//#define HOLONOMIC

float speed_l, speed_theta = 0;
float _x, _y, _theta = 0;


void update_odometry(float elapsed) {

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
    int32_t mot1 = get_delta_enc1();
    int32_t mot1 = get_delta_enc2();
    int32_t mot3 = get_delta_enc3();

    #else
    #error "Make your mind bro! Do you have an holonomic robot or a diff drive ? If you have an halonomic you can't have coding wheels!"
    #endif
    
}

float get_speed() {
    return speed_l;
}

float get_omega() {
    return speed_theta;
}

float get_x() {
    return _x;
}

float get_y() {
    return _y;
}

float get_theta() {
    return _theta;
}