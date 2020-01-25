#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "arm_math.h"

//#define DIFF_DRIVE
//#define CODING_WHEELS
#define HOLONOMIC


#define INC_PER_MM 19.733327579357486

#if defined(DIFF_DRIVE) && !defined(CODING_WHEELS) && !defined(HOLONOMIC)

    #define WHEELBASE 154.84329099722402

#elif defined(DIFF_DRIVE) && defined(CODING_WHEELS) && !defined(HOLONOMIC)

    #define WHEELBASE 154.84329099722402
    #define CODING_WHEELBASE 180.0

#elif !defined(DIFF_DRIVE) && !defined(CODING_WHEELS) && defined(HOLONOMIC)

    #define ROBOT_RADIUS    125.0

#else
#error "pleaase define DIFF_DRIVE/CODING_WHEELS/HOLONOMIC according to your configuration!"
#endif


#if defined(HOLONOMIC)
float32_t RW_to_W(float32_t rw);
float32_t W_to_RW(float32_t w);
#endif



void update_odometry(float32_t elapsed);

float get_speed(void);
float get_omega(void);
float get_x(void);
float get_y(void);
float get_theta(void);

float get_vx(void);
float get_vy(void);
float get_vtheta(void);

void get_motor_speeds(arm_matrix_instance_f32* speed);

#endif /* ODOMETRY_H */
