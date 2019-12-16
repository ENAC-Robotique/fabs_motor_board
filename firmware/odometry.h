#ifndef ODOMETRY_H
#define ODOMETRY_H

#define INC_PER_MM 19.733327579357486
#define WHEELBASE 154.84329099722402

void update_odometry(float elapsed);

float get_speed(void);
float get_omega(void);
float get_x(void);
float get_y(void);
float get_theta(void);

#endif /* ODOMETRY_H */
