#pragma once

class DifferentialControl {
public:

    DifferentialControl(): speed_setPoint(0), omega_setPoint(0),
    _intError_speed(0), _prevError_speed(0), _intError_omega(0), _prevError_omega(0),
    NOMINAL_PGAIN(0.2),
    KP_SPEED(0), KI_SPEED(0), KD_SPEED(0), KP_OMEGA(0), KI_OMEGA(0), KD_OMEGA(0)
    {}

    void set_speed_setPoint(double vx, double vy, double vtheta);
    void set_speed_setPoint_norm_dir(double speed, double direction, double omega);
    void set_pid_gains(double kp, double ki, double kd);
    void speed_control(void *arg);

private:
    
    double speed_setPoint;
    double omega_setPoint;

    double _intError_speed;
	double _prevError_speed;
	double _intError_omega;
	double _prevError_omega;


    double NOMINAL_PGAIN;
	double KP_SPEED;
	double KI_SPEED;
	double KD_SPEED;
	double KP_OMEGA;
	double KI_OMEGA;
	double KD_OMEGA; 
};
