#pragma once
#include "ch.h"
#include "utils.h"

class PID {

public:
    
    void init(double int_max) {
        lastTime = chVTGetSystemTime();
        setpoint = 0;
        error_acc = 0;
        nominal_gain = 0;
        kp = 0;
        ki = 0;
        kd = 0;
        max_acc = int_max;
    }

    /**
     *  value: latest measure
     *  return: new command
     */
    double update(double value) {
        systime_t now = chVTGetSystemTime();
        double elapsed = ((double)(now - lastTime) / CH_CFG_ST_FREQUENCY);
        lastTime = now;

        double error = setpoint - value;
        error_acc += error/elapsed;
        error_acc = clamp(-max_acc, error_acc, max_acc);

        // TODO KD
        double cmd = nominal_gain * setpoint + kp * error + ki * error_acc;
        cmd = clamp(-100, cmd, 100);

        return cmd;
    }


    void set_setpoint(double sp) {
        setpoint = sp;
    }

    void set_gains(double ng, double p, double i, double d) {
        nominal_gain = ng;
        kp = p;
        ki = i;
        kd = d;
    }

    double get_setpoint() {
        return setpoint;
    }

private:
    double setpoint;

    double error_acc;

    systime_t lastTime;

    double nominal_gain;
    double kp;
    double ki;
    double kd;
    double max_acc;

};
