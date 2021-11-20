#pragma once
#include "ch.h"
#include "utils.h"

class PID {

public:
    
    void init(double int_max_cmd, double accel_max) {
        lastTime = chVTGetSystemTime();
        setpoint = 0;
        precommand = 0;
        error_acc = 0;
        feedforward_gain = 0;
        kp = 0;
        ki = 0;
        kd = 0;
        max_int_cmd = int_max_cmd;
        max_accel = accel_max;
    }

    /**
     *  value: latest measure
     *  return: new command
     */
    double update(double value) {

        

        double elapsed = chTimeMS2I(chVTTimeElapsedSinceX(lastTime))/1000.0;
        lastTime = chVTGetSystemTime();

        precommand = clamp(precommand-max_accel*elapsed, setpoint, precommand+max_accel*elapsed);
        double error = precommand - value;

        error_acc += error*elapsed;
        //saturate error such that the contribution of integrator to the cmd cannot exceed max_int_cmd.
        error_acc = clamp(-max_int_cmd/ki, error_acc, max_int_cmd/ki);

        // TODO KD
        double cmd = feedforward_gain * precommand + kp * error + ki * error_acc;
        cmd = clamp(-100, cmd, 100);

        return cmd;
    }


    void set_setpoint(double sp) {
        setpoint = sp;
    }

    void set_gains(double ng, double p, double i, double d) {
        feedforward_gain = ng;
        kp = p;
        ki = i;
        kd = d;
    }

    double get_setpoint() {
        return setpoint;
    }

    double get_precommand() {
        return precommand;
    }

private:
    double setpoint;
    double precommand;

    double error_acc;

    systime_t lastTime;

    double feedforward_gain;
    double kp;
    double ki;
    double kd;
    double max_int_cmd;
    double max_accel;

};
