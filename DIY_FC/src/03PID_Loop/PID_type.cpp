#include <Arduino.h>
#include <PID_type.h>
#include <Var_types.h>

// attitude_t angle; // Attitude
// attitude_t angle_des; // Desired attitude
// attitude_t rate; // Attitude rate
// attitude_t rate_des; // Desired attitude rate
attitude_t angle_err; // Attitude error
attitude_t rate_err; // Attitude rate error

// attitude_t angle_err_prev; // Previous attitude error
// attitude_t rate_err_prev; // Previous attitude rate error

PID_out_t rate_out, stab_out; // Output of rate and stabilization controllers

PID_Params_t rate_params; // PID parameters for rate controller
PID_Params_t stab_params; // PID parameters for stabilization controller


// Initialization of PID parameters. Need to run at setup in the main code.
void initializePIDParams(float RrollPID[3]={0.1,0.01,0.01}, float RyawPID[3]={0.1,0.01,0.01}, float Imax_rate[2]={100.0f,100.0f}, float SrollPID[3]={0.1,0.01,0.01}, float SyawPID[3]={0.1,0.01,0.01}, float Imax_stab[2]={100.0f,100.0f}) {
    // Rate mode parameters
    rate_params.RollP = RrollPID[0];
    rate_params.RollI = RrollPID[1];
    rate_params.RollD = RrollPID[2];
    rate_params.PitchP = rate_params.RollP;
    rate_params.PitchI = rate_params.RollI;
    rate_params.PitchD = rate_params.RollD;
    rate_params.YawP = RyawPID[0];
    rate_params.YawI = RyawPID[1];
    rate_params.YawD = RyawPID[2];
    rate_params.Imax_roll = Imax_rate[0];
    rate_params.Imax_pitch = rate_params.Imax_roll;
    rate_params.Imax_yaw = Imax_rate[1];

    // Stabilize mode parameters
    stab_params.RollP = SrollPID[0];
    stab_params.RollI = SrollPID[1];
    stab_params.RollD = SrollPID[2];
    stab_params.PitchP = stab_params.RollP;
    stab_params.PitchI = stab_params.RollI;
    stab_params.PitchD = stab_params.RollD;
    stab_params.YawP = SyawPID[0];
    stab_params.YawI = SyawPID[1];
    stab_params.YawD = SyawPID[2];
    stab_params.Imax_roll = Imax_stab[0];
    stab_params.Imax_pitch = stab_params.Imax_roll;
    stab_params.Imax_yaw = Imax_stab[1];
}

// PID controller for rate
attitude_t PID_rate(attitude_t des_rate, attitude_t rate, float DT) {
    // Calculate error
    rate_err = des_rate - rate;

    // Calculate P term:
    rate_out.P_term.roll = rate_params.RollP * rate_err.roll;
    rate_out.P_term.pitch = rate_params.PitchP * rate_err.pitch;
    rate_out.P_term.yaw = rate_params.YawP * rate_err.yaw;

    // Calculate I term:
    rate_out.I_term.roll = rate_out.prev_Iterm.roll + rate_params.RollI * rate_err.roll*DT;
    rate_out.I_term.pitch = rate_out.prev_Iterm.pitch + rate_params.PitchI * rate_err.pitch*DT;
    rate_out.I_term.yaw = rate_out.prev_Iterm.yaw + rate_params.YawI * rate_err.yaw*DT;

    // Calculate D term:
    rate_out.D_term.roll = rate_params.RollD * (rate_err.roll - rate_out.prev_err.roll)/DT;
    rate_out.D_term.pitch = rate_params.PitchD * (rate_err.pitch - rate_out.prev_err.pitch)/DT;
    rate_out.D_term.yaw = rate_params.YawD * (rate_err.yaw - rate_out.prev_err.yaw)/DT;

    // Cap the I term
    rate_out.I_term.roll = constrain(rate_out.I_term.roll, -rate_params.Imax_roll, rate_params.Imax_roll);
    rate_out.I_term.pitch = constrain(rate_out.I_term.pitch, -rate_params.Imax_pitch, rate_params.Imax_pitch);
    rate_out.I_term.yaw = constrain(rate_out.I_term.yaw, -rate_params.Imax_yaw, rate_params.Imax_yaw);

    // Time propagation for relevant variables:
    rate_out.prev_err = rate_err;
    rate_out.prev_Iterm = rate_out.I_term;

    // Return the output
    rate_out.PID_ret = rate_out.P_term + rate_out.I_term + rate_out.D_term;
    return rate_out.PID_ret; // This is the motor input values
}

// PID controller for stabilization
attitude_t PID_stab(attitude_t des_angle, attitude_t angle, float DT) {
    // Calculate error
    angle_err = des_angle - angle;

    // Calculate P term:
    stab_out.P_term.roll = stab_params.RollP * angle_err.roll;
    stab_out.P_term.pitch = stab_params.PitchP * angle_err.pitch;
    stab_out.P_term.yaw = stab_params.YawP * angle_err.yaw;

    // Calculate I term:
    stab_out.I_term.roll = stab_out.prev_Iterm.roll + stab_params.RollI * angle_err.roll*DT;
    stab_out.I_term.pitch = stab_out.prev_Iterm.pitch + stab_params.PitchI * angle_err.pitch*DT;
    stab_out.I_term.yaw = stab_out.prev_Iterm.yaw + stab_params.YawI * angle_err.yaw*DT;

    // Calculate D term:
    stab_out.D_term.roll = stab_params.RollD * (angle_err.roll - stab_out.prev_err.roll)/DT;
    stab_out.D_term.pitch = stab_params.PitchD * (angle_err.pitch - stab_out.prev_err.pitch)/DT;
    stab_out.D_term.yaw = stab_params.YawD * (angle_err.yaw - stab_out.prev_err.yaw)/DT;

    // Cap the I term
    stab_out.I_term.roll = constrain(stab_out.I_term.roll, -stab_params.Imax_roll, stab_params.Imax_roll);
    stab_out.I_term.pitch = constrain(stab_out.I_term.pitch, -stab_params.Imax_pitch, stab_params.Imax_pitch);
    stab_out.I_term.yaw = constrain(stab_out.I_term.yaw, -stab_params.Imax_yaw, stab_params.Imax_yaw);

    // Time propagation for relevant variables:
    stab_out.prev_err = angle_err;
    stab_out.prev_Iterm = stab_out.I_term;

    // Return the output
    stab_out.PID_ret = stab_out.P_term + stab_out.I_term + stab_out.D_term;
    

    return stab_out.PID_ret; // This output is the desired rate. now we can use the PID_rate function to get the motor input values
}
