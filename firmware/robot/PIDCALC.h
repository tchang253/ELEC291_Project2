#ifndef PIDCALC_H
#define PIDCALC_H

//for pid, we need a struct to hold the pid parameters and state
typedef struct{
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
    float previous_error; // Previous error for derivative calculation
    float integral; // Integral of the error for integral calculation
}pid_ctrl_t;

//function prototype for pid initialization, we will set the initial values of the pid gains, and prev error and integral to 0
void pid_init(pid_ctrl_t* pid, float kp, float ki, float kd);

//pid calculation
float pid_calc(pid_ctrl_t* pid, float error);


#endif