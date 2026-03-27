#include "PIDCALC.h"


void pid_init(pid_t* pid, float kp, float ki, float kd){
    //initialize the pid to inputted values(tested)
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    //initalize memory to 0
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
}

float pid_calc(pid_t* pid, float error){
    float P;//proportional kp*error
    float derivative;//error - prev.error

    P = (pid->Kp)*error;
    derivative = error - (pid->previous_error);
    pid->integral += error; //sum of the error

    //some protection in case robot stuck, wont let integral 
    //get huge, caps at 100, or whatever tested value we need
    if(pid->integral>100.0f){
        pid->integral = 100.0f;
    }
    if(pid->integral<-100.0f){
        pid->integral = -100.0f;
    }

    //finally update the previous error to the current input
    pid->previous_error = error;

    return P + (pid->integral * pid->Ki) + derivative*(pid->Kd);
}