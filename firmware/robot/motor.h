#ifndef MOTOR_H
#define MOTOR_H

// Motor PWM control functions using TIM2.
// Adapted from TimerPWM example by Jesus Calvino-Fraga.
// See motor.c for pin assignments.

void Motor_Init(void);
void Motor_Left(int speed);   // -100 to 100
void Motor_Right(int speed);  // -100 to 100
void Motor_Stop(void);        // coast (both motors off)
void Motor_Brake(void);       // electronic brake (lock wheels)

#endif
