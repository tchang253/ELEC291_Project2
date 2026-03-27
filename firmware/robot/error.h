#ifndef ERROR_H
#define ERROR_H

#include <stdbool.h>  // Include the standard boolean library

#define ADC_MAX 4095.0 // Define the maximum ADC value for normalization
#define INTERSECTION_THRESHOLD 3500  //When interestion, the inductors will have a voltage spike
//from the extra current. Thus we will read this voltage spike with ADC as an intersection
//this value can be adjusted based on testing and calibration of the robot's sensors.

// Define a structure to hold the robot's error information
//the robot error is between -4095.0 to 4095.0 from the adc, we will normalize 
//it to -1.0 to 1.0 for easier processing.
typedef struct{
    float robot_error;//store the normalized error -1.0 to 1.0 
    bool intersection; // T/F if the robot at an intersection
}error_t;


//function prototype for an all in one. Since all the data needed is in the struct
//we just need one function to update the error and intersection status.
//this function will return a struct of type error_t with the updated error and intersection status.

error_t calc_error(int adc_left, int adc_right,int adc_middle);
//adc outputs integer values, however we will normalize to floats


#endif // ERROR_H