#include "error.h"
#include <stdbool.h>

error_t calc_error(int adc_left, int adc_right,int adc_middle){
    error_t error_current; //local struct to hold the current error and intersection status
    
    error_current.robot_error = (float)(adc_left - adc_right) / ADC_MAX; // normalize error to -1.0 to 1.0

    // Check for intersection based on the middle ADC value
    if(adc_middle > INTERSECTION_THRESHOLD){
        error_current.intersection = true; //intersection detected
    }
    else{
        error_current.intersection = false; //regular path, no intersection
    }

    return error_current;
}
