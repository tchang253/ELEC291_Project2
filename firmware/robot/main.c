#include "motor.h"
#include "error.h"
#include "PIDCALC.h"
#include "Common/Include/stm32l051xx.h"

// --- TUNING CONSTANTS ---
#define START_KP 1.5f   // Start small, increase until it wiggles
#define START_KI 0.0f   // Keep at 0 for initial testing
#define START_KD 0.2f   // Increase to stop the wiggling
#define BASE_SPEED 50  // 0-100% duty cycle

// --- ADC CHANNELS (Based on LQFP32 Pinout) ---
#define CH_LEFT   0     // PA0 (Pin 6)
#define CH_RIGHT  1     // PA1 (Pin 7)
#define CH_MIDDLE 4     // PA4 (Pin 10)

// Function Prototypes
void ADC_Init(void);
int Read_ADC(int channel);

int main(void) {
    // 1. Hardware Init
    Motor_Init();
    ADC_Init();
    
    // 2. PID Memory Setup
    pid_t steering_pid;
    pid_init(&steering_pid, START_KP, START_KI, START_KD);
    
    // 3. Status Struct
    error_t current_state;
    
    while(1) {
        // A. SENSE: Read the 12-bit integers from the inductors
        int L = Read_ADC(CH_LEFT);
        int R = Read_ADC(CH_RIGHT);
        int M = Read_ADC(CH_MIDDLE);
        
        // B. PROCESS: Normalize to -1.0 to 1.0 using your error.c
        current_state = calc_error(L, R, M);
        
        // C. COMPUTE: Calculate PID adjustment using your PIDCALC.c
        float adjustment = pid_calc(&steering_pid, current_state.robot_error);
        
        // D. ACT: Adjust PWM duty cycles (0-100 range)
        int left_speed  = BASE_SPEED - (int)adjustment;
        int right_speed = BASE_SPEED + (int)adjustment;
        
        Motor_Left(left_speed);
        Motor_Right(right_speed);
        
        // E. SAFETY: Simple intersection stop for testing
        if (current_state.intersection) {
            // Uncomment to stop at first cross-wire
            // Motor_Brake(); 
            // while(1); 
        }
    }
}

// --- ADC DRIVER IMPLEMENTATION ---

void ADC_Init(void) {
    // Enable GPIOA and ADC1 Clocks
    RCC->IOPENR  |= BIT0; 
    RCC->APB2ENR |= BIT9; 

    // Configure PA0, PA1, PA4 as Analog (Mode 11)
    GPIOA->MODER |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT8 | BIT9);

    // ADC Calibration Sequence
    if ((ADC1->CR & BIT0) != 0) ADC1->CR |= BIT1; // Ensure ADEN=0
    while ((ADC1->CR & BIT0) != 0);
    
    ADC1->CR |= BIT31;           // Start ADCAL
    while (ADC1->CR & BIT31);    // Wait for calibration
    
    // Set Clock Source (Internal VREF is ~3.3V)
    ADC1->CFGR2 |= BIT30;        // Use PCLK/2
}

int Read_ADC(int channel) {
    ADC1->CHSELR = (1 << channel); // Select channel bit
    
    ADC1->CR |= BIT0;              // Enable ADC (ADEN)
    while (!(ADC1->ISR & BIT0));   // Wait for ADRDY

    ADC1->CR |= BIT2;              // Start Conversion (ADSTART)
    while (!(ADC1->ISR & BIT2));   // Wait for EOC

    return ADC1->DR;               // Result is 0-4095
}