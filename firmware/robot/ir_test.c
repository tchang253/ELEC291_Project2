// ir_test.c — Standalone IR + motor test for STM32L051
// Flash with: make -f Makefile_ir_test
// Uses startup.c (initClock sets 32 MHz), serial.c, newlib_stubs.c
//
// Pin map (same as motor.c):
//   PA15 TIM2_CH1 AF5  Left  motor FWD
//   PB3  TIM2_CH2 AF2  Left  motor REV
//   PA2  TIM2_CH3 AF2  Right motor FWD
//   PA3  TIM2_CH4 AF2  Right motor REV
//   PA11 GPIO input    TSOP33338 IR receiver (EXTI11)
//
// IR protocol (matches REMOTE_IR_VERSION_WORKING.c):
//   Start mark : 2400 us  (accepted 1800-3200 us)
//   Start space: 600 us
//   Bit 1 mark : 1200 us  (>=900 us)
//   Bit 0 mark :  600 us  (<900 us)
//   Sync byte  : 0xAA
//   LSB first
//
// Commands: 0x01=fwd 0x02=bck 0x03=left 0x04=right 0x05=stop

#include "Common/Include/stm32l051xx.h"
#include <stdint.h>
#include <stdio.h>

// ---- Constants ----
#define F_CPU       32000000UL
#define IR_PIN      11U
#define SYNC_BYTE   0xAAU
#define SPEED       60

// ---- IR receiver state ----
typedef enum { IR_IDLE, IR_SYNC, IR_CMD } ir_state_t;

static volatile ir_state_t ir_state = IR_IDLE;
static volatile uint32_t   ir_t_fall = 0;
static volatile uint8_t    ir_bits = 0;
static volatile uint8_t    ir_count = 0;
static volatile uint8_t    ir_cmd = 0;
static volatile uint8_t    ir_new = 0;

// ---- Delay ----
static void wait_1ms(void) {
    SysTick->LOAD = (F_CPU / 1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {}
    SysTick->CTRL = 0;
}

static void waitms(int ms) {
    while (ms-- > 0) wait_1ms();
}

// ---- Motor init (copied from motor.c exactly) ----
static void Motor_Init(void) {
    RCC->IOPENR  |= BIT0 | BIT1;  // port A + B clocks
    RCC->APB1ENR |= BIT0;         // TIM2 clock

    // PA15 = TIM2_CH1 AF5 (Left FWD)
    GPIOA->OSPEEDR |= BIT30;
    GPIOA->OTYPER  &= ~BIT15;
    GPIOA->MODER    = (GPIOA->MODER & ~(BIT30)) | BIT31;
    GPIOA->AFR[1]  |= BIT30 | BIT28;  // AF5

    // PB3 = TIM2_CH2 AF2 (Left REV)
    GPIOB->OSPEEDR |= BIT6;
    GPIOB->OTYPER  &= ~BIT3;
    GPIOB->MODER    = (GPIOB->MODER & ~(BIT6)) | BIT7;
    GPIOB->AFR[0]  |= BIT13;  // AF2

    // ---- RIGHT MOTOR: TIM22_CH1 (PA6, FWD) + TIM22_CH2 (PA7, REV) ----
    // PA6/PA7 confirmed TIM22_CH1/CH2 at AF5 in STM32L051 datasheet.
    // PB0/PB1 have NO timer support on STM32L051.

    RCC->APB2ENR |= BIT5;   // TIM22 clock enable

    // PA6 = TIM22_CH1 AF5 (Right FWD)
    GPIOA->OSPEEDR |= BIT12 | BIT13;
    GPIOA->OTYPER  &= ~BIT6;
    GPIOA->MODER    = (GPIOA->MODER & ~(BIT12 | BIT13)) | BIT13; // AF mode
    GPIOA->AFR[0]  |= BIT26 | BIT24;   // AF5 (0101) for pin 6

    // PA7 = TIM22_CH2 AF5 (Right REV)
    GPIOA->OSPEEDR |= BIT14 | BIT15;
    GPIOA->OTYPER  &= ~BIT7;
    GPIOA->MODER    = (GPIOA->MODER & ~(BIT14 | BIT15)) | BIT15; // AF mode
    GPIOA->AFR[0]  |= BIT30 | BIT28;   // AF5 (0101) for pin 7

    // TIM22 PWM setup (same as TIM2)
    TIM22->PSC    = 319;
    TIM22->ARR    = 100;
    TIM22->CR1   |= BIT7;          // ARPE
    TIM22->CCMR1 |= BIT6  | BIT5;  // CH1 PWM mode 1
    TIM22->CCMR1 |= BIT3;          // OC1PE
    TIM22->CCMR1 |= BIT14 | BIT13; // CH2 PWM mode 1
    TIM22->CCMR1 |= BIT11;         // OC2PE
    TIM22->CCER  |= BIT0 | BIT4;   // CC1E + CC2E
    TIM22->CCR1   = 0;
    TIM22->CCR2   = 0;
    TIM22->EGR   |= BIT0;          // UG
    TIM22->CR1   |= BIT0;          // CEN

    // TIM2: PSC=319, ARR=100 → ~1 kHz PWM, duty 0-100
    TIM2->PSC    = 319;
    TIM2->ARR    = 100;
    TIM2->CR1   |= BIT7;   // ARPE
    TIM2->CR1   |= BIT0;   // CEN

    TIM2->CCMR1 |= BIT6  | BIT5;   // CH1 PWM mode 1
    TIM2->CCMR1 |= BIT3;           // OC1PE
    TIM2->CCMR1 |= BIT14 | BIT13;  // CH2 PWM mode 1
    TIM2->CCMR1 |= BIT11;          // OC2PE
    TIM2->CCMR2 |= BIT6  | BIT5;   // CH3 PWM mode 1
    TIM2->CCMR2 |= BIT3;           // OC3PE
    TIM2->CCMR2 |= BIT14 | BIT13;  // CH4 PWM mode 1
    TIM2->CCMR2 |= BIT11;          // OC4PE

    TIM2->CCER   |= BIT0 | BIT4;  // CC1E + CC2E only (left motor)

    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    TIM2->EGR |= BIT0;  // UG
}

static void Motor_Left(int speed) {
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;
    if (speed >= 0) {
        TIM2->CCR1 = speed;
        TIM2->CCR2 = 0;
    } else {
        TIM2->CCR1 = 0;
        TIM2->CCR2 = -speed;
    }
}

static void Motor_Right(int speed) {
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;
    if (speed >= 0) {
        TIM22->CCR1 = speed;  // PA6 = Right FWD
        TIM22->CCR2 = 0;
    } else {
        TIM22->CCR1 = 0;
        TIM22->CCR2 = -speed; // PA7 = Right REV
    }
}

static void Motor_Stop(void) {
    Motor_Left(0);
    Motor_Right(0);
}

// ---- TIM21: 1 us free-running for IR pulse measurement ----
static void TIM21_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
    TIM21->PSC = 31;      // 32 MHz / 32 = 1 MHz → 1 us tick
    TIM21->ARR = 0xFFFF;
    TIM21->CNT = 0;
    TIM21->CR1 = TIM_CR1_CEN;
}

// ---- EXTI11 for IR receiver ----
static void EXTI_IR_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PA11 as input with pull-up
    GPIOA->MODER &= ~(3U << (IR_PIN * 2));
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(3U << (IR_PIN * 2)))
                  | (1U << (IR_PIN * 2));

    // Route EXTI11 to port A
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(0xFU << 12)) | (0U << 12);

    EXTI->IMR  |= (1U << IR_PIN);  // unmask
    EXTI->FTSR |= (1U << IR_PIN);  // falling edge
    EXTI->RTSR |= (1U << IR_PIN);  // rising edge

    NVIC_SetPriority(EXTI4_15_IRQn, 0);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

// ---- IR ISR ----
// NOTE: startup.c vector table uses EXTI4_15_Handler (not IRQHandler!)
void EXTI4_15_Handler(void) {
    if (!(EXTI->PR & (1U << IR_PIN))) return;
    EXTI->PR = (1U << IR_PIN);  // clear pending

    if (!(GPIOA->IDR & (1U << IR_PIN))) {
        // Falling edge → mark begins
        ir_t_fall = TIM21->CNT;
    } else {
        // Rising edge → mark ends
        uint16_t mark = (uint16_t)(TIM21->CNT - ir_t_fall);

        if (ir_state == IR_IDLE) {
            // Only accept start mark (2400us nominal) when idle
            if (mark >= 1800U && mark <= 3200U) {
                ir_state = IR_SYNC;
                ir_bits  = 0;
                ir_count = 0;
            }
        } else {
            // Receiving data bits
            uint8_t bit = (mark >= 900U) ? 1U : 0U;
            ir_bits |= (uint8_t)(bit << ir_count);
            ir_count++;

            if (ir_count == 8U) {
                if (ir_state == IR_SYNC) {
                    ir_state = (ir_bits == SYNC_BYTE) ? IR_CMD : IR_IDLE;
                } else {
                    ir_cmd = ir_bits;
                    ir_new = 1;
                    ir_state = IR_IDLE;
                }
                ir_bits  = 0;
                ir_count = 0;
            }
        }
    }
}

// ---- Timeout: reset state machine if stuck ----
static void ir_check_timeout(void) {
    if (ir_state != IR_IDLE) {
        uint16_t elapsed = (uint16_t)(TIM21->CNT - ir_t_fall);
        if (elapsed > 40000U) {  // 40 ms
            ir_state = IR_IDLE;
        }
    }
}

// ---- Main ----
void main(void) {
    // startup.c already called initClock() → 32 MHz
    // startup.c already called initUART() → 115200 baud on PA9/PA10

    Motor_Init();
    TIM21_Init();
    EXTI_IR_Init();
    Motor_Stop();

    printf("\r\n=== IR + MOTOR TEST ===\r\n");
    printf("Waiting for IR commands from remote...\r\n");

    while (1) {
        ir_check_timeout();

        if (!ir_new) continue;

        uint8_t cmd = ir_cmd;
        ir_new = 0;

        switch (cmd) {
            case 0x01: Motor_Left( SPEED); Motor_Right( SPEED); printf("FWD\r\n");   break;
            case 0x02: Motor_Left(-SPEED); Motor_Right(-SPEED); printf("BCK\r\n");   break;
            case 0x03: Motor_Left(-SPEED); Motor_Right( SPEED); printf("LEFT\r\n");  break;
            case 0x04: Motor_Left( SPEED); Motor_Right(-SPEED); printf("RIGHT\r\n"); break;
            case 0x05: Motor_Stop();                            printf("STOP\r\n");  break;
            case 0x06: Motor_Left(-SPEED); Motor_Right( SPEED); waitms(600); Motor_Stop(); printf("ROT180\r\n"); break;
            case 0x08: printf("AUTO TOGGLE RECVD\r\n");         break;
            case 0x09: Motor_Left( SPEED); Motor_Right( SPEED/2); printf("FWD-R\r\n"); break;
            case 0x0A: Motor_Left( SPEED/2); Motor_Right( SPEED); printf("FWD-L\r\n"); break;
            case 0x0B: Motor_Left(-SPEED); Motor_Right(-SPEED/2); printf("BCK-R\r\n"); break;
            case 0x0C: Motor_Left(-SPEED/2); Motor_Right(-SPEED); printf("BCK-L\r\n"); break;
            default:   printf("CMD=0x%02X\r\n", cmd);           break;
        }
    }
}
