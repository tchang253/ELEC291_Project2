// motor.c
// Controls the two main drive motors.
// LEFT motor  uses TIM2  (PA15=FWD/CH1/AF5, PB3=REV/CH2/AF2)
// RIGHT motor uses TIM22 (PA6=FWD/CH1/AF5,  PA7=REV/CH2/AF5)
//
// PA2 and PA3 are reserved for Bluetooth USART2 (TX/RX).
// PB0 and PB1 have NO timer support on STM32L051 — do not use for PWM.
//
// LQFP32 Pinout Diagram
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7  (RESERVED for VL53L0X I2C1_SDA)
//      NRST -|4       29|- PB6  (RESERVED for VL53L0X I2C1_SCL)
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3  < L_REV (TIM2_CH2, AF2)
//(USART2_TX)PA2 -|8  25|- PA15 < L_FWD (TIM2_CH1, AF5)
//(USART2_RX)PA3 -|9  24|- PA14
//       PA4 -|10      23|- PA13
//       PA5 -|11      22|- PA12
//(R_FWD)PA6 -|12      21|- PA11
//(R_REV)PA7 -|13      20|- PA10 (Reserved for USART1_RXD)
//       PB0 -|14      19|- PA9  (Reserved for USART1_TXD)
//       PB1 -|15      18|- PA8
//       VSS -|16      17|- VDD
//             ----------
//
// Pin Summary:
// Left  Motor FWD -> PA15 (Pin 25) [TIM2_CH1  / AF5]
// Left  Motor REV -> PB3  (Pin 26) [TIM2_CH2  / AF2]
// Right Motor FWD -> PA6  (Pin 12) [TIM22_CH1 / AF5]
// Right Motor REV -> PA7  (Pin 13) [TIM22_CH2 / AF5]
//
// Hardware Notes for the team:
// - We use 2 PWM signals per motor. Wire the FWD signal to the left half-bridge
// gates,
//   and the REV signal to the right half-bridge gates (matches the 2026 lecture
//   slides).
//
// - Motor_Stop() sets both inputs to 0. This turns on both bottom MOSFETs,
// connecting
//   both motor terminals directly to ground. This safely dissipates the motor's
//   energy and gives us a hard electronic brake (which helps us stop exactly on
//   intersections).
//
// - CAUTION: Setting both FWD and REV high at the same time causes
// "shoot-through".
//   This creates a direct short from VCC to Ground and will destroy the
//   H-bridge. However, our software specifically prevents this in Motor_Left()
//   and Motor_Right() by forcing one channel to 0 whenever the other is active.

#include "motor.h"
#include "Common/Include/stm32l051xx.h"

#define SYSCLK 32000000L

void Motor_Init(void) {
  RCC->IOPENR  |= BIT0 | BIT1; // port A + B clocks
  RCC->APB1ENR |= BIT0;        // TIM2  clock (APB1 bit 0)
  RCC->APB2ENR |= BIT5;        // TIM22 clock (APB2 bit 5)

  // ---- LEFT MOTOR: TIM2_CH1 (PA15, FWD) + TIM2_CH2 (PB3, REV) ----

  // PA15 = TIM2_CH1 AF5 (Left FWD)
  GPIOA->OSPEEDR |= BIT30 | BIT31;              // medium speed (2 bits)
  GPIOA->OTYPER  &= ~BIT15;                     // push-pull
  GPIOA->MODER    = (GPIOA->MODER & ~BIT30) | BIT31; // AF mode
  GPIOA->AFR[1]  |= BIT30 | BIT28;             // AF5 (0101)

  // PB3 = TIM2_CH2 AF2 (Left REV)
  GPIOB->OSPEEDR |= BIT6 | BIT7;               // medium speed (2 bits)
  GPIOB->OTYPER  &= ~BIT3;                      // push-pull
  GPIOB->MODER    = (GPIOB->MODER & ~BIT6) | BIT7; // AF mode
  GPIOB->AFR[0]  |= BIT13;                     // AF2 (0010)

  // TIM2: CH1=PA15(FWD), CH2=PB3(REV) — same 1 kHz PWM
  TIM2->PSC    = 319;
  TIM2->ARR    = 100;
  TIM2->CR1   |= BIT7;          // ARPE
  TIM2->CCMR1 |= BIT6  | BIT5;  // CH1 PWM mode 1
  TIM2->CCMR1 |= BIT3;          // OC1PE
  TIM2->CCMR1 |= BIT14 | BIT13; // CH2 PWM mode 1
  TIM2->CCMR1 |= BIT11;         // OC2PE
  TIM2->CCER  |= BIT0 | BIT4;   // CC1E + CC2E
  TIM2->CCR1   = 0;
  TIM2->CCR2   = 0;
  TIM2->EGR   |= BIT0;          // UG
  TIM2->CR1   |= BIT0;          // CEN

  // ---- RIGHT MOTOR: TIM22_CH1 (PA6, FWD) + TIM22_CH2 (PA7, REV) ----
  // PA6/PA7 are confirmed TIM22_CH1/CH2 at AF5 in the STM32L051 datasheet.
  // PB0/PB1 have NO timer support on STM32L051 — verified from HAL gpio_ex.h.

  // PA6 = TIM22_CH1 AF5 (Right FWD)
  GPIOA->OSPEEDR |= BIT12 | BIT13;              // medium speed (2 bits for pin 6)
  GPIOA->OTYPER  &= ~BIT6;                      // push-pull
  GPIOA->MODER    = (GPIOA->MODER & ~(BIT12 | BIT13)) | BIT13; // AF mode
  GPIOA->AFR[0]  |= BIT26 | BIT24;             // AF5 (0101) for pin 6

  // PA7 = TIM22_CH2 AF5 (Right REV)
  GPIOA->OSPEEDR |= BIT14 | BIT15;              // medium speed (2 bits for pin 7)
  GPIOA->OTYPER  &= ~BIT7;                      // push-pull
  GPIOA->MODER    = (GPIOA->MODER & ~(BIT14 | BIT15)) | BIT15; // AF mode
  GPIOA->AFR[0]  |= BIT30 | BIT28;             // AF5 (0101) for pin 7

  // TIM22: same 1 kHz PWM settings
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
}

// speed: -100 to 100. Positive=forward, negative=reverse, 0=coast.
// We never set both fwd+rev high at same time (would short the H-bridge)
void Motor_Left(int speed) {
  if (speed > 100)
    speed = 100;
  if (speed < -100)
    speed = -100;

  if (speed >= 0) {
    TIM2->CCR1 = speed; // CH1 fwd
    TIM2->CCR2 = 0;     // CH2 rev off
  } else {
    TIM2->CCR1 = 0;
    TIM2->CCR2 = -speed;
  }
}

void Motor_Right(int speed) {
  if (speed > 100)  speed = 100;
  if (speed < -100) speed = -100;

  if (speed >= 0) {
    TIM22->CCR1 = speed; // CH1 = PA6 = Right FWD
    TIM22->CCR2 = 0;
  } else {
    TIM22->CCR1 = 0;
    TIM22->CCR2 = -speed; // CH2 = PA7 = Right REV
  }
}

void Motor_Stop(void) {
  Motor_Left(0);
  Motor_Right(0);
}

