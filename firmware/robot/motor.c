// motor.c
// Controls the two main drive motors using TIM2 PWM.
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
// (R_FWD)PA2 -|8       25|- PA15 < L_FWD (TIM2_CH1, AF5)
// (R_REV)PA3 -|9       24|- PA14
//       PA4 -|10      23|- PA13
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10 (Reserved for RXD)
//       PB0 -|14      19|- PA9  (Reserved for TXD)
//       PB1 -|15      18|- PA8
//       VSS -|16      17|- VDD
//             ----------
//
// Pin Summary:
// Left Motor FWD  -> PA15 (Pin 25) [TIM2_CH1 / AF5]
// Left Motor REV  -> PB3  (Pin 26) [TIM2_CH2 / AF2]
// Right Motor FWD -> PA2  (Pin 8)  [TIM2_CH3 / AF2] - double check these pins
// on the actual board! Right Motor REV -> PA3  (Pin 9)  [TIM2_CH4 / AF2]
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
  RCC->IOPENR |= BIT0 | BIT1; // peripheral clock enable for port A and B
  RCC->APB1ENR |= BIT0;       // turn on clock for timer2

  // Configure PA15 for alternate function (TIM2_CH1, pin 25 in LQFP32 package)
  // Copied from TimerPWM example, same pin same AF
  GPIOA->OSPEEDR |= BIT30;                          // MEDIUM SPEED
  GPIOA->OTYPER &= ~BIT15;                          // Push-pull
  GPIOA->MODER = (GPIOA->MODER & ~(BIT30)) | BIT31; // AF-Mode
  GPIOA->AFR[1] |= BIT30 | BIT28;                   // AF5 (table 16)

  // Configure PB3 for alternate function (TIM2_CH2)
  GPIOB->OSPEEDR |= BIT6;
  GPIOB->OTYPER &= ~BIT3;
  GPIOB->MODER = (GPIOB->MODER & ~(BIT6)) | BIT7; // AF-Mode
  GPIOB->AFR[0] |= BIT13;                         // AF2 (0010)

  // Configure PA2 for alternate function (TIM2_CH3)
  GPIOA->OSPEEDR |= BIT4;
  GPIOA->OTYPER &= ~BIT2;
  GPIOA->MODER = (GPIOA->MODER & ~(BIT4)) | BIT5; // AF-Mode
  GPIOA->AFR[0] |= BIT9;                          // AF2 (0010)

  // Configure PA3 for alternate function (TIM2_CH4)
  GPIOA->OSPEEDR |= BIT6;
  GPIOA->OTYPER &= ~BIT3;
  GPIOA->MODER = (GPIOA->MODER & ~(BIT6)) | BIT7; // AF-Mode
  GPIOA->AFR[0] |= BIT13;                         // AF2 (0010)

  // Timer 2 setup
  // We are using a prescaler of 319 to drop the PWM frequency down to ~1kHz.
  // This is SUPER IMPORTANT. The LTV-847 optocouplers we're using take 5-10us
  // to turn off. When we ran this at 20kHz earlier, they couldn't switch fast
  // enough, causing shoot-through that fried the MOSFETs. Keep this at ~1kHz to
  // be safe.
  TIM2->PSC = 319;
  TIM2->ARR = 100;   // period=100 so duty cycle maps cleanly from 0-100%
  TIM2->CR1 |= BIT7; // auto-reload preload enable
  TIM2->CR1 |= BIT0; // enable the counter

  // Enable PWM mode 1 on all 4 channels ([6..4]=110, OC preload)
  // We're upcounting now. CCR=0 means 0% duty cycle (OFF), CCR=100 means full
  // speed.
  TIM2->CCMR1 |= BIT6 | BIT5;   // CH1 PWM mode 1
  TIM2->CCMR1 |= BIT3;          // OC1PE
  TIM2->CCMR1 |= BIT14 | BIT13; // CH2 PWM mode 1
  TIM2->CCMR1 |= BIT11;         // OC2PE
  TIM2->CCMR2 |= BIT6 | BIT5;   // CH3 PWM mode 1
  TIM2->CCMR2 |= BIT3;          // OC3PE
  TIM2->CCMR2 |= BIT14 | BIT13; // CH4 PWM mode 1
  TIM2->CCMR2 |= BIT11;         // OC4PE

  // Enable output compare on all 4 channels
  TIM2->CCER |= BIT0;  // CC1E
  TIM2->CCER |= BIT4;  // CC2E
  TIM2->CCER |= BIT8;  // CC3E
  TIM2->CCER |= BIT12; // CC4E

  // Start with motors off
  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;
  TIM2->CCR3 = 0;
  TIM2->CCR4 = 0;
  TIM2->EGR |= BIT0; // UG=1
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
  if (speed > 100)
    speed = 100;
  if (speed < -100)
    speed = -100;

  if (speed >= 0) {
    TIM2->CCR3 = speed; // CH3 fwd
    TIM2->CCR4 = 0;     // CH4 rev off
  } else {
    TIM2->CCR3 = 0;
    TIM2->CCR4 = -speed;
  }
}

void Motor_Stop(void) {
  Motor_Left(0);
  Motor_Right(0);
}

