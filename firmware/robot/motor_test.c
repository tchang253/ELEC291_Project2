// motor_test.c
// Simple test program to verify H-bridge and motor wiring.
// It outputs an automated sequence (fwd, rev, left, right) for both motors
// and prints the state to the serial port (115200 baud).
//

#include "Common/Include/serial.h"
#include "Common/Include/stm32l051xx.h"
#include "motor.h"
#include <stdio.h>

#define F_CPU 32000000L

void wait_1ms(void) {
  SysTick->LOAD = (F_CPU / 1000L) - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
  while ((SysTick->CTRL & BIT16) == 0)
    ;
  SysTick->CTRL = 0x00;
}

void waitms(int len) {
  while (len--)
    wait_1ms();
}

void print_status(void) {
  printf("  PWMs -> L_fwd:%u  L_rev:%u  R_fwd:%u  R_rev:%u\r\n", TIM2->CCR1,
         TIM2->CCR2, TIM2->CCR3, TIM2->CCR4);
}

void ramp_both_motors(void) {
  int i;
  printf("7. RAMPING: Both motors 0 -> 100 -> 0\r\n");
  for (i = 0; i <= 100; i += 10) {
    Motor_Left(i);
    Motor_Right(i);
    print_status();
    waitms(200);
  }
  for (i = 100; i >= 0; i -= 10) {
    Motor_Left(i);
    Motor_Right(i);
    print_status();
    waitms(200);
  }
  Motor_Stop();
  printf("   RAMP DONE\r\n");
}

int main(void) {
  waitms(500);

  printf("\x1b[2J\x1b[1;1H");
  printf("=== MOTOR HARDWARE TEST ===\r\n");
  printf("Running automated sequence...\r\n\r\n");

  Motor_Init();
  printf("Motor_Init() done. Motors off.\r\n");
  print_status();

  while (1) {
    printf("\r\n--- STARTING AUTO TEST SEQUENCE ---\r\n");

    printf("1. Both Motors Forward (50%%) for 2 seconds\r\n");
    Motor_Left(50);
    Motor_Right(50);
    print_status();
    waitms(2000);

    printf("2. Both Motors Reverse (50%%) for 2 seconds\r\n");
    Motor_Left(-50);
    Motor_Right(-50);
    print_status();
    waitms(2000);

    printf("3. Stop for 1 second\r\n");
    Motor_Stop();
    print_status();
    waitms(1000);

    printf("4. Left Motor ONLY Forward (50%%) for 2 seconds\r\n");
    Motor_Left(50);
    Motor_Right(0);
    print_status();
    waitms(2000);

    printf("5. Right Motor ONLY Forward (50%%) for 2 seconds\r\n");
    Motor_Left(0);
    Motor_Right(50);
    print_status();
    waitms(2000);

    printf("6. Stop for 2 seconds before repeating...\r\n");
    Motor_Stop();
    print_status();
    waitms(2000);

    ramp_both_motors();
    waitms(2000);
  }
  return 0;
}
