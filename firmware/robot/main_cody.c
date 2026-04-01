// main_cody.c  --  Mock PID control test via PuTTY
// Type three comma-separated ADC values into PuTTY
// and the robot will immediately drive the motors
// through the PID loop.
//
// NOTE: startup.c already calls initUART(115200) for us
//       when compiled with -DUSE_USART1. Do NOT init
//       USART1 again here or the interrupts will fight
//       the polling code and eat your keystrokes.

#include "Common/Include/serial.h"
#include "Common/Include/stm32l051xx.h"
#include "PIDCALC.h"
#include "error.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>

int main(void) {
  // 1. Hardware Setup
  //    UART is already running from startup.c (initUART(115200))
  Motor_Init();

  // 2. Control Setup
  pid_ctrl_t steering_pid;
  pid_init(&steering_pid, 25.0f, 0.0f, 5.0f); // Tuning values

  int L, R, M;
  error_t current_state;
  char buf[64];

  while (1) {
    // --- STEP A: WAIT FOR PUTTY INPUT ---
    printf("\r\nREADY> Enter L,R,M (e.g. 2000,1000,500): ");
    fflush(stdout);

    // egets_echo reads from the interrupt-driven RX buffer
    // AND echoes each character back so you can see what you type
    egets_echo(buf, sizeof(buf));

    // Parse the CSV
    if (sscanf(buf, "%d,%d,%d", &L, &R, &M) != 3) {
      printf("\r\n** Bad format, try again **\r\n");
      continue;
    }

    printf("\r\n[GOT] L=%d  R=%d  M=%d\r\n", L, R, M);

    // --- STEP B: RUN YOUR LOGIC ---
    current_state = calc_error(L, R, M);
    float adjustment = pid_calc(&steering_pid, current_state.robot_error);

    // --- STEP C: DRIVE MOTORS ---
    int l_speed = 40 - (int)adjustment;
    int r_speed = 40 + (int)adjustment;

    // Clamp 0-100
    if (l_speed < 0)
      l_speed = 0;
    if (l_speed > 100)
      l_speed = 100;
    if (r_speed < 0)
      r_speed = 0;
    if (r_speed > 100)
      r_speed = 100;

    printf("[PID] adj=%.1f  => L_motor=%d%%  R_motor=%d%%\r\n",
           (double)adjustment, l_speed, r_speed);

    Motor_Left(l_speed);
    Motor_Right(r_speed);
  }
}
