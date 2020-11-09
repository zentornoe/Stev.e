#include "stm32f1xx_hal.h"

// Gimbal PWM : PA0, TIM2_CH1

extern int G_dir;

int G_direction[9] = {3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500};    // MIN : 3200, MAX : 4800

void GimbalCtrl(){
  TIM2->CCR1 = G_direction[G_dir];
  printf("G%d,", G_dir+1);
}