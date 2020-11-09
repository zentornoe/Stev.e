#include "stm32f1xx_hal.h"

// RIGHT PWM : PA8, TIM1_CH1
// LEFT PWM : PA9, TIM1_CH2

extern int S_dir;

int S_direction[9]={3200, 3400, 3600, 3800, 4000, 4200, 4400, 4600, 4800};      // Min : 800, Max : 2200

void SolarTracker(){
  TIM1->CCR1 = S_direction[S_dir];
  TIM1->CCR2 = S_direction[S_dir];
  printf("T%d, ", S_dir+1);
}