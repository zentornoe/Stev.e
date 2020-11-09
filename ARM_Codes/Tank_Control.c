#include "stm32f1xx_hal.h"

// RIGHT_PWM : PA10, TIM1_CH3
// LEFT PWM : PA11, TIM1_CH4
// RIGHT DR : PB1, GPIO_Output
// LEFT DR : PB2, GPIO_Output

#define STOP 0
#define M_MID 20000             //min : 0, max : 20000
#define M_ROT 4000              // Rotation speed

void TankSt(){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // RIGHT DR : ST
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   // LEFT DR : ST
  TIM1->CCR3=M_MID;                            // RIGHT
  TIM1->CCR4=M_MID;                            // LEFT
  printf("W");
 }

void TankRight(){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // RIGHT DR ST
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // LEFT DR REVERSE
  TIM1->CCR3=M_ROT;       // RIGHT
  TIM1->CCR4=M_ROT;       // LEFT
  printf("R");
}

void TankLeft(){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);    // RIGHT DR REVERSE
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // LEFT DR ST
  TIM1->CCR3=M_ROT;       // RIGHT
  TIM1->CCR4=M_ROT;       // LEFT
  printf("L");
}

void TankBack(){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);   // RIGHT DR : REVERSE
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);   // LEFT DR : REVERSE
  TIM1->CCR3=M_ROT;     // RIGHT
  TIM1->CCR4=M_ROT;     // LEFT
  printf("S");
}

void TankStop(){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // RIGHT DR ST
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // LEFT DR ST
  TIM1->CCR3=STOP;         // RIGHT
  TIM1->CCR4=STOP;         // LEFT
  printf("X");
}