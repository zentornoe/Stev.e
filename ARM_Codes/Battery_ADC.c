#include "stm32f1xx_hal.h"

// Battery ADC : PA7, ADC2_CH7

uint32_t volt=0;
int Bat_val;

void BatteryChk(ADC_HandleTypeDef *hadc){
  HAL_ADC_Start(hadc);
  
  volt = HAL_ADC_GetValue(hadc);
  Bat_val = volt;
  
  printf("b%d", Bat_val);
}