#include "stm32f1xx_hal.h"


// CDS : PA6, ADC1_CH6

uint32_t cds_val[10];

extern int S_dir;

void SolarChk(ADC_HandleTypeDef *hadc){
  HAL_ADC_Start(hadc);
  int cds = HAL_ADC_GetValue(hadc);
  
  
  double CDS = (((double)(cds))*(3.30))/((double)4095);
  
  if(CDS<1.50){
    if(S_dir<9)
      S_dir++;          // Turn Right
  }
  else if(CDS>1.80){
    if(S_dir>0)
      S_dir--;          // Turn Left
  }
}