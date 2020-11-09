#include "stm32f1xx_hal.h"

// Relay : PC10, GPIO_Output

void RelayCtrl(){
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)==GPIO_PIN_RESET){     // if PC10 : low
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   
    printf("Relay on");
  }// Turn on
  else{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    printf("Relay off");
  }
// Turn off
}

void RelayOn(){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);          // Turn on
  printf("Relay on");
}

void RelayOff(){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);        // Turn off
  printf("Relay off");
}