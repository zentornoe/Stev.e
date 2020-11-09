#include "stm32f1xx_hal.h"
#include "total.h"

#define M_MID   10000
#define G_MAX   8
#define S_MAX   8
#define MIN     0

extern uint8_t buffer[1];       // main.c
int S_dir=4, G_dir=4;        // motor speed(Tank_Control.c), Solar Tracker direction(SolarTracker.c),Gimbal direction(Gimbal_Control.c)



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART1){
    switch(buffer[0]){
    case 'w' :  // motor straight
    case 'W' :
      TankSt();         // Tank_Control.c
      break;
      
      
      
    case 's' :  // motor back
    case 'S' :
      TankBack();       // Tank_Control.c
      break;
      
      
      
    case 'd' :  // turn right
    case 'D' :
      TankRight();      // Tank_Control.c
      break;
      
      
      
    case 'a' :  // turn left (small letter of 'L")
    case 'A' :
      TankLeft();       // Tank_Control.c
      break;
      
      
      
    case 'x' :  // motor stop
    case 'X' :
      TankStop();       // Tank_Control.c
      break;
      
      
      
    case 'k' : // Gimbal Turn Right
    case 'K' :
      G_dir--;
      if(G_dir<MIN)
        G_dir=MIN;      // Left MIN
      GimbalCtrl();     // Gimbal_Control.c
      GimbalCtrl();     // Gimbal_Control.c
      break;
      
      
      
    case 'j' :  // Gimbal Turn Left
    case 'J' :
      G_dir++;  // Turn right the Gimbal
      if(G_dir>G_MAX)
        G_dir=G_MAX;    // Right MAX
      break;
      
      
      
    case 'o' : // Turn ON
    case 'O' :
      RelayOn();  
      break;
      
      
    case 'f' :  //Turn OFF
    case 'F' :
      RelayOff();
      break;
    }   // Switch end
  HAL_UART_Receive_IT(huart, buffer, 1);   // Receive RF - UART1 (Interrupt)
  }     // UART IT end
}
