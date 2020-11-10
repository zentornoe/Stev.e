# 2019 Hanium Contest ( STEV.e )


2019 Hanium Contest ( 3 students )

#### ● Machine design, ARM Coding(STM32F103RB - nucleo board), Solar Tracking part
https://github.com/zentornoe/Stev.e  ( Me )

#### ● Sever, LIDAR(Raspberry Pi 3), GUI(Web Programming) part
http://github.com/63days/hanium
#### ● Citcuit, Action Camera, RF Communication part
Daeuk Kim

* * *
## 1. ARM_Codes
 MCU : STM32F103RB ( ARM Cortex 4 ) , Modularized Program Code
 
### 1) main.c
  main program
  
### 2) Battery_ADC.c
 Check the battery power level.
 
 Checking voltage of battery using ADC. (Reason of Division - ARM max voltage : 3.3[V], Battery max voltage : 14.4[V])
 
### 3) CDS_ADC.c
 Get the value from light sensor
 
 Check the voltage between 2 CDS, and return the value as up/down.
 
### 4) Gimbal_Control.c
 Control the Gimbal(for Action Camera).
 
 From RF module receive the signal as UART. After that, control the Gimbal by PWM signal. ( Left and Right )
 
### 5) Relay.c
 Control 'Relay' for turn on/off Raspberry Pi 3 remotely.
 
 Receive the singal from RF module, and turn on/off.
 
### 6) SolarTracker.c
 Control Solar Panel for solar tracking.
 
 From module **'CDS_ADC.c'**, receive the value, and control the angle of solar tracker automatically. ( Angle : 9 steps )
 
### 7) Tank_Control.c
 Control the Car remotely using PWM signal.
 
 4 DC motors in tank. Go/Back straight(9 steps), Turn right/left(difference of 2 motors).
 
 From RF module receive the signal as UART. 
 
### 8) UART.c
 UART communication by using RF module for communicating with Server
 
 Receive the control signal, and Send the current state signal.


* * *
## 2. STEV.e
**S**olar **T**racking **E**xploration **V**ehicles . **E**xpandable version
### 1) Flow Chart
<img src="https://user-images.githubusercontent.com/58382336/98515379-558eae00-22ae-11eb-9804-f0dc4c5a3bff.png"  width="700" height="370">

### 2) Conneted Device
<img src="https://user-images.githubusercontent.com/58382336/98515383-56bfdb00-22ae-11eb-85a6-94b91b72b8b6.png"  width="700" height="370">
