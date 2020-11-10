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
#### Diagram
<img src="https://user-images.githubusercontent.com/58382336/98700006-126e3100-23bb-11eb-89cf-4b1badd93114.png"  width="700" height="382">

#### RF Module 
<img src="https://user-images.githubusercontent.com/58382336/98700440-86103e00-23bb-11eb-9d88-03f6216ec8ba.PNG"  width="350" height="180">

#### LIDAR & Raspberry Pi
<img src="https://user-images.githubusercontent.com/58382336/98700404-7d1f6c80-23bb-11eb-944c-37b6f734e429.PNG"  width="350" height="180">

#### Gimbal
<img src="https://user-images.githubusercontent.com/58382336/98700490-945e5a00-23bb-11eb-9c55-2fd1ac06e7e9.PNG"  width="350" height="180">

#### Solar Charging System
<img src="https://user-images.githubusercontent.com/58382336/98700307-6842d900-23bb-11eb-9c94-18f1635206c6.png"  width="700" height="370">


### 3) Video
Click Image! ( Linked to YouTube )

[![STEV.e](https://img.youtube.com/vi/2x-dOrIg4Pk/0.jpg)](https://youtu.be/2x-dOrIg4Pk) 


### 4) Function
It is easy for users to recognize and control Web GUI, which sends control signals from the server to the vehicle. 

The vehicle recognizes the control signal and controls the vehicle and its internal modules.

The internal module consists of a gimbal with an action cam attached, a solar panel with automatic solar tracking, and 4 DC motors which adjust the direction through speed differences, and use Buck Converter to reduce heat losses, and make solar charging and battery protection circuits PCB.

And the vehicle sends signals to the server the current direction and speed of vehicle, and the angle of the gimbal and solar panels, and the server shows these on the Web GUI.

Then, when the exploration vehicle returns, the 2D environmental map obtained by the LIDAR connected to the Raspberry Pi can be analyzed.

All signals are communicated using RF modules. And the video of the action cam is always sent to the server, independent of the control of the ARM.


| Control Signals (From Server to Vehicle) | Current State Signals (From Vehicle to Server) |
|:---:|:---:|
|Gimbal angle, Direction/Speed of Vehicle, Turn on/off Raspberry PI 3|Angle of Gimbal/Solar Panel, Battery power level, Tank Direction/Speed, state of Raspberry Pi 3 power| 
* * *


