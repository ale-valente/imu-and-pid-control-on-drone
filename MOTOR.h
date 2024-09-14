#ifndef MOTOR_H
#define MOTOR_H

// -------------------- Include ---------------------------------------------------------------------

#include <Servo.h>

// -------------------- Pin Configuration -----------------------------------------------------------

// Motor: <company> <model>
#define MOTOR_NUM 4
uint8_t MotorPin[MOTOR_NUM] = {3, 5, 9, 10}; // {M1, M2, M3, M4}

// -------------------- Configuration ---------------------------------------------------------------

#define MOTOR_MIN_US 1000 // us [OFF]
#define MOTOR_MAX_US 2000 // us [ON]

// Motor Delay Setup
#define MOTOR_DELAY_SETUP_MS 10000 // ms

// ------------------------ Global Variables -------------------------------------------------------

Servo motor[MOTOR_NUM]; float motorValue[MOTOR_NUM]; //for(i=0; i<MOTOR_NUM; i++) motorValue[i] = 0.0f;

// -------------------- Subroutines -----------------------------------------------------------------

void MOTOR_Setup() {
  for(i=0; i<MOTOR_NUM; i++) {motorValue[i] = MOTOR_MIN_US; pinMode(MotorPin[i], OUTPUT); motor[i].attach(MotorPin[i]); motor[i].writeMicroseconds(motorValue[i]);}    
  delay(MOTOR_DELAY_SETUP_MS); return;}



void MOTOR_Off() {for(i=0; i<MOTOR_NUM; i++) {motorValue[i] = MOTOR_MIN_US; motor[i].writeMicroseconds(motorValue[i]); Serial.print(motorValue[i]); if(i<MOTOR_NUM-1) Serial.print(" ");} Serial.println(""); return;}

/*
void MOTOR_On(Servo mot, float perc) { // perc from 0.000.. to 100.000...
  if(perc <= 0)     {mot.writeMicroseconds(MOTOR_MIN_US); return;}
  if(perc >= 100)   {mot.writeMicroseconds(MOTOR_MAX_US); return;}
  uint16_t us = (uint16_t)((float) perc * (float)((float)MOTOR_MAX_US - (float)MOTOR_MIN_US)) + (uint16_t)MOTOR_MIN_US;
  mot.writeMicroseconds(us); return;}
void MOTOR_Off(Servo mot) {MOTOR_On(mot, 0); return;}
*/



#endif
