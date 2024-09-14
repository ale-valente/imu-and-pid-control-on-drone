#ifndef LED_H
#define LED_H

// -------------------- Pin Configuration -----------------------------------------------------------

#define LED_RED 13
#define LED_YELLOW 12
#define LED_GREEN 11

// -------------------- Subroutines -----------------------------------------------------------------

void LED_Setup() {pinMode(LED_RED, OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_GREEN, OUTPUT); return;}
void LED_Off() {digitalWrite(LED_RED, LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_GREEN, LOW); return;}
void LED_Red() {digitalWrite(LED_RED, HIGH); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_GREEN, LOW); return;}
void LED_Yellow() {digitalWrite(LED_RED, LOW); digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_GREEN, LOW); return;}
void LED_Green() {digitalWrite(LED_RED, LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_GREEN, HIGH); return;}

#endif
