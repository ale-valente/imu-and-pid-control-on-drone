// -------------------- Output ----------------------------------------------------------------------

#define OUTPUT_TIME 0.01f // ms
#define SERIAL_BAUDRATE 115200 // baud = bit/s

// -------------------- Constants -------------------------------------------------------------------

#define RAD_TO_DEG 57.295779513082320876798154814105f // 180/pi

// ------------------------ Global Variables -------------------------------------------------------

// General
int i;
// Angles & Throttle
float Roll_Angle = 0.0f, Pitch_Angle = 0.0f, Yaw_Angle = 0.0f, Throttle = 0.0f; // Angles (rad)
// Timer
extern unsigned long timer0_millis; // to reset millis()
float timeNow, timeBefore, sampleTime;
// Output
float OutputTime = 0.0f;

// -------------------- Include ---------------------------------------------------------------------

//#include "LED.h"
#include "MOTOR.h"
#include "IMU.h"
#include "PID.h"

// ------------------------ Program ----------------------------------------------------------------


void setup() {
  Serial.begin(SERIAL_BAUDRATE); // begin serial (usb) comunication // while (!Serial) {/* wait for Arduino serial Monitor port to connect */} delay(100);
  Wire.begin(); //begin the wire I2C comunication
  //Serial.println("Start Setup...");
  //LED_Setup();
  MOTOR_Setup(); // for security (due to ESC setup), call this function before other in the setup
  IMU_Setup();
  timer0_millis = 0; // reset millis()
  timeNow = millis();
  //Serial.println("End Setup...");
}

bool StopMot = true;

void loop() {

  timeBefore = timeNow; timeNow = millis(); sampleTime /* [s] */ = (timeNow - timeBefore)/1000.0f;
  IMU_Read(sampleTime); // Roll, Pitch
  PID_Control(sampleTime); // Roll, Pitch (, Yaw, Throttle)

  if(StopMot) {
    MOTOR_Off();
  }
  else {
    MOTOR_Update();
  }
  
  while(Serial.available() > 0) {
    char c = (char)Serial.read();
    if(c != '\n' && c != '\r') {
      if(c == '0') {StopMot = true; Serial.println("Motor OFF");}
      else {StopMot = false; Serial.println("Motor ON");}
    }
  }


}
