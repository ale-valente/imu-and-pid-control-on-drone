#ifndef PID_H
#define PID_H

// -------------------- Configuration ---------------------------------------------------------------

#define PID_RollPitch_KP  55.00f          // Roll
#define PID_RollPitch_KI  10.0f         // Roll
#define PID_RollPitch_KD  2.05f          // Roll

#define PID_Roll_KP  PID_RollPitch_KP          // Roll
#define PID_Roll_KI  PID_RollPitch_KI         // Roll
#define PID_Roll_KD  PID_RollPitch_KD          // Roll
#define PID_Roll_TAU 0.02f          // Roll, [s] for "D"
#define PID_Pitch_KP  PID_RollPitch_KP         // Pitch
#define PID_Pitch_KI  PID_RollPitch_KI        // Pitch
#define PID_Pitch_KD  PID_RollPitch_KD         // Pitch
#define PID_Pitch_TAU 0.02f         // Pitch, [s] for "D"
#define PID_Yaw_KP  2.00f           // Yaw
#define PID_Yaw_KI  0.005f          // Yaw
#define PID_Yaw_KD  2.05f           // Yaw
#define PID_Yaw_TAU 0.02f           // Yaw, [s] for "D"
#define PID_Throttle_KP  2.00f      // Throttle
#define PID_Throttle_KI  0.005f     // Throttle
#define PID_Throttle_KD  2.05f      // Throttle
#define PID_Throttle_TAU 0.02f      // Throttle, [s] for "D"

// MOTOR REFERENCE SYSTEM MATRIX
float MOTOR_REFERENCE_SYSTEM_MATRIX[4*MOTOR_NUM] = { // motorValue = M * [PID_Roll, PID_Pitch, PID_Yaw, PID_Throttle]     ( M = MOTOR_REFERENCE_SYSTEM_MATRIX )
   1.0f, -1.0f, -1.0f,  1.0f,
   1.0f,  1.0f,  1.0f,  1.0f,
  -1.0f,  1.0f, -1.0f,  1.0f,
  -1.0f, -1.0f,  1.0f,  1.0f
  };
  
// ------------------------ Global Variables -------------------------------------------------------

float Roll_Angle_Before = 0.0f, Pitch_Angle_Before = 0.0f, Yaw_Angle_Before = 0.0f, Throttle_Before = 0.0f; // Angles (rad)
float Roll_Angle_Error = 0.0f, Pitch_Angle_Error = 0.0f, Yaw_Angle_Error = 0.0f, Throttle_Error = 0.0f; // Rad
float Roll_Angle_Error_Before = 0.0f, Pitch_Angle_Error_Before = 0.0f, Yaw_Angle_Error_Before = 0.0f, Throttle_Error_Before = 0.0f; // Rad
float Roll_Angle_0 = 0.0f, Pitch_Angle_0 = 0.0f, Yaw_Angle_0 = 0.0f, Throttle_0 = 0.0f; // Rad
float PID_Roll = 0.0f, PID_Pitch = 0.0f, PID_Yaw = 0.0f, PID_Throttle = 0.0f;
float PID_Roll_P = 0.0f, PID_Roll_I = 0.0f, PID_Roll_D = 0.0f;
float PID_Pitch_P = 0.0f, PID_Pitch_I = 0.0f, PID_Pitch_D = 0.0f;
float PID_Yaw_P = 0.0f, PID_Yaw_I = 0.0f, PID_Yaw_D = 0.0f;
float PID_Throttle_P = 0.0f, PID_Throttle_I = 0.0f, PID_Throttle_D = 0.0f;

// -------------------- Subroutines -----------------------------------------------------------------

void PID_Setpoint(float roll_0, float pitch_0, float yaw_0, float throttle_0) {Roll_Angle_0 = roll_0; Pitch_Angle_0 = pitch_0; Yaw_Angle_0 = yaw_0; Throttle_0 = throttle_0; return;}
void PID_SetpointRoll(float roll_0) {Roll_Angle_0 = roll_0; return;}
void PID_SetpointPitch(float pitch_0) {Pitch_Angle_0 = pitch_0; return;}
void PID_SetpointYaw(float yaw_0) {Yaw_Angle_0 = yaw_0; return;}
void PID_SetpointThrottle(float throttle_0) {Throttle_0 = throttle_0; return;}

void PID_Control(float sampleTime) {
  Roll_Angle_Error = Roll_Angle_0 - Roll_Angle;                 // Error Roll
  Pitch_Angle_Error = Pitch_Angle_0 - Pitch_Angle;              // Error Pitch
  // Yaw_Angle_Error = Yaw_Angle_0 - Yaw_Angle;                 // Error Yaw
  // Throttle_Error = Throttle_0 - Throttle;                    // Error Throttle
  PID_Roll_P = PID_Roll_KP * Roll_Angle_Error;                  // KP Roll
  PID_Pitch_P = PID_Pitch_KP * Pitch_Angle_Error;               // KP Pitch
  // PID_Yaw_P = PID_Yaw_KP * Yaw_Angle_Error;                  // KP Yaw
  // PID_Throttle_P = PID_Throttle_KP * Throttle_Error;         // KP Throttle
  // PID_I = PID_KI * error / s <- Tustin: s = (2/sampleTime) * (z-1)/(z+1) (Tustin Integration)
  PID_Roll_I += PID_Roll_KI * sampleTime * 0.5f * (Roll_Angle_Error + Roll_Angle_Error_Before);         // KI Roll
  PID_Pitch_I += PID_Pitch_KI * sampleTime * 0.5f * (Pitch_Angle_Error + Pitch_Angle_Error_Before);     // KI Pitch
  // PID_Yaw_I += PID_Yaw_KI * sampleTime * 0.5f * (Yaw_Angle_Error + Yaw_Angle_Error_Before);          // KI Yaw
  // PID_Throttle_I += PID_Throttle_KI * sampleTime * 0.5f * (Throttle_Error + Throttle_Error_Before);  // KI Throttle
  /* PID Roll */ PID_Roll = PID_Roll_P + PID_Roll_I + PID_Roll_D; /* Anti Wind-Up -> */ // if(PID_Roll < MOTOR_MIN_US) PID_Roll = MOTOR_MIN_US; if(PID_Roll > MOTOR_MAX_US) PID_Roll = MOTOR_MAX_US;
  /* PID Pitch */ PID_Pitch = PID_Pitch_P + PID_Pitch_I + PID_Pitch_D; /* Anti Wind-Up -> */ // if(PID_Pitch < MOTOR_MIN_US) PID_Pitch = MOTOR_MIN_US; if(PID_Pitch > MOTOR_MAX_US) PID_Pitch = MOTOR_MAX_US;
  /* PID Yaw */ // PID_Yaw = PID_Yaw_P + PID_Yaw_I + PID_Yaw_D; /* Anti Wind-Up -> */ // if(PID_Yaw < MOTOR_MIN_US) PID_Yaw = MOTOR_MIN_US; if(PID_Yaw > MOTOR_MAX_US) PID_Yaw = MOTOR_MAX_US;
  /* PID Throttle */ // PID_Throttle = PID_Throttle_P + PID_Throttle_I + PID_Throttle_D; /* Anti Wind-Up -> */ // if(PID_Throttle < MOTOR_MIN_US) PID_Throttle = MOTOR_MIN_US; if(PID_Throttle > MOTOR_MAX_US) PID_Throttle = MOTOR_MAX_US;
  
  /******/ PID_Throttle = 1200; /******/ 
  
  Roll_Angle_Error_Before = Roll_Angle_Error; Roll_Angle_Before = Roll_Angle;
  Pitch_Angle_Error_Before = Pitch_Angle_Error; Pitch_Angle_Before = Pitch_Angle;
  // Yaw_Angle_Error_Before = Yaw_Angle_Error; Yaw_Angle_Before = Yaw_Angle;
  // Throttle_Error_Before = Throttle_Error; Throttle_Before = Throttle;
  return;}


void MOTOR_Update() {
  for(i=0; i<MOTOR_NUM; i++) {
    motorValue[i] = MOTOR_REFERENCE_SYSTEM_MATRIX[4*i + 0]*PID_Roll + MOTOR_REFERENCE_SYSTEM_MATRIX[4*i + 1]*PID_Pitch + MOTOR_REFERENCE_SYSTEM_MATRIX[4*i + 2]*PID_Yaw + MOTOR_REFERENCE_SYSTEM_MATRIX[4*i + 3]*PID_Throttle; 
    /* Anti Wind-Up -> */ if(motorValue[i] < MOTOR_MIN_US) motorValue[i] = MOTOR_MIN_US; if(motorValue[i] > MOTOR_MAX_US) motorValue[i] = MOTOR_MAX_US;
    motor[i].writeMicroseconds(motorValue[i]); Serial.print(motorValue[i]); if(i<MOTOR_NUM-1) Serial.print(" ");} Serial.println("");
  return;}

#endif
