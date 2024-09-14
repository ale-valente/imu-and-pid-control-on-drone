#ifndef IMU_H
#define IMU_H

// -------------------- Include ---------------------------------------------------------------------

#include <Wire.h>

// -------------------- Pin Configuration -----------------------------------------------------------

// IMU: TDK InvenSense MPU6050
// A4 (SDA), A5 (SCL)

// -------------------- Configuration ---------------------------------------------------------------

// Linear Combination
#define IMU_ALPHA_LC                      0.95f // angle = alpha * angle_gyr + (1-alpha) * angle_acc
//#define IMU_INTEGRATION_TIME_LC           0.0001f // 100 uS ( time in seconds [s] )

// Filter IIR (First Order) IMU Data : y[n] = alpha * y[n-1] + (1-alpha) * x[n]
#define LPF_GYR_ALPHA                     0.1f  // <- un po meno filtrato perche performa bene nel breve periodo -> giroscopio contiene alte frequenze da tenere (non va filtrato molto)
#define LPF_ACC_ALPHA                     0.2f // <- un po piu filtrato perche performa bene nel lungo periodo -> accelerometro contiene basse frequenze da tenere (è bene filtrarlo un po di piu)
#define LPF_TEMP_ALPHA                    0.35f // <- filtro disturbi temperatura

// Calibration Offset
#define IMU_OFFSET_AVERAGE_SAMPLES        1000

// IMU REFERENCE SYSTEM MATRIX: it depends on the reference system of the IMU and how it's mounted on the drone
float IMU_REFERENCE_SYSTEM_MATRIX[9] = { // RefSys_Drone = M * RefSys_IMU     ( M = IMU_REFERENCE_SYSTEM_MATRIX )
  1.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 1.0f
};

// Register Definitions
#define I2C_ADDR_IMU                      0x68 // #define MPU6050_I2C_ADDR (0x68 << 1) // Left-Shift due to 7-bit addressing
//#define IMU_REG_CONFIG                0x1A
//#define IMU_REG_INT_PIN_CFG           0x37
//#define IMU_REG_INT_ENABLE            0x38
#define IMU_REG_PWR_MGMT_1                0x6B
#define IMU_REG_ACC_DATA_START            0x3B
#define IMU_REG_ACC_DATA_NUM_REGISTERS    6
#define IMU_REG_GYR_DATA_START            0x43
#define IMU_REG_GYR_DATA_NUM_REGISTERS    6
#define IMU_REG_TEMP_DATA_START           0x41
#define IMU_REG_TEMP_DATA_NUM_REGISTERS   2
// Constants Conversion (default settings: acc: +-2g, gyr: +-250dps)
#define IMU_ACC_RAW_TO_MPS2               0.000598754882813f // g/16384 [m/s^2] (16384 -> Datasheet a +-2g) (g = 9.81)
#define IMU_GYR_RAW_TO_RPS                0.00013323124061f // (1/131)*(2pi/360) [rad/s] (131 -> Datasheet a +-250 °/s)
#define IMU_TEMP_RAW_TO_DEGC              0.002941176470588f // 1/340 (340 -> Datasheet)
#define IMU_TEMP_OFFSET_DEGC              36.53f // (36.53 -> Datasheet)

// ------------------------ Global Variables -------------------------------------------------------

float IMU_Acc_X = 0.0f, IMU_Acc_Y = 0.0f, IMU_Acc_Z = 0.0f; // Accelerometer (m/s^2)
float IMU_Gyr_X = 0.0f, IMU_Gyr_Y = 0.0f, IMU_Gyr_Z = 0.0f; // Gyroscope (rad/s)
float IMU_Temp = 0.0f; // Temperature (°C)
float IMU_Roll_Angle = 0.0f, IMU_Pitch_Angle = 0.0f, IMU_Yaw_Angle = 0.0f; // rotation around X, Y, Z (rad)
float IMU_ANGLE_ROLL_OFFSET_RAD = 0.0f, IMU_ANGLE_PITCH_OFFSET_RAD = 0.0f, IMU_ANGLE_YAW_OFFSET_RAD = 0.0f;

// -------------------- Subroutines -----------------------------------------------------------------



void IMU_Read_NoOffset(float sampleTime) {
  // MPU-6050 gives 16 bits data so we have to create some 16int constants to store the data for accelerations and gyro (int16_t or float)
  Wire.beginTransmission(I2C_ADDR_IMU); Wire.write(IMU_REG_ACC_DATA_START); Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADDR_IMU,IMU_REG_ACC_DATA_NUM_REGISTERS+IMU_REG_GYR_DATA_NUM_REGISTERS+IMU_REG_TEMP_DATA_NUM_REGISTERS,true); 
  // Raw Data -> Convert signed ints to 'units' (m/s^2, °C, rad/s)
  float IMU_Acc_X_temp = (float)(Wire.read()<<8 | Wire.read())*IMU_ACC_RAW_TO_MPS2;
  float IMU_Acc_Y_temp = (float)(Wire.read()<<8 | Wire.read())*IMU_ACC_RAW_TO_MPS2;
  float IMU_Acc_Z_temp = (float)(Wire.read()<<8 | Wire.read())*IMU_ACC_RAW_TO_MPS2;
  float IMU_Temp_temp = (float)(Wire.read()<<8 | Wire.read())*IMU_TEMP_RAW_TO_DEGC + IMU_TEMP_OFFSET_DEGC;
  float IMU_Gyr_X_temp = (float)(Wire.read()<<8 | Wire.read())*IMU_GYR_RAW_TO_RPS;
  float IMU_Gyr_Y_temp = (float)(Wire.read()<<8 | Wire.read())*IMU_GYR_RAW_TO_RPS;
  float IMU_Gyr_Z_temp = (float)(Wire.read()<<8 | Wire.read())*IMU_GYR_RAW_TO_RPS;
  // Remap to the drone reference system
  float IMU_Acc_X_remap = IMU_REFERENCE_SYSTEM_MATRIX[0]*IMU_Acc_X_temp + IMU_REFERENCE_SYSTEM_MATRIX[1]*IMU_Acc_Y_temp + IMU_REFERENCE_SYSTEM_MATRIX[2]*IMU_Acc_Z_temp;
  float IMU_Acc_Y_remap = IMU_REFERENCE_SYSTEM_MATRIX[3]*IMU_Acc_X_temp + IMU_REFERENCE_SYSTEM_MATRIX[4]*IMU_Acc_Y_temp + IMU_REFERENCE_SYSTEM_MATRIX[5]*IMU_Acc_Z_temp;
  float IMU_Acc_Z_remap = IMU_REFERENCE_SYSTEM_MATRIX[6]*IMU_Acc_X_temp + IMU_REFERENCE_SYSTEM_MATRIX[7]*IMU_Acc_Y_temp + IMU_REFERENCE_SYSTEM_MATRIX[8]*IMU_Acc_Z_temp;
  float IMU_Gyr_X_remap = IMU_REFERENCE_SYSTEM_MATRIX[0]*IMU_Gyr_X_temp + IMU_REFERENCE_SYSTEM_MATRIX[1]*IMU_Gyr_Y_temp + IMU_REFERENCE_SYSTEM_MATRIX[2]*IMU_Gyr_Z_temp;
  float IMU_Gyr_Y_remap = IMU_REFERENCE_SYSTEM_MATRIX[3]*IMU_Gyr_X_temp + IMU_REFERENCE_SYSTEM_MATRIX[4]*IMU_Gyr_Y_temp + IMU_REFERENCE_SYSTEM_MATRIX[5]*IMU_Gyr_Z_temp;
  float IMU_Gyr_Z_remap = IMU_REFERENCE_SYSTEM_MATRIX[6]*IMU_Gyr_X_temp + IMU_REFERENCE_SYSTEM_MATRIX[7]*IMU_Gyr_Y_temp + IMU_REFERENCE_SYSTEM_MATRIX[8]*IMU_Gyr_Z_temp;
  // Filter IIR (First Order) IMU Data : y[n] = alpha * y[n-1] + (1 - alpha) * x[n]
  IMU_Acc_X = LPF_ACC_ALPHA * IMU_Acc_X + (1.0f - LPF_ACC_ALPHA) * IMU_Acc_X_remap;
  IMU_Acc_Y = LPF_ACC_ALPHA * IMU_Acc_Y + (1.0f - LPF_ACC_ALPHA) * IMU_Acc_Y_remap;
  IMU_Acc_Z = LPF_ACC_ALPHA * IMU_Acc_Z + (1.0f - LPF_ACC_ALPHA) * IMU_Acc_Z_remap;
  IMU_Gyr_X = LPF_GYR_ALPHA * IMU_Gyr_X + (1.0f - LPF_GYR_ALPHA) * IMU_Gyr_X_remap;
  IMU_Gyr_Y = LPF_GYR_ALPHA * IMU_Gyr_Y + (1.0f - LPF_GYR_ALPHA) * IMU_Gyr_Y_remap;
  IMU_Gyr_Z = LPF_GYR_ALPHA * IMU_Gyr_Z + (1.0f - LPF_GYR_ALPHA) * IMU_Gyr_Z_remap;
  IMU_Temp = LPF_TEMP_ALPHA * IMU_Temp + (1.0f - LPF_TEMP_ALPHA) * IMU_Temp_temp;
  // Linear Combination : sampleTime in seconds [s] ! for ex. 150 ms = 0.15 s => sampleTime = 0.15
  float IMU_Acc_Roll_Angle = atan((IMU_Acc_Y)/sqrt(pow((IMU_Acc_X),2) + pow((IMU_Acc_Z),2))); // (rad) angleX = phi = atan( accY / sqrt(accX^2 + accZ^2) ) <- more accurate than the two-axis imu where it's used the formula: roll_angle = atan(Ax/Ay)
  float IMU_Acc_Pitch_Angle = atan(-1*(IMU_Acc_X)/sqrt(pow((IMU_Acc_Y),2) + pow((IMU_Acc_Z),2))); // (rad) angleY = phi = atan( -accX / sqrt(accY^2 + accZ^2) )
  float IMU_Gyr_Roll_Angle = IMU_Roll_Angle + IMU_Gyr_X*sampleTime; // Euler integration for gyro angles
  float IMU_Gyr_Pitch_Angle = IMU_Pitch_Angle + IMU_Gyr_Y*sampleTime; // Euler integration for gyro angles
  IMU_Roll_Angle = IMU_ALPHA_LC*IMU_Gyr_Roll_Angle + (1.0f-IMU_ALPHA_LC)*IMU_Acc_Roll_Angle;
  IMU_Pitch_Angle = IMU_ALPHA_LC*IMU_Gyr_Pitch_Angle + (1.0f-IMU_ALPHA_LC)*IMU_Acc_Pitch_Angle;
  return;}

void IMU_Read(float sampleTime) {
  IMU_Read_NoOffset(sampleTime);
  // Remove Offset
  Roll_Angle = IMU_Roll_Angle - IMU_ANGLE_ROLL_OFFSET_RAD; // rad
  Pitch_Angle = IMU_Pitch_Angle - IMU_ANGLE_PITCH_OFFSET_RAD; // rad
  return;}

void IMU_Setup() {
  // Disable FSYNC, enable Digital low-pass filter (fs=1kHz, bandwidth: acc=94Hz, gyr=98Hz)
  //Wire.beginTransmission(I2C_ADDR_IMU); Wire.write(IMU_REG_CONFIG); Wire.write(0x02); Wire.endTransmission(true);
  // Enable interrupt and interrupt status bits are cleared on any read operation
  //Wire.beginTransmission(I2C_ADDR_IMU); Wire.write(IMU_REG_INT_PIN_CFG); Wire.write(0x10); Wire.endTransmission(true);
  //Wire.beginTransmission(I2C_ADDR_IMU); Wire.write(IMU_REG_INT_ENABLE); Wire.write(0x01); Wire.endTransmission(true);
  // Wake-up IMU
  Wire.beginTransmission(I2C_ADDR_IMU); Wire.write(IMU_REG_PWR_MGMT_1); Wire.write(0); Wire.endTransmission(true);
  // Calibration
  IMU_ANGLE_ROLL_OFFSET_RAD = 0.0f; IMU_ANGLE_PITCH_OFFSET_RAD = 0.0f; // <- non strettamente necessaria questa inizializzazione perche l ho gia fatta ma puo servire se con un bottone devo resettare nuovamente la calibrazione
  float tNow = millis(), tBefore, sTime;
  for(i=0; i<IMU_OFFSET_AVERAGE_SAMPLES; i++) {
    tBefore = tNow; tNow = millis(); sTime = (tNow - tBefore)/1000.0f /* [s] */;
    IMU_Read_NoOffset(sTime); 
    IMU_ANGLE_ROLL_OFFSET_RAD = IMU_ANGLE_ROLL_OFFSET_RAD + IMU_Roll_Angle; IMU_ANGLE_PITCH_OFFSET_RAD = IMU_ANGLE_PITCH_OFFSET_RAD + IMU_Pitch_Angle;
    delay(20);}
  IMU_ANGLE_ROLL_OFFSET_RAD = IMU_ANGLE_ROLL_OFFSET_RAD / IMU_OFFSET_AVERAGE_SAMPLES; IMU_ANGLE_PITCH_OFFSET_RAD = IMU_ANGLE_PITCH_OFFSET_RAD / IMU_OFFSET_AVERAGE_SAMPLES;
  return;}
  
#endif
