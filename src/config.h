/**********************************************************************************************************************
 * Filename: config.h
 * Author: [Your Name]
 * Collaborators: [If any]
 * Created: [Insert Date]
 * Modified: [Insert Date]
 * Link: [Insert if applicable]
 *
 * Purpose: Configuration header file to store user-defined settings, sensor settings, and hardware mappings.
 *
 * Attributes:
 * - Sensor configuration settings
 * - User-specific variables
 * - Pin mappings
 * - Filter parameters
 *
 * Methods: None
 **********************************************************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

//Uncomment only one IMU
#define USE_MPU6050_I2C //Default
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//



//REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer
#include <MPU6050.h>

#if defined USE_SBUS_RX
  #include "SBUS.h"   //sBus interface
#endif

#if defined USE_DSM_RX
  #include "DSMRX.h"  
#endif

#if defined USE_MPU6050_I2C
  #include "MPU6050.h"
  extern MPU6050 mpu6050; // Declare the object
#elif defined USE_MPU9250_SPI
  #include "MPU9250.h"
  extern MPU9250 mpu9250; // Declare the object
#else
  #error No MPU defined... 
#endif



//========================================================================================================================//



//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif
  
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           

// Failsafe values for radio channels
extern unsigned long channel_1_fs;
extern unsigned long channel_2_fs;
extern unsigned long channel_3_fs;
extern unsigned long channel_4_fs;
extern unsigned long channel_5_fs;
extern unsigned long channel_6_fs;

// Filter parameters
extern float B_madgwick;  
extern float B_accel;     
extern float B_gyro;      
extern float B_mag;       

// Magnetometer calibration parameters
extern float MagErrorX, MagErrorY, MagErrorZ;
extern float MagScaleX, MagScaleY, MagScaleZ;

// IMU calibration parameters
extern float AccErrorX, AccErrorY, AccErrorZ;
extern float GyroErrorX, GyroErrorY, GyroErrorZ;

// Controller parameters
extern float i_limit;
extern float maxRoll, maxPitch, maxYaw;
extern float Kp_roll_angle, Ki_roll_angle, Kd_roll_angle;
extern float Kp_pitch_angle, Ki_pitch_angle, Kd_pitch_angle;
extern float B_loop_roll, B_loop_pitch; // Roll and pitch damping terms
extern float Kp_roll_rate, Ki_roll_rate, Kd_roll_rate;
extern float Kp_pitch_rate, Ki_pitch_rate, Kd_pitch_rate;
extern float Kp_yaw, Ki_yaw, Kd_yaw;


//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup

// Radio
const int ch1Pin = 15; // throttle
const int ch2Pin = 16; // ail
const int ch3Pin = 17; // ele
const int ch4Pin = 20; // rudd
const int ch5Pin = 21; // gear (throttle cut)
const int ch6Pin = 22; // aux1 (free aux channel)
const int PPM_Pin = 23;

// OneShot125 ESC pin outputs
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
const int m6Pin = 5;

// PWM servo or ESC outputs
const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 8;
const int servo4Pin = 9;
const int servo5Pin = 10;
const int servo6Pin = 11;
const int servo7Pin = 12;

// Servo object definitions
extern PWMServo servo1;
extern PWMServo servo2;
extern PWMServo servo3;
extern PWMServo servo4;
extern PWMServo servo5;
extern PWMServo servo6;
extern PWMServo servo7;

// IMU variables
extern float AccX, AccY, AccZ;
extern float GyroX, GyroY, GyroZ;
extern float roll_IMU, pitch_IMU, yaw_IMU;

// General loop timing
extern float dt;
extern unsigned long current_time, prev_time;
extern unsigned long print_counter, serial_counter;
extern unsigned long blink_counter, blink_delay;
extern bool blinkAlternate;

// Radio communication:
extern unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
extern unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

// SBUS and DSM-related
#if defined USE_SBUS_RX
extern SBUS sbus;
extern uint16_t sbusChannels[16];
extern bool sbusFailSafe;
extern bool sbusLostFrame;
#endif

#if defined USE_DSM_RX
extern DSM1024 DSM;
#endif

// IMU:
extern float AccX, AccY, AccZ;
extern float AccX_prev, AccY_prev, AccZ_prev;
extern float GyroX, GyroY, GyroZ;
extern float GyroX_prev, GyroY_prev, GyroZ_prev;
extern float MagX, MagY, MagZ;
extern float MagX_prev, MagY_prev, MagZ_prev;
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float roll_IMU_prev, pitch_IMU_prev;
extern float q0, q1, q2, q3; // Initialize quaternion for madgwick filter

// Normalized desired state:
extern float thro_des, roll_des, pitch_des, yaw_des;
extern float roll_passthru, pitch_passthru, yaw_passthru;

// Controller:
extern float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID;
extern float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID;
extern float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID;

// Mixer
extern float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
extern int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
extern float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
extern int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

// Flight status
extern bool armedFly;

#endif