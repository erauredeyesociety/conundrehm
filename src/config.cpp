/*********************************************************
* Filename: config.cpp
* Author: <Your_Name>
* Collaborators: None
* Created: 2023-12-07
* Modified: 2023-12-07
* Link: <PlaceLinkHere>
*
* Purpose: Implementation for configuration variables
*
*********************************************************/

#include "config.h"
#include "SBUS.h"
#include "DSMRX.h"

#if defined USE_MPU6050_I2C
  MPU6050 mpu6050; // Define the object here
#elif defined USE_MPU9250_SPI
  MPU9250 mpu9250(SPI2,36); // Define the object here
#endif

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           

// Failsafe values for radio channels
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1

// Filter parameters
float B_madgwick = 0.04;  // Madgwick filter parameter
float B_accel = 0.14;     // Accelerometer LP filter paramter
float B_gyro = 0.1;       // Gyro LP filter paramter
float B_mag = 1.0;        // Magnetometer LP filter parameter

// Magnetometer calibration parameters
float MagErrorX = 0.0;
float MagErrorY = 0.0;
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

// IMU calibration parameters
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;

// Controller parameters
float i_limit = 25.0;     // Integrator saturation level
float maxRoll = 30.0;     // Max roll angle in degrees for angle mode
float maxPitch = 30.0;    // Max pitch angle in degrees for angle mode
float maxYaw = 160.0;     // Max yaw rate in degrees/second

float Kp_roll_angle = 0.2;    // Roll P-gain
float Ki_roll_angle = 0.3;    // Roll I-gain
float Kd_roll_angle = 0.05;   // Roll D-gain
float B_loop_roll = 0.9;      // Roll damping term for control

float Kp_pitch_angle = 0.2;   // Pitch P-gain
float Ki_pitch_angle = 0.3;   // Pitch I-gain
float Kd_pitch_angle = 0.05;  // Pitch D-gain
float B_loop_pitch = 0.9;     // Pitch damping term for control

float Kp_roll_rate = 0.15;    // Roll P-gain for rate mode
float Ki_roll_rate = 0.2;     // Roll I-gain for rate mode
float Kd_roll_rate = 0.0002;  // Roll D-gain for rate mode
float Kp_pitch_rate = 0.15;   // Pitch P-gain for rate mode
float Ki_pitch_rate = 0.2;    // Pitch I-gain for rate mode
float Kd_pitch_rate = 0.0002; // Pitch D-gain for rate mode

float Kp_yaw = 0.3;           // Yaw P-gain
float Ki_yaw = 0.05;          // Yaw I-gain
float Kd_yaw = 0.00015;       // Yaw D-gain      

// Servo objects
PWMServo servo1;  
PWMServo servo2; 
PWMServo servo3;
PWMServo servo4; 
PWMServo servo5; 
PWMServo servo6;
PWMServo servo7;

// General loop timing
float dt = 0.0f;
unsigned long current_time = 0, prev_time = 0;
unsigned long print_counter = 0, serial_counter = 0;
unsigned long blink_counter = 0, blink_delay = 0;
bool blinkAlternate = false;

// Radio communication:
unsigned long channel_1_pwm = 0, channel_2_pwm = 0, channel_3_pwm = 0, channel_4_pwm = 0, channel_5_pwm = 0, channel_6_pwm = 0;
unsigned long channel_1_pwm_prev = 0, channel_2_pwm_prev = 0, channel_3_pwm_prev = 0, channel_4_pwm_prev = 0;

// Define SBUS or DSM conditionally
#if defined USE_SBUS_RX
SBUS sbus(Serial5);
uint16_t sbusChannels[16] = {0};
bool sbusFailSafe = false;
bool sbusLostFrame = false;
#endif

#if defined USE_DSM_RX
DSM1024 DSM;
#endif

// IMU
float AccX = 0.0f, AccY = 0.0f, AccZ = 0.0f;
float AccX_prev = 0.0f, AccY_prev = 0.0f, AccZ_prev = 0.0f;
float GyroX = 0.0f, GyroY = 0.0f, GyroZ = 0.0f;
float GyroX_prev = 0.0f, GyroY_prev = 0.0f, GyroZ_prev = 0.0f;
float MagX = 0.0f, MagY = 0.0f, MagZ = 0.0f;
float MagX_prev = 0.0f, MagY_prev = 0.0f, MagZ_prev = 0.0f;
float roll_IMU = 0.0f, pitch_IMU = 0.0f, yaw_IMU = 0.0f;
float roll_IMU_prev = 0.0f, pitch_IMU_prev = 0.0f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Desired state
float thro_des = 0.0f, roll_des = 0.0f, pitch_des = 0.0f, yaw_des = 0.0f;
float roll_passthru = 0.0f, pitch_passthru = 0.0f, yaw_passthru = 0.0f;

// Controller
float error_roll = 0.0f, error_roll_prev = 0.0f, roll_des_prev = 0.0f, integral_roll = 0.0f;
float integral_roll_il = 0.0f, integral_roll_ol = 0.0f, integral_roll_prev = 0.0f, integral_roll_prev_il = 0.0f, integral_roll_prev_ol = 0.0f, derivative_roll = 0.0f;
float error_pitch = 0.0f, error_pitch_prev = 0.0f, pitch_des_prev = 0.0f;
float integral_pitch = 0.0f, integral_pitch_il = 0.0f, integral_pitch_ol = 0.0f, integral_pitch_prev = 0.0f, integral_pitch_prev_il = 0.0f, integral_pitch_prev_ol = 0.0f, derivative_pitch = 0.0f;
float error_yaw = 0.0f, error_yaw_prev = 0.0f, integral_yaw = 0.0f;
float integral_yaw_prev = 0.0f, derivative_yaw = 0.0f;
float roll_PID = 0.0f, pitch_PID = 0.0f, yaw_PID = 0.0f;

// Mixer
float m1_command_scaled = 0.0f, m2_command_scaled = 0.0f, m3_command_scaled = 0.0f, m4_command_scaled = 0.0f, m5_command_scaled = 0.0f, m6_command_scaled = 0.0f;
int m1_command_PWM = 0, m2_command_PWM = 0, m3_command_PWM = 0, m4_command_PWM = 0, m5_command_PWM = 0, m6_command_PWM = 0;
float s1_command_scaled = 0.0f, s2_command_scaled = 0.0f, s3_command_scaled = 0.0f, s4_command_scaled = 0.0f, s5_command_scaled = 0.0f, s6_command_scaled = 0.0f, s7_command_scaled = 0.0f;
int s1_command_PWM = 0, s2_command_PWM = 0, s3_command_PWM = 0, s4_command_PWM = 0, s5_command_PWM = 0, s6_command_PWM = 0, s7_command_PWM = 0;

// Flight status
bool armedFly = false;