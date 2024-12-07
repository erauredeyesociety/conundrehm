#include <Arduino.h>
#include "printData.h"

// Function to print radio data
void printRadioData(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6) {
  Serial.print(F("CH1:"));
  Serial.print(ch1);
  Serial.print(F(" CH2:"));
  Serial.print(ch2);
  Serial.print(F(" CH3:"));
  Serial.print(ch3);
  Serial.print(F(" CH4:"));
  Serial.print(ch4);
  Serial.print(F(" CH5:"));
  Serial.print(ch5);
  Serial.print(F(" CH6:"));
  Serial.println(ch6);
}

// Function to print desired state
void printDesiredState(float thro, float roll_d, float pitch_d, float yaw_d) {
  Serial.print(F("thro_des:"));
  Serial.print(thro);
  Serial.print(F(" roll_des:"));
  Serial.print(roll_d);
  Serial.print(F(" pitch_des:"));
  Serial.print(pitch_d);
  Serial.print(F(" yaw_des:"));
  Serial.println(yaw_d);
}

// Function to print gyro data
void printGyroData(float gyroX, float gyroY, float gyroZ) {
  Serial.print(F("GyroX:"));
  Serial.print(gyroX);
  Serial.print(F(" GyroY:"));
  Serial.print(gyroY);
  Serial.print(F(" GyroZ:"));
  Serial.println(gyroZ);
}

// Function to print accelerometer data
void printAccelData(float accX, float accY, float accZ) {
  Serial.print(F("AccX:"));
  Serial.print(accX);
  Serial.print(F(" AccY:"));
  Serial.print(accY);
  Serial.print(F(" AccZ:"));
  Serial.println(accZ);
}

// Function to print magnetometer data
void printMagData(float magX, float magY, float magZ) {
  Serial.print(F("MagX:"));
  Serial.print(magX);
  Serial.print(F(" MagY:"));
  Serial.print(magY);
  Serial.print(F(" MagZ:"));
  Serial.println(magZ);
}

// Function to print roll, pitch, yaw data
void printRollPitchYaw(float roll, float pitch, float yaw) {
  Serial.print(F("roll:"));
  Serial.print(roll);
  Serial.print(F(" pitch:"));
  Serial.print(pitch);
  Serial.print(F(" yaw:"));
  Serial.println(yaw);
}

// Function to print PID outputs
void printPIDoutput(float roll_pid, float pitch_pid, float yaw_pid) {
  Serial.print(F("roll_PID:"));
  Serial.print(roll_pid);
  Serial.print(F(" pitch_PID:"));
  Serial.print(pitch_pid);
  Serial.print(F(" yaw_PID:"));
  Serial.println(yaw_pid);
}

// Function to print motor command data
void printMotorCommands(float m1, float m2, float m3, float m4, float m5, float m6) {
  Serial.print(F("m1:"));
  Serial.print(m1);
  Serial.print(F(" m2:"));
  Serial.print(m2);
  Serial.print(F(" m3:"));
  Serial.print(m3);
  Serial.print(F(" m4:"));
  Serial.print(m4);
  Serial.print(F(" m5:"));
  Serial.print(m5);
  Serial.print(F(" m6:"));
  Serial.println(m6);
}

// Function to print servo command data
void printServoCommands(float s1, float s2, float s3, float s4, float s5, float s6, float s7) {
  Serial.print(F("s1:"));
  Serial.print(s1);
  Serial.print(F(" s2:"));
  Serial.print(s2);
  Serial.print(F(" s3:"));
  Serial.print(s3);
  Serial.print(F(" s4:"));
  Serial.print(s4);
  Serial.print(F(" s5:"));
  Serial.print(s5);
  Serial.print(F(" s6:"));
  Serial.print(s6);
  Serial.print(F(" s7:"));
  Serial.println(s7);
}

// Function to print loop rate
void printLoopRate(float dt) {
  Serial.print(F("dt:"));
  Serial.println(dt * 1000000.0);
}
