#ifndef PRINTDATA_H
#define PRINTDATA_H

void printRadioData(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6);
void printDesiredState(float thro, float roll_d, float pitch_d, float yaw_d);
void printGyroData(float gyroX, float gyroY, float gyroZ);
void printAccelData(float accX, float accY, float accZ);
void printMagData(float magX, float magY, float magZ);
void printRollPitchYaw(float roll, float pitch, float yaw);
void printPIDoutput(float roll_pid, float pitch_pid, float yaw_pid);
void printMotorCommands(float m1, float m2, float m3, float m4, float m5, float m6);
void printServoCommands(float s1, float s2, float s3, float s4, float s5, float s6, float s7);
void printLoopRate(float dt);

#endif
