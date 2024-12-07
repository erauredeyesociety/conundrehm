// Arduino/Teensy Flight Controller - dRehmFlight
// Author: Nicholas Rehm
// Project Start: 1/6/2020
// Last Updated: 7/29/2022
// Version: Beta 1.3

#ifndef RADIO_COMM_H
#define RADIO_COMM_H

#include <Arduino.h>

//========================================================================================================================//

// Declare shared variables
extern unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
extern unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
extern int ppm_counter;
extern unsigned long time_ms;


// Function prototypes
void radioSetup();
unsigned long getRadioPWM(int ch_num);
void getCh1();
void getCh2();
void getCh3();
void getCh4();
void getCh5();
void getCh6();
void getPPM();
void serialEvent3(void);

#endif
