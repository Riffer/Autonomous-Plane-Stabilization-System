
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>

#define PWM_MAX 2000
#define PWM_MIN 1000
#define PWM_MID 1500

#define MPU_ADDRESS 0x68

enum {ROLL=0, PITCH=1, YAW=2, CHANNEL_MAX};

enum {GVAL=0, GCAL=1, GACC=2, GYRO_MAX};

struct pidgainStruct
{
  float p = 0;
  float i = 0;
  float d = 0;
  const int max = 400;
  const int max_i = 100;
};

struct channelValStruct 
{              // INPUT:              // OUTPUT:
  int ch1 = 0; // Receiver Roll       // ROLL
  int ch2 = 0; // Receiver Pitch      // PITCH
  int ch3 = 0; // Receiver Yaw        // INVERTED ROLL
  int ch4 = 0; // Intensity Knob      // YAW
};

struct gyroStruct
{
  long x = 0;
  long y = 0;
  long z = 0;
  long totalVector = 0;
  //bool set_gyro_angles;
};


struct timerISRStruct
{
  byte ch1 = 0;             // Receiver Roll
  unsigned long timer1;
  byte ch2 = 0;             // Receiver Pitch
  unsigned long timer2;
  byte ch3 = 0;             // Receiver Yaw
  unsigned long timer3;
  byte ch4 = 0;             // Intensity Knob
  unsigned long timer4;
};


struct angleValStruct
{
  float Chan[CHANNEL_MAX];
  float Acc[CHANNEL_MAX];
  float Out[CHANNEL_MAX];
  float Adjust[CHANNEL_MAX];
};

struct RollPitchYawStruct
{
  float chan[CHANNEL_MAX];
};

struct PIDStruct
{
  RollPitchYawStruct i_mem;
  RollPitchYawStruct input;
  RollPitchYawStruct output;
  RollPitchYawStruct setpoint;
  RollPitchYawStruct d_error;
  RollPitchYawStruct gyro;
};

#define DEBUG 1 //0 for turn off, 1 for turn on - this works function wise and the compiler optimizes if(0){} out


//for debugging output
#define serial_printF(x, ...)                     \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.print(F(x), ##__VA_ARGS__);      \
    } while (0)

#define serial_print(x, ...)                     \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.print(x, ##__VA_ARGS__);      \
    } while (0)

#define serial_printlnF(x, ...)                   \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.println(F(x), ##__VA_ARGS__);    \
    } while (0)

#define serial_println(x, ...)                   \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.println(x, ##__VA_ARGS__);    \
    } while (0)



#endif // MAIN_H