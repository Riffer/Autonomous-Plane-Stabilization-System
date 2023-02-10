
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>

struct pidgainStruct
{
  float p = 0;
  float i = 0;
  float d = 0;
  int max = 400;
  int max_i = 100;
};

struct channelValStruct
{
  int channel1 = 0;
  int channel2 = 0;
  int channel3 = 0;
  int channel4 = 0;
};

struct gyroStruct
{
  long x = 0;
  long y = 0;
  long z = 0;
  long totalvector = 0;
  //bool set_gyro_angles;
};

struct timerISRStruct
{
  byte channel1 = 0;
  unsigned long timer1;
  byte channel2 = 0;
  unsigned long timer2;
  byte channel3 = 0;
  unsigned long timer3;
  byte channel4 = 0;
  unsigned long timer4;
};


#endif // MAIN_H