//========================================================Author:SLOMOGANGSTA(VSYK)===========================================================
//========================================================Date: 27th January 2019=============================================================
#include "main.h"
#include "mpu.hpp"
//#include <CPPM.h>
#include <jm_CPPM.h>

gyroStruct gyroVal;
gyroStruct gyroCal;
gyroStruct gyroAcc;

pidgainStruct gainroll;
pidgainStruct gainpitch;
pidgainStruct gainyaw;

angleValStruct angleVals;

volatile channelValStruct inputVals;
channelValStruct servoVals;

PIDStruct pid;


double mapf(double val, double in_min, double in_max, double out_min, double out_max);

void calculate_pid();
void setup_MPU();
//void setup_PWM();
void setup_SERVO();
void cycle_CPPM();
void setup_Wire();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  CPPM.begin();
  //setup_CPPM();

  setup_Wire();

  setup_MPU();


  delay(2500);

  //setup_PWM();
  //setup_SERVO();
}
 

void loop()
{
  static bool set_gyro_angles = false;
  static int loopCounter = 0;
  static unsigned long loop_start_time = 0;
  static float pinten = 0;
  static unsigned long interval = 100;
  static int onoff = HIGH;

  static unsigned long lastTime = millis(); // initialisiert die Variable, um die letzte Ausführungszeit zu speichern

  cycle_CPPM();

  interval = map(inputVals.ch1, 700, 2300, 150, 50);
  interval = constrain(interval, 50, 150);


  if (millis() >= (lastTime + interval))
  { // prüft, ob genug Zeit vergangen ist
    digitalWrite(LED_BUILTIN, onoff);
    onoff = !onoff;
    lastTime = millis(); // speichert die letzte Ausführungszeit
  }


  // channel input 4 intensivity to zero if below PWM_MIN
  if (inputVals.ch4 <= PWM_MIN)
  {
    gainroll.d = 0;
    gainroll.p = 0;
    gainpitch.d = 0;
    gainpitch.p = 0;
    gainyaw.p = 0;
    gainyaw.d = 0;
  }
  else
  {
    // channel input 4 intensivity mapped to value between 0.07 and 0.17
    pinten = mapf(inputVals.ch4, PWM_MIN, (PWM_MAX + 50), 0.07, 0.17);
    gainroll.d = pinten;
    gainroll.p = pinten;
    gainpitch.d = pinten;
    gainpitch.p = pinten;
    gainyaw.p = pinten;
    gainyaw.d = pinten;
  }

//#ifdef unused
  if (!mpu_read_data(&gyroAcc, &gyroVal))
  {
    delay(1000);
    return;
  }
//#endif

  gyroVal.x -= gyroCal.x;
  gyroVal.y -= gyroCal.y;
  gyroVal.z -= gyroCal.z;

  angleVals.pitch += gyroVal.x * 0.0000611;
  angleVals.roll += gyroVal.y * 0.0000611;

  angleVals.pitch += angleVals.roll * sin(gyroVal.z * 0.000001066);
  angleVals.roll -= angleVals.pitch * sin(gyroVal.z * 0.000001066);

  gyroAcc.totalVector = sqrt((gyroAcc.x * gyroAcc.x) + (gyroAcc.y * gyroAcc.y) + (gyroAcc.z * gyroAcc.z));
  angleVals.pitchAcc = asin((float)gyroAcc.y / gyroAcc.totalVector) * 57.296;
  angleVals.rollAcc = asin((float)gyroAcc.x / gyroAcc.totalVector) * -57.296;

  angleVals.pitchAcc -= 0.0;
  angleVals.rollAcc -= 0.0;

  if (set_gyro_angles)
  {
    angleVals.pitch = angleVals.pitch * 0.9996 + angleVals.pitchAcc * 0.0004;
    angleVals.roll = angleVals.roll * 0.9996 + angleVals.rollAcc * 0.0004;
  }
  else
  {
    angleVals.pitch = angleVals.pitchAcc;
    angleVals.roll = angleVals.rollAcc;
    set_gyro_angles = true;
  }

  angleVals.pitchOut = angleVals.pitchOut * 0.9 + angleVals.pitch * 0.1;
  angleVals.rollOut = angleVals.rollOut * 0.9 + angleVals.roll * 0.1;

  angleVals.pitchAdjust = angleVals.pitchOut * 15;
  angleVals.rollAdjust = angleVals.rollOut * 15;

  float uptake = 0.2;
  float oneMinusUptake = 1 - uptake;

  pid.input.pitch = (pid.input.pitch * oneMinusUptake) + (gyroVal.x * uptake);
  pid.gyro.roll = (pid.gyro.roll * oneMinusUptake) + (gyroVal.y * uptake);
  pid.input.yaw = (pid.input.yaw * oneMinusUptake) + (gyroVal.z * uptake);

  pid.setpoint.roll = 0;
  if (inputVals.ch1 > (PWM_MID + 8))
    pid.setpoint.roll = inputVals.ch1 - (PWM_MID + 8);
  else if (inputVals.ch1 < (PWM_MID - 8))
    pid.setpoint.roll = inputVals.ch1 - (PWM_MID - 8);
  pid.setpoint.roll -= angleVals.rollAdjust;
  pid.setpoint.roll /= 3.0;

  pid.setpoint.pitch = 0;
  if (inputVals.ch2 > (PWM_MID + 8))
    pid.setpoint.pitch = inputVals.ch2 - (PWM_MID + 8);
  else if (inputVals.ch2 < (PWM_MID - 8))
    pid.setpoint.pitch = inputVals.ch2 - (PWM_MID - 8);
  pid.setpoint.pitch -= angleVals.pitchAdjust;
  pid.setpoint.pitch /= 3.0;

  pid.setpoint.yaw = 0;
  if (inputVals.ch3 > (PWM_MID + 8))
    pid.setpoint.yaw = inputVals.ch3 - (PWM_MID + 8);
  else if (inputVals.ch3 < (PWM_MID - 8))
    pid.setpoint.yaw = inputVals.ch3 - (PWM_MID - 8);
  pid.setpoint.pitch /= 3.0;

  calculate_pid();

#ifdef unused
  serial_printF("RX roll: ");
  serial_print(inputVals.ch1);
  serial_printF(" pitch: ");
  serial_print(inputVals.ch2);
  serial_printF(" yaw: ");
  serial_print(inputVals.ch3);
  serial_printF(" knob: ");
  serial_println(inputVals.ch4);
#endif

  // a bit mixing
  servoVals.ch1 = inputVals.ch1 + pid.output.roll;     // ROLL
  servoVals.ch2 = inputVals.ch2 + pid.output.pitch;    // PITCH
  servoVals.ch3 = (PWM_MID - servoVals.ch1) + PWM_MID; // INVERTED ROLL
  servoVals.ch4 = inputVals.ch3 + pid.output.yaw;      // PITCH + YAW?

//#ifdef unused
  serial_printF("pid roll: ");
  serial_print(pid.output.roll);
  serial_printF(" pitch: ");
  serial_print(pid.output.pitch);
  serial_printF(" error: ");
  serial_println(pid.d_error.roll);
//#endif



  // wait until 0,4 miliseconds gone by (1000 micros are 1 milis, 1 second has 1.000.000 micros!)
  while (micros() - loop_start_time < 4000)
    ;
loop_start_time = micros();

loopCounter++;
if (loopCounter >= 0)
{

    loopCounter = 0;

    PORTD |= B11110000;
    unsigned long timer_channel_1 = servoVals.ch1 + loop_start_time;
    unsigned long timer_channel_2 = servoVals.ch2 + loop_start_time;
    unsigned long timer_channel_3 = servoVals.ch3 + loop_start_time;
    unsigned long timer_channel_4 = servoVals.ch4 + loop_start_time;

    // PWM out in a loop - initally set high for all 4 channels and look until all 4 channels gone by
    byte cnt = 0;
    while (cnt < 4) // leading to slowdown of the loop
    {
      cnt = 0;
      unsigned long esc_loop_start_time = micros();
      if (timer_channel_1 <= esc_loop_start_time)
      {
        PORTD &= B11101111;
        cnt++;
      }
      if (timer_channel_2 <= esc_loop_start_time)
      {
        PORTD &= B11011111;
        cnt++;
      }
      if (timer_channel_3 <= esc_loop_start_time)
      {
        PORTD &= B10111111;
        cnt++;
      }
      if (timer_channel_4 <= esc_loop_start_time)
      {
        PORTD &= B01111111;
        cnt++;
      }
    }
  }
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/* TODO code is wrong! */
void inline calculate(PIDStruct *pid, pidgainStruct *gain)
{
  float temp_pid_error;

  temp_pid_error = pid->input.pitch - pid->setpoint.pitch; // check error for pitch versus setpoint
  pid->i_mem.pitch += gain->i * temp_pid_error; // integrate error for pitch over gain
  pid->i_mem.pitch = constrain(pid->i_mem.pitch, -gain->max, gain->max); // constrain pitch integration in cage

  pid->output.pitch = gain->p * temp_pid_error + pid->i_mem.pitch + gain->d * (temp_pid_error - pid->d_error.pitch); // calculate output pitch with knob, error, integration and gain by acutal error minus last error
  pid->output.pitch = constrain(pid->output.pitch, -gain->max, gain->max); // keep pitch in cage
  pid->d_error.pitch = temp_pid_error; // remember last error
}

/**
 * TODO: understand :-)
 */

void calculate_pid()
{
#ifdef irrelevant
  calculate(&pid, &gainpitch);
  calculate(&pid, &gainroll);
  calculate(&pid, &gainyaw);
#endif // unused

  float pid_error_temp;

  pid_error_temp = pid.input.pitch - pid.setpoint.pitch; // check error for pitch versus setpoint
  pid.i_mem.pitch += gainpitch.i * pid_error_temp; // integrate error for pitch over gain
  pid.i_mem.pitch = constrain(pid.i_mem.pitch, -gainpitch.max, gainpitch.max); // constrain pitch integration in cage

  pid.output.pitch = gainpitch.p * pid_error_temp + pid.i_mem.pitch + gainpitch.d * (pid_error_temp - pid.d_error.pitch); // calculate output pitch with knob, error, integration and gain by acutal error minus last error
  pid.output.pitch = constrain(pid.output.pitch, -gainpitch.max, gainpitch.max); // keep pitch in cage
  pid.d_error.pitch = pid_error_temp; // remember last error for pitch


  pid_error_temp = pid.gyro.roll - pid.setpoint.roll; // check error versus setpoint
  pid.i_mem.roll += gainroll.i * pid_error_temp; // integrate error for roll over gain
  pid.i_mem.roll = constrain(pid.i_mem.roll, -gainroll.max, gainroll.max); // keep roll in cage

  pid.output.roll = gainroll.p * pid_error_temp + pid.i_mem.roll + gainroll.d * (pid_error_temp - pid.d_error.roll); // calulate output roll, with knob, error, integration, gain by current error minus last error
  pid.output.roll = constrain(pid.output.roll, -gainroll.max, gainroll.max); // keep roll output in cage
  pid.d_error.roll = pid_error_temp; // remember last error for roll


  pid_error_temp = pid.input.yaw - pid.setpoint.yaw;  // check error versus setpoint
  pid.i_mem.yaw += gainyaw.i * pid_error_temp; // integrate error for yaw over gain;
  pid.i_mem.yaw = constrain(pid.i_mem.yaw, -gainyaw.max_i, gainyaw.max_i); // keep yaw output in cage

  pid.output.yaw = gainyaw.p * pid_error_temp + pid.i_mem.yaw + gainyaw.d * (pid_error_temp - pid.d_error.yaw); // calulate outputch yaw, with knob, error, integration, gain by current error minus last error
  pid.output.yaw = constrain(pid.output.yaw, -gainyaw.max, gainyaw.max); // keep output for yaw in cage
  pid.d_error.yaw = pid_error_temp; // remember last error for yaw

}

void setup_MPU()
{
  if(!mpu_setup())
  {
    //while (1)
    {
      serial_printlnF("no gyro - startup failed!");
      //while (1)
      {
        delay(250);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    }
  }

#ifdef unused
  serial_printlnF("calculated Offsets are");
  serial_printF("pitch offset: ");
  serial_println(gyroCal.x);
  serial_printF("roll offset: ");
  serial_println(gyroCal.y);
  serial_printF("yaw offset: ");
  serial_println(gyroCal.z);

  for (;;)
  {
    xyzFloat g = IMU.getGyrValues();
    serial_printF("g.x: ");
    serial_print(g.x);
    serial_printF(" g.y: ");
    serial_print(g.y);
    serial_printF(" g.z: ");
    serial_print(g.z);
    serial_printlnF("");

    delay(500);
  }
#endif  
}

void cycle_CPPM()
{
  CPPM.cycle(); // update some variables and check timeouts...

  if (CPPM.synchronized())
  {
    inputVals.ch1 = CPPM.read_us(0);
    inputVals.ch2 = CPPM.read_us(1);
    inputVals.ch3 = CPPM.read_us(2);
    inputVals.ch4 = CPPM.read_us(3);
  }
}

void setup_Wire()
{
  serial_printlnF("setup wire");
  Wire.begin();
  Wire.setClock(400UL * 1000UL); // 400 kHz

  /**
   * TODO: change method with different plaform
   */
#if F_CPU == 16000000L
  const bool clockspeed_ok = true;
#else
  const bool clockspeed_ok = false;
#endif

  if (TWBR == 12 && clockspeed_ok)
  {
    serial_printlnF("clock speeds correctly set");
  }
  else
  {
    serial_printlnF("I2C clock speed is not set to 400kHz. (ERROR 8)");
    while (1)
    {
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }

}

void setup_SERVO()
{
  serial_printlnF("setting DIGITAL PIN 4, 5, 6, 7 as OUTPUTS");

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
}