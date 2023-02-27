//========================================================Author:SLOMOGANGSTA(VSYK)===========================================================
//========================================================Date: 27th January 2019=============================================================
#include "main.h"
#include "mpu.hpp"
//#include <CPPM.h>
#include <jm_CPPM.h>

gyroStruct gyroVal;
gyroStruct gyroCal;
gyroStruct gyroAcc;


pidgainStruct gain[CHANNEL_MAX];

angleValStruct angleVals;

volatile channelValStruct inputVals;
channelValStruct servoVals;

PIDStruct pid;


double mapf(double val, double in_min, double in_max, double out_min, double out_max);

void calculate_pid();
void setup_MPU();
void setup_SERVO();
void cycle_CPPM();
void setup_Wire();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  CPPM.begin(); // setup CPPM - will be called in loop

  setup_Wire(); // setup Wire for I2C incl. check of speed

  setup_MPU();  // setup I2C device

  setup_SERVO(); // setup output pins

  delay(2500);
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
    gain[ROLL].d = 0;
    gain[ROLL].p = 0;
    gain[PITCH].d = 0;
    gain[PITCH].p = 0;
    gain[YAW].p = 0;
    gain[YAW].d = 0;
  }
  else
  {
    // channel input 4 intensivity mapped to value between 0.07 and 0.17
    pinten = mapf(inputVals.ch4, PWM_MIN, (PWM_MAX + 50), 0.07, 0.17);
    gain[ROLL].d = pinten;
    gain[ROLL].p = pinten;
    gain[PITCH].d = pinten;
    gain[PITCH].p = pinten;
    gain[YAW].p = pinten;
    gain[YAW].d = pinten;
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


  angleVals.Chan[PITCH] += gyroVal.x * 0.0000611;
  angleVals.Chan[ROLL] += gyroVal.y * 0.0000611;

  angleVals.Chan[PITCH] += angleVals.Chan[ROLL] * sin(gyroVal.z * 0.000001066);
  angleVals.Chan[ROLL] -= angleVals.Chan[PITCH] * sin(gyroVal.z * 0.000001066);

  gyroAcc.totalVector = sqrt((gyroAcc.x * gyroAcc.x) + (gyroAcc.y * gyroAcc.y) + (gyroAcc.z * gyroAcc.z));
  angleVals.Acc[PITCH] = asin((float)gyroAcc.y / gyroAcc.totalVector) * 57.296;
  angleVals.Acc[ROLL] = asin((float)gyroAcc.x / gyroAcc.totalVector) * -57.296;

  angleVals.Acc[PITCH] -= 0.0;
  angleVals.Acc[ROLL] -= 0.0;

  if (set_gyro_angles)
  {
    angleVals.Chan[PITCH] = angleVals.Chan[PITCH] * 0.9996 + angleVals.Acc[PITCH] * 0.0004;
    angleVals.Chan[ROLL] = angleVals.Chan[ROLL] * 0.9996 + angleVals.Acc[ROLL] * 0.0004;
  }
  else
  {
    angleVals.Chan[PITCH] = angleVals.Acc[PITCH];
    angleVals.Chan[ROLL] = angleVals.Acc[ROLL];
    set_gyro_angles = true;
  }

  angleVals.Out[PITCH] = angleVals.Out[PITCH] * 0.9 + angleVals.Chan[PITCH] * 0.1;
  angleVals.Out[ROLL] = angleVals.Out[ROLL] * 0.9 + angleVals.Chan[ROLL] * 0.1;

  angleVals.Adjust[PITCH] = angleVals.Out[PITCH] * 15;
  angleVals.Adjust[ROLL] = angleVals.Out[ROLL] * 15;

  float uptake = 0.2;
  float oneMinusUptake = 1 - uptake;

  

  pid.input.chan[PITCH] = (pid.input.chan[PITCH] * oneMinusUptake) + (gyroVal.x * uptake);
  pid.gyro.chan[ROLL] = (pid.gyro.chan[ROLL] * oneMinusUptake) + (gyroVal.y * uptake);
  pid.input.chan[YAW] = (pid.input.chan[YAW] * oneMinusUptake) + (gyroVal.z * uptake);

  pid.setpoint.chan[ROLL] = 0;
  if (inputVals.ch1 > (PWM_MID + 8))
    pid.setpoint.chan[ROLL] = inputVals.ch1 - (PWM_MID + 8);
  else if (inputVals.ch1 < (PWM_MID - 8))
    pid.setpoint.chan[ROLL] = inputVals.ch1 - (PWM_MID - 8);
  pid.setpoint.chan[ROLL] -= angleVals.Adjust[ROLL];
  pid.setpoint.chan[ROLL] /= 3.0;

  
  pid.setpoint.chan[PITCH] = 0;
  if (inputVals.ch2 > (PWM_MID + 8))
    pid.setpoint.chan[PITCH] = inputVals.ch2 - (PWM_MID + 8);
  else if (inputVals.ch2 < (PWM_MID - 8))
    pid.setpoint.chan[PITCH] = inputVals.ch2 - (PWM_MID - 8);
  pid.setpoint.chan[PITCH] -= angleVals.Adjust[PITCH];
  pid.setpoint.chan[PITCH] /= 3.0;


  pid.setpoint.chan[YAW] = 0;
  if (inputVals.ch3 > (PWM_MID + 8))
    pid.setpoint.chan[YAW] = inputVals.ch3 - (PWM_MID + 8);
  else if (inputVals.ch3 < (PWM_MID - 8))
    pid.setpoint.chan[YAW] = inputVals.ch3 - (PWM_MID - 8);
  pid.setpoint.chan[YAW] /= 3.0;

  calculate_pid();


  // a bit mixing
  
  servoVals.ch1 = inputVals.ch1 + pid.output.chan[ROLL];     // ROLL
  servoVals.ch2 = inputVals.ch2 + pid.output.chan[PITCH];    // PITCH
  servoVals.ch3 = (PWM_MID - servoVals.ch1) + PWM_MID; // INVERTED ROLL
  servoVals.ch4 = inputVals.ch3 + pid.output.chan[YAW];      // PITCH + YAW?

  serial_printF("angleVals.Chan[PITCH]: ");
  serial_print(angleVals.Chan[PITCH]);
  serial_printF(" angleVals.Chan[ROLL]: ");
  serial_println(angleVals.Chan[ROLL]);

#ifdef unused
  serial_printF("gyroVal.x: ");
  serial_print(gyroVal.x);
  serial_printF(" gyroVal.y: ");
  serial_print(gyroVal.y);
  serial_printF(" gyroVal.z: ");
  serial_println(gyroVal.z);
#endif

#ifdef unused
      serial_printF("RX roll: ");
  serial_print(servoVals.ch1);
  serial_printF(" pitch: ");
  serial_print(servoVals.ch2);
  serial_printF(" inverted roll: ");
  serial_print(servoVals.ch3);
  serial_printF(" yaw: ");
  serial_println(servoVals.ch4);
#endif

#ifdef unused
  serial_printF("pid output ROLL: ");
  serial_print(pid.output.chan[ROLL]);
  serial_printF(" PITCH: ");
  serial_println(pid.output.chan[PITCH]);
#endif

#ifdef unused
  serial_printF("setpoint roll: ");
  serial_print(pid.setpoint.chan[ROLL]);
  serial_printF(" setpoint pitch: ");
  serial_println(pid.setpoint.chan[PITCH]);
#endif

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

#ifdef unused
  serial_printF("pid roll: ");
  serial_print(pid.output.chan[ROLL]);
  serial_printF(" pitch: ");
  serial_print(pid.output.chan[PITCH]);
  serial_printF(" error: ");
  serial_println(pid.d_error.chan[ROLL]);
#endif


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


/* TODO check if code works */
void inline calculate(PIDStruct *pid, pidgainStruct *gain, int chan)
{
  float temp_pid_error;


  temp_pid_error = pid->input.chan[chan] - pid->setpoint.chan[chan]; // check error for pitch versus setpoint

  pid->i_mem.chan[chan] += gain->i * temp_pid_error; // integrate error for pitch over gain
  pid->i_mem.chan[chan] = constrain(pid->i_mem.chan[chan], -gain->max, gain->max); // constrain pitch integration in cage

  pid->output.chan[chan] = gain->p * temp_pid_error + pid->i_mem.chan[chan] + gain->d * (temp_pid_error - pid->d_error.chan[chan]); // calculate output pitch with knob, error, integration and gain by acutal error minus last error
  pid->output.chan[chan] = constrain(pid->output.chan[chan], -gain->max, gain->max); // keep pitch in cage
  pid->d_error.chan[chan] = temp_pid_error; // remember last error
}

/**
 * TODO: understand :-)
 */

void calculate_pid()
{
  calculate(&pid, &gain[PITCH], PITCH);
  calculate(&pid, &gain[ROLL], ROLL);
  calculate(&pid, &gain[YAW], YAW);

#ifdef irrelevant
  float pid_error_temp;

  pid_error_temp = pid.input.pitch - pid.setpoint.pitch; // check error for pitch versus setpoint
  pid.i_mem.pitch += gain[PITCH].i * pid_error_temp; // integrate error for pitch over gain
  pid.i_mem.pitch = constrain(pid.i_mem.pitch, -gain[PITCH].max, gain[PITCH].max); // constrain pitch integration in cage

  pid.output.pitch = gain[PITCH].p * pid_error_temp + pid.i_mem.pitch + gain[PITCH].d * (pid_error_temp - pid.d_error.pitch); // calculate output pitch with knob, error, integration and gain by acutal error minus last error
  pid.output.pitch = constrain(pid.output.pitch, -gain[PITCH].max, gain[PITCH].max); // keep pitch in cage
  pid.d_error.pitch = pid_error_temp; // remember last error for pitch


  pid_error_temp = pid.gyro.roll - pid.setpoint.roll; // check error versus setpoint
  pid.i_mem.roll += gain[ROLL].i * pid_error_temp; // integrate error for roll over gain
  pid.i_mem.roll = constrain(pid.i_mem.roll, -gain[ROLL].max, gain[ROLL].max); // keep roll in cage

  pid.output.roll = gain[ROLL].p * pid_error_temp + pid.i_mem.roll + gain[ROLL].d * (pid_error_temp - pid.d_error.roll); // calulate output roll, with knob, error, integration, gain by current error minus last error
  pid.output.roll = constrain(pid.output.roll, -gain[ROLL].max, gain[ROLL].max); // keep roll output in cage
  pid.d_error.roll = pid_error_temp; // remember last error for roll


  pid_error_temp = pid.input.yaw - pid.setpoint.yaw;  // check error versus setpoint
  pid.i_mem.yaw += gain[YAW].i * pid_error_temp; // integrate error for yaw over gain;
  pid.i_mem.yaw = constrain(pid.i_mem.yaw, -gain[YAW].max_i, gain[YAW].max_i); // keep yaw output in cage

  pid.output.yaw = gain[YAW].p * pid_error_temp + pid.i_mem.yaw + gain[YAW].d * (pid_error_temp - pid.d_error.yaw); // calulate outputch yaw, with knob, error, integration, gain by current error minus last error
  pid.output.yaw = constrain(pid.output.yaw, -gain[YAW].max, gain[YAW].max); // keep output for yaw in cage
  pid.d_error.yaw = pid_error_temp; // remember last error for yaw
#endif // unused

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

  pinMode(4, OUTPUT); // ROLL
  pinMode(5, OUTPUT); // PITCH
  pinMode(6, OUTPUT); // INVERTED ROLL
  pinMode(7, OUTPUT); // PITCH + YAW?

}