//========================================================Author:SLOMOGANGSTA(VSYK)===========================================================
//========================================================Date: 27th January 2019=============================================================
#include "main.h"
#include "mpu.hpp"
#include <jm_CPPM.h>


gyroStruct gyro[GYRO_MAX];
pidgainStruct gain[CHANNEL_MAX];

angleValStruct angleVals[CHANNEL_MAX];

volatile channelValStruct inputVals;
channelValStruct servoVals;

PIDStruct pid[CHANNEL_MAX];


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
  // channel input 4 intensivity mapped to value between 0.07 and 0.17
  // onliner
  float pinten = inputVals.ch4 <= PWM_MIN?0:mapf(inputVals.ch4, PWM_MIN, (PWM_MAX + 50), 0.07, 0.17);
  
  // only one array iteration
  for(pidgainStruct &g : gain)
  {
    g.d = g.p = pinten;
  }

  if (!mpu_read_data(&gyro[GACC], &gyro[GVAL]))
  {
    delay(1000);
    return;
  }

  gyro[GVAL].x -= gyro[GCAL].x;
  gyro[GVAL].y -= gyro[GCAL].y;
  gyro[GVAL].z -= gyro[GCAL].z;


  angleVals[PITCH].chan += gyro[GVAL].x * 0.0000611;
  angleVals[ROLL].chan += gyro[GVAL].y * 0.0000611;

  angleVals[PITCH].chan += angleVals[ROLL].chan * sin(gyro[GVAL].z * 0.000001066);
  angleVals[ROLL].chan -= angleVals[PITCH].chan * sin(gyro[GVAL].z * 0.000001066);

  gyro[GACC].totalVector = sqrt((gyro[GACC].x * gyro[GACC].x) + (gyro[GACC].y * gyro[GACC].y) + (gyro[GACC].z * gyro[GACC].z));
  angleVals[PITCH].acc = asin((float)gyro[GACC].y / gyro[GACC].totalVector) * 57.296;
  angleVals[ROLL].acc = asin((float)gyro[GACC].x / gyro[GACC].totalVector) * -57.296;

  angleVals[PITCH].acc -= 0.0;
  angleVals[ROLL].acc -= 0.0;

  // TODO: check for neccesity
  if (set_gyro_angles)
  {
    angleVals[PITCH].chan = angleVals[PITCH].chan * 0.9996 + angleVals[PITCH].acc * 0.0004;
    angleVals[ROLL].chan = angleVals[ROLL].chan * 0.9996 + angleVals[ROLL].acc * 0.0004;
  }
  else
  {
    angleVals[PITCH].chan = angleVals[PITCH].acc;
    angleVals[ROLL].chan = angleVals[ROLL].acc;
    set_gyro_angles = true;
  }

  angleVals[PITCH].out = angleVals[PITCH].out * 0.9 + angleVals[PITCH].chan * 0.1;
  angleVals[ROLL].out = angleVals[ROLL].out * 0.9 + angleVals[ROLL].chan * 0.1;

  angleVals[PITCH].adjust = angleVals[PITCH].out * 15;
  angleVals[ROLL].adjust = angleVals[ROLL].out * 15;

  float uptake = 0.2;
  float oneMinusUptake = 1 - uptake;

  

  pid[ROLL].input = (pid[ROLL].input * oneMinusUptake) + (gyro[GVAL].x * uptake);
  pid[ROLL].gyro = (pid[ROLL].gyro * oneMinusUptake) + (gyro[GVAL].y * uptake);
  pid[YAW].input = (pid[YAW].input * oneMinusUptake) + (gyro[GVAL].z * uptake);

  pid[ROLL].setpoint = 0;
  if (inputVals.ch1 > (PWM_MID + 8))
    pid[ROLL].setpoint = inputVals.ch1 - (PWM_MID + 8);
  else if (inputVals.ch1 < (PWM_MID - 8))
    pid[ROLL].setpoint = inputVals.ch1 - (PWM_MID - 8);
  pid[ROLL].setpoint -= angleVals[ROLL].adjust;
  pid[ROLL].setpoint /= 3.0;

  
  pid[PITCH].setpoint = 0;
  if (inputVals.ch2 > (PWM_MID + 8))
    pid[PITCH].setpoint = inputVals.ch2 - (PWM_MID + 8);
  else if (inputVals.ch2 < (PWM_MID - 8))
    pid[PITCH].setpoint = inputVals.ch2 - (PWM_MID - 8);
  pid[PITCH].setpoint -= angleVals[PITCH].adjust;
  pid[PITCH].setpoint /= 3.0;


  pid[YAW].setpoint = 0;
  if (inputVals.ch3 > (PWM_MID + 8))
    pid[YAW].setpoint = inputVals.ch3 - (PWM_MID + 8);
  else if (inputVals.ch3 < (PWM_MID - 8))
    pid[YAW].setpoint = inputVals.ch3 - (PWM_MID - 8);
  pid[YAW].setpoint /= 3.0;

  calculate_pid();


  // a bit mixing
  
  servoVals.ch1 = inputVals.ch1 + pid[ROLL].output;     // ROLL
  servoVals.ch2 = inputVals.ch2 + pid[PITCH].output;    // PITCH
  servoVals.ch3 = (PWM_MID - servoVals.ch1) + PWM_MID; // INVERTED ROLL
  servoVals.ch4 = inputVals.ch3 + pid[YAW].output;      // PITCH + YAW?

  serial_printF("angleVals[PITCH].Chan: ");
  serial_print(angleVals[PITCH].chan);
  serial_printF(" angleVals[ROLL].Chan: ");
  serial_println(angleVals[ROLL].chan);

#ifdef unused
  serial_printF("gyro[GVAL].x: ");
  serial_print(gyro[GVAL].x);
  serial_printF(" gyro[GVAL].y: ");
  serial_print(gyro[GVAL].y);
  serial_printF(" gyro[GVAL].z: ");
  serial_println(gyro[GVAL].z);
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
  serial_print(pid[ROLL].output);
  serial_printF(" PITCH: ");
  serial_println(pid[PITCH].output);
#endif

#ifdef unused
  serial_printF("setpoint roll: ");
  serial_print(pid[ROLL].setpoint);
  serial_printF(" setpoint pitch: ");
  serial_println(pid[PITCH].setpoint);
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
  serial_print(pid[ROLL].output);
  serial_printF(" pitch: ");
  serial_print(pid[PITCH].output);
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
void inline calculate(PIDStruct *pid, pidgainStruct *gain)
{
  float temp_pid_error;

  temp_pid_error = pid->input - pid->setpoint; // check error for pitch versus setpoint

  pid->i_mem += gain->i * temp_pid_error; // integrate error for pitch over gain
  pid->i_mem = constrain(pid->i_mem, -gain->max, gain->max); // constrain pitch integration in cage

  pid->output = gain->p * temp_pid_error + pid->i_mem + gain->d * (temp_pid_error - pid->d_error); // calculate output pitch with knob, error, integration and gain by acutal error minus last error
  pid->output = constrain(pid->output, -gain->max, gain->max); // keep pitch in cage
  pid->d_error = temp_pid_error; // remember last error
}

/**
 * TODO: understand :-)
 */

void calculate_pid()
{
  calculate(&pid[PITCH], &gain[PITCH]);
  calculate(&pid[ROLL], &gain[ROLL]);
  calculate(&pid[YAW], &gain[YAW]);
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
  serial_println(gyro[GCAL].x);
  serial_printF("roll offset: ");
  serial_println(gyro[GCAL].y);
  serial_printF("yaw offset: ");
  serial_println(gyro[GCAL].z);

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