//========================================================Author:SLOMOGANGSTA(VSYK)===========================================================
//========================================================Date: 27th January 2019=============================================================
#include "main.h"
#include "mpu.hpp"
#include <jm_CPPM.h>

gyroStruct gyros[GYRO_MAX];
pidgainStruct gains[CHANNEL_MAX];
angleValStruct angles[CHANNEL_MAX];

volatile channelValStruct inputVals;
channelValStruct servoVals;

PIDStruct PIDs[CHANNEL_MAX];

void calculate_pid();
void setup_MPU();
void setup_SERVO();
void cycle_CPPM();
void setup_Wire();

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  setup_Wire(); // setup Wire for I2C incl. check of speed

  delay(2500);

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

  cycle_CPPM();

#ifdef unused
  static unsigned long lastTime = millis(); // initialisiert die Variable, um die letzte Ausführungszeit zu speichern
  static int onoff = HIGH;
  // blink LED according to channel 1 
  static unsigned long interval = 100;
  interval = map(inputVals.ch1, 700, 2300, 150, 50); 
  interval = constrain(interval, 50, 150);

  if (millis() >= (lastTime + interval))
  { 
    digitalWrite(LED_BUILTIN, onoff);
    onoff = !onoff;
    lastTime = millis(); // speichert die letzte Ausführungszeit
  }
 #endif

  // channel input 4 intensivity to zero if below PWM_MIN
  // channel input 4 intensivity mapped to value between 0.07 and 0.17
  // onliner
  float pinten = inputVals.ch4 <= PWM_MIN?0:mapf(inputVals.ch4, PWM_MIN, (PWM_MAX + 50), 0.07, 0.17);
  
  // only one array iteration
  for(pidgainStruct &g : gains)
  {
    g.d = g.p = pinten;
  }

  if (!mpu_read_data(&gyros[ACC], &gyros[VAL]))
  {
    delay(1000);
    return;
  }

  // apply correction values
  gyros[VAL].x -= gyros[CAL].x;
  gyros[VAL].y -= gyros[CAL].y;
  gyros[VAL].z -= gyros[CAL].z;


  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angles[PITCH].chan += gyros[VAL].x * 0.0000611;
  angles[ROLL].chan += gyros[VAL].y * 0.0000611;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angles[PITCH].chan += angles[ROLL].chan * sin(gyros[VAL].z * 0.000001066);
  angles[ROLL].chan -= angles[PITCH].chan * sin(gyros[VAL].z * 0.000001066);

  gyros[ACC].totalVector = sqrt((gyros[ACC].x * gyros[ACC].x) + (gyros[ACC].y * gyros[ACC].y) + (gyros[ACC].z * gyros[ACC].z));
  angles[PITCH].acc = asin((float)gyros[ACC].y / gyros[ACC].totalVector) * RAD_TO_DEG;
  angles[ROLL].acc = asin((float)gyros[ACC].x / gyros[ACC].totalVector) * -RAD_TO_DEG;

  // useless...
  angles[PITCH].acc -= 0.0;
  angles[ROLL].acc -= 0.0;

  // TODO: check for neccesity
  if (set_gyro_angles)
  {
    angles[PITCH].chan = angles[PITCH].chan * 0.9996 + angles[PITCH].acc * 0.0004;
    angles[ROLL].chan = angles[ROLL].chan * 0.9996 + angles[ROLL].acc * 0.0004;
  }
  else
  {
    angles[PITCH].chan = angles[PITCH].acc;
    angles[ROLL].chan = angles[ROLL].acc;
    set_gyro_angles = true;
  }

  /**
   * To dampen the pitch and roll angles a complementary filter is used
   **/
  
  //take 90% of the output pitch value and add 10% of the raw pitch value
  angles[PITCH].out = angles[PITCH].out * 0.9 + angles[PITCH].chan * 0.1;
  //take 90% of the output roll value and add 10% of the raw roll value
  angles[ROLL].out = angles[ROLL].out * 0.9 + angles[ROLL].chan * 0.1;

  angles[PITCH].adjust = angles[PITCH].out * 15;
  angles[ROLL].adjust = angles[ROLL].out * 15;

  float uptake = 0.2;
  float oneMinusUptake = 1 - uptake;

  

  PIDs[ROLL].input = (PIDs[ROLL].input * oneMinusUptake) + (gyros[VAL].x * uptake);
  PIDs[ROLL].gyro = (PIDs[ROLL].gyro * oneMinusUptake) + (gyros[VAL].y * uptake);
  PIDs[YAW].input = (PIDs[YAW].input * oneMinusUptake) + (gyros[VAL].z * uptake);

  PIDs[ROLL].setpoint = 0;
  if (inputVals.ch1 > (PWM_MID + 8))
    PIDs[ROLL].setpoint = inputVals.ch1 - (PWM_MID + 8);
  else if (inputVals.ch1 < (PWM_MID - 8))
    PIDs[ROLL].setpoint = inputVals.ch1 - (PWM_MID - 8);
  PIDs[ROLL].setpoint -= angles[ROLL].adjust;
  PIDs[ROLL].setpoint /= 3.0;

  
  PIDs[PITCH].setpoint = 0;
  if (inputVals.ch2 > (PWM_MID + 8))
    PIDs[PITCH].setpoint = inputVals.ch2 - (PWM_MID + 8);
  else if (inputVals.ch2 < (PWM_MID - 8))
    PIDs[PITCH].setpoint = inputVals.ch2 - (PWM_MID - 8);
  PIDs[PITCH].setpoint -= angles[PITCH].adjust;
  PIDs[PITCH].setpoint /= 3.0;


  PIDs[YAW].setpoint = 0;
  if (inputVals.ch3 > (PWM_MID + 8))
    PIDs[YAW].setpoint = inputVals.ch3 - (PWM_MID + 8);
  else if (inputVals.ch3 < (PWM_MID - 8))
    PIDs[YAW].setpoint = inputVals.ch3 - (PWM_MID - 8);
  PIDs[YAW].setpoint /= 3.0;

  calculate_pid();


  // a bit mixing
  servoVals.ch1 = inputVals.ch1 + PIDs[ROLL].output;     // ROLL input plus calculated PID-roll
  servoVals.ch2 = inputVals.ch2 + PIDs[PITCH].output;    // PITCH  input plus calculated PID-pitch
  servoVals.ch3 = (PWM_MID - servoVals.ch1) + PWM_MID; // INVERTED ROLL
  servoVals.ch4 = inputVals.ch3 + PIDs[YAW].output;      // YAW-input plus calculated PID-yaw

#ifdef unused
  serial_printF("angles[PITCH].Chan: ");
  serial_print(angles[PITCH].chan);
  serial_printF(" angles[ROLL].Chan: ");
  serial_println(angles[ROLL].chan);
#endif

  serial_printF("angle pitch: ");
  serial_print(angles[PITCH].out);
  serial_printF(" roll: ");
  serial_println(angles[ROLL].out);


#ifdef unused
  serial_printF("gyro[VAL].x: ");
  serial_print(gyro[VAL].x);
  serial_printF(" gyro[VAL].y: ");
  serial_print(gyro[VAL].y);
  serial_printF(" gyro[VAL].z: ");
  serial_println(gyro[VAL].z);
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
  serial_printF("PIDs output ROLL: ");
  serial_print(PIDs[ROLL].output);
  serial_printF(" PITCH: ");
  serial_println(PIDs[PITCH].output);
#endif

#ifdef unused
  serial_printF("setpoint roll: ");
  serial_print(PIDs[ROLL].setpoint);
  serial_printF(" setpoint pitch: ");
  serial_println(PIDs[PITCH].setpoint);
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
  serial_printF("PIDs roll: ");
  serial_print(PIDs[ROLL].output);
  serial_printF(" pitch: ");
  serial_print(PIDs[PITCH].output);
  serial_printF(" error: ");
  serial_println(PIDs.d_error.chan[ROLL]);
#endif


  // wait until at least 0,4 miliseconds gone by (1000 micros are 1 milis, 1 second has 1.000.000 micros!) since last time
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
  calculate(&PIDs[ROLL], &gains[ROLL]);
  calculate(&PIDs[PITCH], &gains[PITCH]);
  calculate(&PIDs[YAW], &gains[YAW]);
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
  serial_println(gyro[CAL].x);
  serial_printF("roll offset: ");
  serial_println(gyro[CAL].y);
  serial_printF("yaw offset: ");
  serial_println(gyro[CAL].z);

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
  CPPM.cycle(); // update some variables and check for timeouts...

  if (CPPM.synchronized()) // only in sync at specific timespans (TODO: more buffer?)
  {
    /*
    inputVals.ch1 = CPPM.read_us(0); // Receiver Roll
    inputVals.ch2 = CPPM.read_us(1); // Receiver Pitch
    inputVals.ch3 = CPPM.read_us(2); // Receiver Yaw
    inputVals.ch4 = CPPM.read_us(3); // Intensity Knob
    */

    inputVals.ch1 = CPPM.read_us(ROLL); // Receiver Roll
    inputVals.ch2 = CPPM.read_us(PITCH); // Receiver Pitch
    inputVals.ch3 = CPPM.read_us(YAW); // Receiver Yaw
    inputVals.ch4 = CPPM.read_us(KNOB); // Intensity Knob


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