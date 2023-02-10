//========================================================Author:SLOMOGANGSTA(VSYK)===========================================================
//========================================================Date: 27th January 2019=============================================================
#include "main.h"

gyroStruct gyroVal;
gyroStruct gyroCal;
gyroStruct gyroAcc;

channelValStruct inputVals;
channelValStruct servoVals;
timerISRStruct lastVals;

pidgainStruct gainroll;
pidgainStruct gainpitch;
pidgainStruct gainyaw;

angleValStruct angleVals;

boolean set_gyro_angles;

struct RollPitchYawStruct
{
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
};

struct PIDStruct
{
  RollPitchYawStruct i_mem;
  RollPitchYawStruct input;
  RollPitchYawStruct output;
  RollPitchYawStruct setpoint;
  RollPitchYawStruct d_error;
};

PIDStruct pid;


//float pid_i_mem_roll = 0, pid_roll_setpoint = 0, gyro_roll_input = 0, pid_output_roll = 0, pid_last_roll_d_error = 0;
//float pid_i_mem_pitch = 0, pid_pitch_setpoint = 0, gyro_pitch_input = 0, pid_output_pitch = 0, pid_last_pitch_d_error = 0;
//float pid_i_mem_yaw = 0, pid_yaw_setpoint = 0, gyro_yaw_input = 0, pid_output_yaw = 0, pid_last_yaw_d_error = 0;

bool clockspeed_ok = false;
int temperature = 0;
int loopCounter = 0;
unsigned long loop_start_time = 0;
float pinten = 0;


// forward declaration
void setup_mpu_6050_registers();
void read_mpu_6050_data();
double mapf(double val, double in_min, double in_max, double out_min, double out_max);
void calculate_pid();
void PWM();

void setup()
{
  Serial.begin(57600);
  Wire.begin();

  // setup PWM interrupts
  pinMode(0, INPUT);
  attachInterrupt(digitalPinToInterrupt(0), PWM, CHANGE);

  pinMode(1, INPUT);
  attachInterrupt(digitalPinToInterrupt(1), PWM, CHANGE);

  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), PWM, CHANGE);

  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), PWM, CHANGE);

  Serial.println("Interrupts Enabled Successfully");
  delay(100);

  Serial.println(F("Checking I2C clock speed."));
  delay(100);

  Wire.setClock(400UL * 1000UL);

#if F_CPU == 16000000L
  clockspeed_ok = true;
  Serial.println(F("CPU clockspeed ok"));
#endif

  /**
   * TODO: change method with different plaform
  */
  if (TWBR == 12 && clockspeed_ok)
  {
    Serial.println(F("I2C clock speed is correctly set to 400kHz."));
  }
  else
  {
    Serial.println(F("I2C clock speed is not set to 400kHz. (ERROR 8)"));
    exit(0);
  }

  Serial.println("Setting DIGITAL PIN 4 , 5 , 6 ,7 as OUTPUTS");

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  Serial.println("Setting up registers of MPU6050");
  setup_mpu_6050_registers();

  Serial.println("Calculating Offset ");
  for (int cal_int = 0; cal_int < 2000; cal_int++)
  {
    if (cal_int % 125 == 0)
      Serial.print(".");
    read_mpu_6050_data();
    gyroCal.x += gyroVal.x;
    gyroCal.y += gyroVal.y;
    gyroCal.z += gyroVal.z;
    delay(3);
  }
  gyroCal.x /= 2000;
  gyroCal.y /= 2000;
  gyroCal.z /= 2000;

  Serial.println("Calculated Offsets are");
  Serial.print("Pitch offset: ");
  Serial.println(gyroCal.x);
  Serial.print("Roll offset: ");
  Serial.println(gyroCal.y);
  Serial.print("Yaw offset: ");
  Serial.println(gyroCal.z);
}

void loop()
{
  // Serial.print("Receiver Input 3: ");
  // Serial.println(inputVals.channel3);
  if (inputVals.channel4 < 1000)
  {
    gainroll.d = 0;
    gainroll.p = 0;
    gainpitch.d = 0;
    gainpitch.p = 0;
    gainyaw.p = 0;
    gainyaw.d = 0;
    // Serial.println("INTENSITY : 0");
  }

  else
  {
    pinten = mapf(inputVals.channel4, 1000, 2050, 0.07, 0.17);
    gainroll.d = pinten;
    gainroll.p = pinten;
    gainpitch.d = pinten;
    gainpitch.p = pinten;
    gainyaw.p = pinten;
    gainyaw.d = pinten;
    // Serial.print("INTENSITY: ");
    // Serial.println(pinten);
  }

  read_mpu_6050_data();

  gyroVal.x -= gyroCal.x;
  gyroVal.y -= gyroCal.y;
  gyroVal.z -= gyroCal.z;

  angleVals.pitch += gyroVal.x * 0.0000611;
  angleVals.roll += gyroVal.y * 0.0000611;

  angleVals.pitch += angleVals.roll * sin(gyroVal.z * 0.000001066);
  angleVals.roll -= angleVals.pitch * sin(gyroVal.z * 0.000001066);

  gyroAcc.totalvector = sqrt((gyroAcc.x * gyroAcc.x) + (gyroAcc.y * gyroAcc.y) + (gyroAcc.z * gyroAcc.z));
  angleVals.pitch_acc = asin((float)gyroAcc.y / gyroAcc.totalvector) * 57.296;
  angleVals.roll_acc = asin((float)gyroAcc.x / gyroAcc.totalvector) * -57.296;

  angleVals.pitch_acc -= 0.0;
  angleVals.roll_acc -= 0.0;

  if (set_gyro_angles)
  {
    angleVals.pitch = angleVals.pitch * 0.9996 + angleVals.pitch_acc * 0.0004;
    angleVals.roll = angleVals.roll * 0.9996 + angleVals.roll_acc * 0.0004;
  }
  else
  {
    angleVals.pitch = angleVals.pitch_acc;
    angleVals.roll = angleVals.roll_acc;
    set_gyro_angles = true;
  }

  angleVals.pitch_out = angleVals.pitch_out * 0.9 + angleVals.pitch * 0.1;
  angleVals.roll_out = angleVals.roll_out * 0.9 + angleVals.roll * 0.1;

  angleVals.pitch_adjust = angleVals.pitch_out * 15;
  angleVals.roll_adjust = angleVals.roll_out * 15;

  float uptake = 0.2;
  float oneMinusUptake = 1 - uptake;

  gyro_pitch_input = (gyro_pitch_input * oneMinusUptake) + (gyroVal.x * uptake);
  gyro_roll_input = (gyro_roll_input * oneMinusUptake) + (gyroVal.y * uptake);
  gyro_yaw_input = (gyro_yaw_input * oneMinusUptake) + (gyroVal.z * uptake);

  pid_roll_setpoint = 0;
  if (inputVals.channel1 > 1508)
    pid_roll_setpoint = inputVals.channel1 - 1508;
  else if (inputVals.channel1 < 1492)
    pid_roll_setpoint = inputVals.channel1 - 1492;
  pid_roll_setpoint -= angleVals.roll_adjust;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;
  if (inputVals.channel2 > 1508)
    pid_pitch_setpoint = inputVals.channel2 - 1508;
  else if (inputVals.channel2 < 1492)
    pid_pitch_setpoint = inputVals.channel2 - 1492;
  pid_pitch_setpoint -= angleVals.pitch_adjust;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;
  if (inputVals.channel3 > 1508)
    pid_yaw_setpoint = inputVals.channel3 - 1508;
  else if (inputVals.channel3 < 1492)
    pid_yaw_setpoint = inputVals.channel3 - 1492;
  pid_pitch_setpoint /= 3.0;

  calculate_pid();

  Serial.print("Receiver Roll: ");
  Serial.println(inputVals.channel1);
  Serial.print("Receiver Pitch: ");
  Serial.println(inputVals.channel2);
  Serial.print("Receiver Yaw: ");
  Serial.println(inputVals.channel3);
  Serial.print("Intensity Knob: ");
  Serial.println(inputVals.channel4);

  servoVals.channel1 = inputVals.channel1 + pid_output_roll;
  servoVals.channel2 = inputVals.channel2 + pid_output_pitch;
  servoVals.channel3 = servoVals.channel1;
  servoVals.channel3 = (1500 - servoVals.channel1) + 1500;
  servoVals.channel4 = inputVals.channel3 + pid_output_yaw;

  Serial.print("Calculated roll input : ");
  Serial.println(servoVals.channel1);
  Serial.print("Calculated pitch input : ");
  Serial.println(servoVals.channel2);
  Serial.print("Calculated Yaw input : ");
  Serial.println(servoVals.channel4);

  while (micros() - loop_start_time < 4000)
    ;
  loop_start_time = micros();

  loopCounter++;

  if (loopCounter >= 0)
  {

    loopCounter = 0;

    PORTD |= B11110000;
    unsigned long timer_channel_1 = servoVals.channel1 + loop_start_time;
    unsigned long timer_channel_2 = servoVals.channel2 + loop_start_time;
    unsigned long timer_channel_3 = servoVals.channel3 + loop_start_time;
    unsigned long timer_channel_4 = servoVals.channel4 + loop_start_time;

    byte cnt = 0;
    while (cnt < 4)
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

void setup_mpu_6050_registers()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(1000);
  // Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  delay(1000);
  // Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(1000);
}

void read_mpu_6050_data()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14)
    ;
  gyroAcc.x = Wire.read() << 8 | Wire.read();
  gyroAcc.y = Wire.read() << 8 | Wire.read();
  gyroAcc.z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyroVal.x = Wire.read() << 8 | Wire.read();
  gyroVal.y = Wire.read() << 8 | Wire.read();
  gyroVal.z = Wire.read() << 8 | Wire.read();
  gyroAcc.x *= -1;
  gyroVal.y *= -1;
}

void calculate_pid()
{
  float pid_error_temp;
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += gainpitch.i * pid_error_temp;
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -gainpitch.max, gainpitch.max);

  pid_output_pitch = gainpitch.p * pid_error_temp + pid_i_mem_pitch + gainpitch.d * (pid_error_temp - pid_last_pitch_d_error);
  pid_output_pitch = constrain(pid_output_pitch, -gainpitch.max, gainpitch.max);

  pid_last_pitch_d_error = pid_error_temp;

  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += gainroll.i * pid_error_temp;
  pid_i_mem_roll = constrain(pid_i_mem_roll, -gainroll.max, gainroll.max);

  pid_output_roll = gainroll.p * pid_error_temp + pid_i_mem_roll + gainroll.d * (pid_error_temp - pid_last_roll_d_error);
  pid_output_roll = constrain(pid_output_roll, -gainroll.max, gainroll.max);

  pid_last_roll_d_error = pid_error_temp;

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += gainyaw.i * pid_error_temp;
  pid_i_mem_yaw = constrain(pid_i_mem_yaw, -gainyaw.max_i, gainyaw.max_i);

  pid_output_yaw = gainyaw.p * pid_error_temp + pid_i_mem_yaw + gainyaw.d * (pid_error_temp - pid_last_yaw_d_error);
  pid_output_yaw = constrain(pid_output_yaw, -gainyaw.max, gainyaw.max);

  pid_last_yaw_d_error = pid_error_temp;
}

// ISR for PWM values
void PWM()
{

  if (lastVals.channel1 == 0 && PINB & B00000001)
  {
    lastVals.channel1 = 1;
    lastVals.timer1 = micros();
  }
  else if (lastVals.channel1 == 1 && !(PINB & B00000001))
  {
    lastVals.channel1 = 0;
    inputVals.channel1 = micros() - lastVals.timer1;
  }

  if (lastVals.channel2 == 0 && PINB & B00000010)
  {
    lastVals.channel2 = 1;
    lastVals.timer2 = micros();
  }
  else if (lastVals.channel2 == 1 && !(PINB & B00000010))
  {
    lastVals.channel2 = 0;
    inputVals.channel2 = micros() - lastVals.timer2;
  }

  if (lastVals.channel3 == 0 && PINB & B00000100)
  {
    lastVals.channel3 = 1;
    lastVals.timer3 = micros();
  }
  else if (lastVals.channel3 == 1 && !(PINB & B00000100))
  {
    lastVals.channel3 = 0;
    inputVals.channel3 = micros() - lastVals.timer3;
  }

  if (lastVals.channel4 == 0 && PINB & B00001000)
  {
    lastVals.channel4 = 1;
    lastVals.timer4 = micros();
  }
  else if (lastVals.channel4 == 1 && !(PINB & B00001000))
  {
    lastVals.channel4 = 0;
    inputVals.channel4 = micros() - lastVals.timer4;
  }
}
