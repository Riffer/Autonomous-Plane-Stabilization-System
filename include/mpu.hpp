#ifndef MPU_H
#define MPU_H

#include <Wire.h>

//#include <MPU9250_WE.h>
#include "main.h"


struct xyzFloat
{
    float x;
    float y;
    float z;
};



#define I2C_ERROR_MSG "I2C error"

uint8_t mpu_write_pair(int a, int b, bool waitAfterWriteMs=300)
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(a);
    Wire.write(b);
    uint8_t error = Wire.endTransmission();
    if (error)
    {
        serial_printlnF(I2C_ERROR_MSG);
        return error;
    }
    delay(waitAfterWriteMs);
    return error;
}

uint8_t mpu_read8(uint8_t reg)
{
    uint8_t error;
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(reg);
    error = Wire.endTransmission();

    if (error)
    {
        serial_printlnF(I2C_ERROR_MSG);
        return false;
    }

    Wire.requestFrom(MPU_ADDRESS, 1); // resets all 3 sensors

    //while (Wire.available() < 1); // wait until buffer has 1 bytes
    if(Wire.available())
    // read 8 bit pairs while shifting first byte to left, building into a full 16 bit (long integer)
        return (Wire.read());
    
    return 0;
}

uint8_t whoAmI()
{
    return mpu_read8(0x75);
}


uint64_t mpu_read3x16(uint8_t reg)
{
    uint8_t triple[6];
    uint64_t regValue = 0;

    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6);
    if (Wire.available())
    {
        for (int i = 0; i < 6; i++)
        {
            triple[i] = Wire.read();
        }
    }

    regValue = ((uint64_t)triple[0] << 40) + ((uint64_t)triple[1] << 32) + ((uint64_t)triple[2] << 24) +
               +((uint64_t)triple[3] << 16) + ((uint64_t)triple[4] << 8) + (uint64_t)triple[5];
    return regValue;
}


xyzFloat getRawValues(uint8_t reg)
{
    uint64_t const xyzDataReg = mpu_read3x16(reg);
    int16_t const xRaw = static_cast<int16_t>((xyzDataReg >> 32) & 0xFFFF);
    int16_t const yRaw = static_cast<int16_t>((xyzDataReg >> 16) & 0xFFFF);
    int16_t const zRaw = static_cast<int16_t>(xyzDataReg & 0xFFFF);

#ifdef unused
    serial_printF("xRaw: ");
    serial_print(xRaw);
    serial_printF(" yRaw: ");
    serial_print(yRaw);
    serial_printF(" zRaws: ");
    serial_println(zRaw);
#endif

    return xyzFloat{static_cast<float>(xRaw), static_cast<float>(yRaw), static_cast<float>(zRaw)};
}

int temp = 0;
bool mpu_read_data(gyroStruct *acc, gyroStruct *gyro) // reads the current acc and gyro in one take (from starting address)
{

    uint8_t error;
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    error = Wire.endTransmission();

    if (error)
    {
        serial_printlnF(I2C_ERROR_MSG);
        return false;
    }

    Wire.requestFrom(MPU_ADDRESS, 14); // resets all 3 sensors

    while (Wire.available() < 14){}; // wait until buffer has 14 bytes

    // read 8 bit pairs while shifting first byte to left, building into a full 16 bit (long integer)
    acc->x = Wire.read() << 8 | Wire.read(); //0x3B
    acc->y = Wire.read() << 8 | Wire.read(); //0x3D
    acc->z = Wire.read() << 8 | Wire.read(); //0x3F
    acc->x *= -1; // invert value

    // temperature not used
    temp = Wire.read() << 8 | Wire.read(); // 0x41

    gyro->x = Wire.read() << 8 | Wire.read(); // 0x43
    gyro->y = Wire.read() << 8 | Wire.read(); // 0x45
    gyro->z = Wire.read() << 8 | Wire.read(); // 0x47
    gyro->y *= -1; // invert value

    return true;
}

// setup MPU in a cascade
bool mpu_setup()
{
    serial_printlnF("setting up registers of MPU6050");

    serial_printF("mpu: ");
    uint8_t mputype = whoAmI();
    String mputype_string = "unknown";

    switch (mputype)
    {
    case 113:
        mputype_string = "MPU9250";
        break;
    case 104:
        mputype_string = "MPU6050";
        break;
    case 112:
        mputype_string = "MPU6500";
        break;
    }
    serial_printF("mpu: ");
    serial_print(mputype);
    serial_printF(" => ");
    serial_println(mputype_string);

    serial_printF("powerregister: ");
    serial_println(mpu_read8(0x6B));

    serial_printF("fifo: ");
    serial_println(mpu_read8(0x23));


    uint8_t c = mpu_read8(0x1B); // get current GYRO_CONFIG register value

#define  GFS_250DPS  0
    c = c & ~0x02; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | GFS_250DPS << 3; // Set full scale range for the gyro

    if (0 == mpu_write_pair(0x6B, 0x80))         // reset
        if (0 == mpu_write_pair(0x6B, 0x00))     // disable sleep
            if (0 == mpu_write_pair(0x6B, 0x01)) // auto clock source to be PLL gyrospcove
                if (0 == mpu_write_pair(0x37, 0x22))
                    if (0 == mpu_write_pair(0x38, 0x01))                         // Enable data ready (bit 0) interrupt
                        if (0 == mpu_write_pair(0x1A, 0x03))                     // disable FSYNC, set gyro to 41-42 Hz
                            if (0 == mpu_write_pair(0x19, 0x04))                 // set samplerate to 200hz
                                if (0 == mpu_write_pair(0x37, 0x02))             // bypass enable
                                    if (0 == mpu_write_pair(0x1B, c))            // write gyroscope scale
                                        if (0 == mpu_write_pair(0x1C, 0x03))     // configure the accelerometer (+/-8g)
                                            if (0 == mpu_write_pair(0x1B, 0x08)) // configure the gyro (500dps full scale)
                                                return true;

    return false;
  
}

void mpu_test()
{
#ifdef unused
    for (;;)
    {
        xyzFloat a = IMU.getAngles();
        serial_printF("a.x: ");
        serial_print(a.x);
        serial_printF(" a.y: ");
        serial_print(a.y);
        serial_printF(" a.z: ");
        serial_print(a.z);
        serial_printlnF("");

        xyzFloat g = IMU.getGValues();
        serial_printF("g.x: ");
        serial_print(g.x);
        serial_printF(" g.y: ");
        serial_print(g.y);
        serial_printF(" g.z: ");
        serial_print(g.z);
        serial_printlnF("");

        delay(1000);
    }
#endif
}

bool mpu_calibrate(gyroStruct *acc, gyroStruct *gyro, gyroStruct *cal)
{
    serial_printlnF("calculating acc offset");
    for (int cal_int = 0; cal_int < PWM_MAX; cal_int++)
    {
        if (cal_int % 125 == 0)
        {
            // serial_printF(".");
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }

        if (!mpu_read_data(acc, gyro))
            return false;

        cal->x += gyro->x;
        cal->y += gyro->y;
        cal->z += gyro->z;
    
        delay(3);
    }

        serial_printF("cal->x: ");
        serial_print(cal->x);
        serial_printF(" cal->y: ");
        serial_print(cal->y);
        serial_printF(" cal->z: ");
        serial_print(cal->z);
        serial_printlnF("");

    cal->x /= PWM_MAX;
    cal->y /= PWM_MAX;
    cal->z /= PWM_MAX;

    return true;
}

#endif // MPU_H
