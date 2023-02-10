#ifndef MPU_6050
#define MPU_6050

#include <Wire.h>


#define I2C_ERROR_MSG "I2C error"

uint8_t mpu_6050_write_pair(int a, int b, bool waitAfterWriteMs=300)
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(a);
    Wire.write(b);
    uint8_t error = Wire.endTransmission();
    if (error)
    {
        //serial_printlnF(I2C_ERROR_MSG);
        return error;
    }
    delay(waitAfterWriteMs);
    return error;
}


int temp = 0;
bool mpu_6050_read_data(gyroStruct *acc, gyroStruct *val)
{
    uint8_t error;
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    error = Wire.endTransmission();

    if (error)
    {
        //serial_printlnF(I2C_ERROR_MSG);
        return false;
    }

    Wire.requestFrom(0x68, 14); // resets all 3 sensors

    while (Wire.available() < 14); // wait until buffer has 14 bytes

    // read 8 bit pairs while shifting first byte to left, building into a full 16 bit (long integer)
    acc->x = Wire.read() << 8 | Wire.read();
    acc->x *= -1; // invert value
    acc->y = Wire.read() << 8 | Wire.read();
    acc->z = Wire.read() << 8 | Wire.read();

    // temperature not used
    temp = Wire.read() << 8 | Wire.read();

    val->x = Wire.read() << 8 | Wire.read();
    val->y *= -1; // invert value
    val->y = Wire.read() << 8 | Wire.read();
    val->z = Wire.read() << 8 | Wire.read();

    return true;
}

// setup MPU in a cascade
bool mpu_6050_setup()
{
    serial_printlnF("setting up registers of MPU6050");

    if (0 == mpu_6050_write_pair(0x6B, 0x00))         // disable sleep
        if (0 == mpu_6050_write_pair(0x1C, 0x10))     // configure the accelerometer (+/-8g)
            if (0 == mpu_6050_write_pair(0x1B, 0x08)) // configure the gyro (500dps full scale)
                return true;

    return false;
}

bool mpu_6050_calibrate(gyroStruct *acc, gyroStruct *val, gyroStruct *cal)
{
    serial_printlnF("calculating acc offset");
    for (int cal_int = 0; cal_int < PWM_MAX; cal_int++)
    {
        if (cal_int % 125 == 0)
        {
            // serial_printF(".");
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }

        if (!mpu_6050_read_data(acc, val))
            return false;

        cal->x += val->x;
        cal->y += val->y;
        cal->z += val->z;
        delay(3);
    }
    cal->x /= PWM_MAX;
    cal->y /= PWM_MAX;
    cal->z /= PWM_MAX;

    return true;
}

#endif // MPU_6050
