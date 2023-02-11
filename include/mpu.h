#ifndef MPU_PLANE
#define MPU_PLANE

#include <Wire.h>

#define MPU9250
#if defined(MPU9250)
#include <MPU9250_WE.h>
#endif
#define MPU_ADDR 0x68

namespace motion
{

    MPU9250_WE IMU = MPU9250_WE(MPU_ADDR);

    bool mpu_6050_read_data(gyroStruct *acc, gyroStruct *val)
    {
        xyzFloat gVAccalue = IMU.getGValues();
        acc->x = gVAccalue.x;
        acc->x *= -1; // invert value
        acc->y = gVAccalue.y;
        acc->z = gVAccalue.z;

        xyzFloat gyroValues = IMU.getGyrValues();
        val->x = gyroValues.x;
        val->y = gyroValues.y;
        val->y *= -1; // invert value
        val->z = gyroValues.z;

        serial_printF(" X: ");
        serial_print(acc->x);
        serial_printF(" Y: ");
        serial_print(acc->y);
        serial_printF(" Z: ");
        serial_print(acc->z);
        serial_printF(" V: ");
        serial_print(acc->totalVector);

        serial_printF(" X: ");
        serial_print(val->x);
        serial_printF(" Y: ");
        serial_print(val->y);
        serial_printF(" Z: ");
        serial_print(val->z);

        serial_printlnF("");

        return true;
    }

    // setup MPU in a cascade
    bool mpu_6050_setup()
    {
        Wire.begin();

        if (!IMU.init())
        {
            return false;
        }

        IMU.autoOffsets();
        IMU.getAccOffset();

        IMU.enableGyrDLPF();
        IMU.setGyrDLPF(MPU6500_DLPF_6);
        IMU.setSampleRateDivider(5);
        IMU.setGyrRange(MPU6500_GYRO_RANGE_1000);
        IMU.setAccRange(MPU6500_ACC_RANGE_8G);
        IMU.enableAccDLPF(true);
        IMU.setAccDLPF(MPU6500_DLPF_6);
        return true;
    }
}
#endif // MPU_PLANE
