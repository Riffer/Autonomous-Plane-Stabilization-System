/*
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;


// cosas de Timers
float dt;
long tiempo_prev;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;


float pitch_prev = 0;
float roll_prev = 0;

double pitch_accel = 0;
double roll_accel = 0;

void setup() {
  // serial to display data
  Serial.begin(9600);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  //diferencal de tiempo para su uso en el calculo de rotacion (pitch/ang Y, y roll/angX)
  /*calculo del dt en segundos*/ 
  dt = (millis()-tiempo_prev)/1000.0;
  /*tomo nota de tiempo_prev para siguiente calculo del dt*/
  tiempo_prev = millis();
  
  // read the sensor
  IMU.readSensor();
  
  
  /*
  Serial.write("Accelerometer in the X direction: "); //escribo la cadena "Accelerometer in the X direction: "
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write("Accelerometer in the Y direction: "); //escribo la cadena "Accelerometer in the Y direction: "
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write("Accelerometer in the Z direction: "); //escribo la cadena "Accelerometer in the Z direction: "
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write('\n'); //salto de linea y retorno de carro
  
  Serial.write("Gyroscope in the X direction: "); //escribo la cadena "Gyroscope in the X direction: "
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write("Gyroscope in the Y direction: "); //escribo la cadena "Gyroscope in the Y direction: "
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write("Gyroscope in the Z direction: "); //escribo la cadena "Gyroscope in the Z direction: "
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write('\n'); //salto de linea y retorno de carro
  
  Serial.write("Magnetometer in the X direction: "); //escribo la cadena "Magnetometer in the X direction: "
  Serial.print(IMU.getMagX_uT(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write("Magnetometer in the Y direction: "); //escribo la cadena "Magnetometer in the Y direction: "
  Serial.print(IMU.getMagY_uT(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write("Magnetometer in the Z direction: "); //escribo la cadena "Magnetometer in the Z direction: "
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write('\n'); //salto de linea y retorno de carro
  
  Serial.write("Temeperatura: "); //escribo la cadena "Temeperatura: "
  Serial.println(IMU.getTemperature_C(),6);
  Serial.write('\n'); //salto de linea y retorno de carro
  Serial.write('\n'); //salto de linea y retorno de carro
  */
 
  //Roll & Pitch Equations con acelerometro ojo: double  atan2 (double __y, double __x)  ---> arc tangente of y/x
  //puede ser atan normal pero atan2 es por que esta en progreso que mida de -180 a 180 no solo de -90 a 90
  pitch_accel = atan2((IMU.getAccelY_mss()), sqrt(pow((IMU.getAccelX_mss()), 2) + pow((IMU.getAccelZ_mss()), 2)))*(180.0/3.14);
  roll_accel = atan2(-(IMU.getAccelX_mss()), sqrt(((IMU.getAccelY_mss()) * (IMU.getAccelY_mss())) + ((IMU.getAccelZ_mss()) * (IMU.getAccelZ_mss()))))*(180.0/3.14);
  
  float y_acc_nvo = atan2 ((IMU.getAccelY_mss()),(IMU.getAccelZ_mss()))*(180.0/3.14);

  //Calcular angulo de rotación (pitch/angulo en Y, y roll/angulo en X) con giroscopio y filtro complemento  
  pitch = 0.98*(pitch_prev+((IMU.getGyroX_rads())/131)*dt) + 0.02*pitch_accel;
  roll  = 0.98*(roll_prev+((IMU.getGyroY_rads())/131)*dt) + 0.02*roll_accel;
  
  float y_nvo = 0.98*(pitch_prev+((IMU.getGyroX_rads())/131)*dt) + 0.02*y_acc_nvo;
  float y_nvo_prev = y_nvo;
  
  pitch_prev = pitch;
  roll_prev = roll;
  
  //Yaw from mag
  float Yh = ((IMU.getMagY_uT()) * cos(roll)) - ((IMU.getMagZ_uT()) * sin(roll));
  float Xh = ((IMU.getMagX_uT()) * cos(pitch))+((IMU.getMagY_uT()) * sin(roll)*sin(pitch)) + ((IMU.getMagZ_uT()) * cos(roll) * sin(pitch));
  yaw = atan2(Yh, Xh)*(180.0/3.14);

  /*
  //para comunicar con monitor serie arduino
  Serial.print(roll);
  Serial.write('\t'); //tab
  Serial.print(pitch);
  Serial.write('\t'); //tab
  Serial.print(yaw);
  Serial.write('\n'); //salto de linea y retorno de carro
  */
  /*
  //para comunicar con labview
  Serial.write('A'); //inicio de trama
  Serial.print(roll);
  Serial.write(','); //caracter entre datos ","
  Serial.print(pitch);
  Serial.write(','); //caracter entre datos ","
  Serial.print(yaw);
  Serial.write('B'); //final de trama
  */
  
  //rolls comparando monitor serie
  Serial.write("roll viejo: ");
  Serial.print(roll);
  Serial.write('\t'); //tab
  Serial.write("roll/ptch?: ");
  Serial.print(y_nvo);
  Serial.write('\t'); //tab
  Serial.write("roll/ptch? sin filtro: ");
  Serial.print(y_acc_nvo);
  Serial.write('\t'); //tab
  Serial.write("pitch viejo sin filtro: ");
  Serial.print(pitch_accel);
  Serial.write('\t'); //tab
  Serial.write("pitch viejo: ");
  Serial.print(pitch);
  Serial.write('\n'); //salto de linea y retorno de carro
  
  


}
