// Библиотека для работы с протоколом I2C (порты A5/SCL и A4/SDA)
#include <Wire.h>

#define ALFA 0.01
#define ONE_ALFA 0.99//

// упрощеный I2C адрес нашего гироскопа/акселерометра MPU-6050.
const int MPU_addr = 0x68;
// переменные для хранения данных возвращаемых прибором.
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double CompensatorZ, CompensatorX, CompensatorY,  CompensatorAcX;
unsigned long timer = 0;
/////////////////////////////////////////////////////////////////////////
//// Считывание данных с mpu6050
/////////////////////////////////////////////////////////////////////////
void Data_mpu6050()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  //Готовим для чтения регистры  с адреса 0x3B.
  Wire.endTransmission(false);
  // Запрос 14 регистров.
  Wire.requestFrom(MPU_addr, 14, true);
  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcX = Wire.read() << 8 | Wire.read();
  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcY = Wire.read() << 8 | Wire.read();
  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();
  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Tmp = Wire.read() << 8 | Wire.read();
  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyX = Wire.read() << 8 | Wire.read();
  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyY = Wire.read() << 8 | Wire.read();
  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();
}
/////////////////////////////////////////////////////////////////////////
/// Запуск гироскопа
/////////////////////////////////////////////////////////////////////////
void giroscop_setup()
{
  /* Enable I2C */
  Wire.begin();//22,23); //SDA //SCL
#ifdef ESP8266
  Wire.setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // Производим запись в регистр энергосбережения MPU-6050
  Wire.write(0);     // устанавливем его в ноль
  Wire.endTransmission(true);
  CompensatorZ = 0;
  CompensatorX = 0;
  CompensatorY = 0;
}
/////////////////////////////////////////////////////////////////////////
// Компенсация нуля гироскопа
void Calc_CompensatorZ(float mill_sec)
{
  long ms = mill_sec;
  float i = 0;
  CompensatorZ = 0;
  CompensatorX = 0;
  CompensatorY = 0;
  timer = millis();
  unsigned long endtime = millis() + ms;
  while (endtime > timer) {
    timer = millis();
    Data_mpu6050();
    CompensatorZ += (float)(GyZ);
    CompensatorX += (float)(GyX);
    CompensatorY += (float)(GyY);
    delay(2);
    i++;
  }
  CompensatorZ /= i;
  CompensatorX /= i;
  CompensatorY /= i;
}
