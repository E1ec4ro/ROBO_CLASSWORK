// Библиотека для работы с протоколом I2C (порты A5/SCL и A4/SDA)
#include <Wire.h>

#define ALFA 0.01
#define ONE_ALFA 0.99//1.0-1.0E-3

// упрощеный I2C адрес нашего гироскопа/акселерометра MPU-6050.
const int MPU_addr = 0x68;
// переменные для хранения данных возвращаемых прибором.
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double CompensatorZ, CompensatorX, CompensatorY,  CompensatorAcX;
unsigned long timer = 0;
double OldAcYsum = 0;
double AcYsumOld = 0;
double AcYsum=0;
double   _1_d_131 = 1.0 * DEG_TO_RAD / 131.0;
double Acsel = 0;
double Gyro = 0;
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
  Wire.begin();//(22,23); //SDA //SCL

  // BNO055 clock stretches for 500us or more!
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
   uint32_t micros_;
  float i = 0;
  CompensatorZ = 0;
  CompensatorX = 0;
  CompensatorY = 0;
  timer = millis();
  unsigned long endtime = millis() + ms;
  t0 = micros();
  while (endtime > timer) {
    timer = millis();
    Data_mpu6050();
   
  
  //Опрос гироприбора:
  //bno.getEvent(&event); // Берем сразу углы Эйлера
  dt = micros() - t0; // Длительность предыдущего периода регулирования.
  t0 += dt; //Точка начала нового периода регулирования.
  double Dt = double(dt) * 0.000001;
  // Нужно поймать угол, рядом с которым угловая скорость колеблется в районе нуля.
  OldAcYsum = AcYsum;
  //Data_mpu6050();
  //Расчет угла по показаниям акселерометра
   Acsel = (atan2(AcX, AcZ))-PI/2.0;// * RAD_TO_DEG;
   // скорость угловая В радианах - падения
   Gyro = - (float(GyY))  * _1_d_131; 
   //Комплементарный фильтр
  AcYsum = ONE_ALFA * (OldAcYsum + Gyro * Dt) + ALFA * Acsel;
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
