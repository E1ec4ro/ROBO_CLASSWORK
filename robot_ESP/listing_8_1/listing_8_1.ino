#include "gyro_acsel.h"

//Переменные управления интервалами опроса гироприбора и интервалами управления
unsigned long dt;
unsigned long t0;
unsigned long t2;

double AcYsum, GyXsum0;

double OldGyro = 0;
double OldAcYsum = 0;
double AcYsumOld = 0;

int32_t t_period;

//double   _1_d_131 = 1.0 * DEG_TO_RAD / 131.0;
double   _1_d_131 = 1.0  / 131.0;
double Acsel = 0;
double Gyro = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();
  giroscop_setup();
  delay(100);
  Calc_CompensatorZ(2000);
  t0 = micros() - 5000;
  t2 = micros();
}

void loop()
{
  t_period = 5000;
  static int i = 0; // для торможения вывода
  uint32_t micros_;
  micros_ = micros();
  //Если период < t2 (прошло 5000 если да, проиводим очередной расчет)
  if (micros_ < t2) return;
   dt = micros() - t0; // Длительность предыдущего периода регулирования.
  t0 += dt; //Точка начала нового периода регулирования.
  //переводим в секунды время от предыдущего опроса.
  double Dt = double(dt) * 0.000001;
  //Опрос гироприбора:
  Data_mpu6050();
  //Расчет угла по показаниям акселерометра
  //с учетом поворота прибора 
  double Atan = atan2(AcX, AcZ);
  if(Atan>-PI/2.0)
  Acsel = (Atan-PI/2.0) * RAD_TO_DEG;
  else Acsel = (PI*3.0/2.0+Atan) * RAD_TO_DEG;
  // скорость угловая - падения
  Gyro = - (float(GyY) - CompensatorY)  * _1_d_131;
  //Комплементарный фильтр
  AcYsum = ONE_ALFA * (OldAcYsum + Gyro * Dt) + ALFA * Acsel;
  GyXsum0 =GyXsum0+Gyro * Dt; 
  OldAcYsum = AcYsum;
  t2 = t0 + t_period;
 if (i > 100)
  {
    i = 0;
   Serial.print(GyXsum0); 
    Serial.print(" ");
    Serial.print(Acsel); 
    Serial.print(" ");
    Serial.println(AcYsum);
  }
  i++;
}
