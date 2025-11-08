#include "defin.h"
#include "gyro_acsel.h"
#include "motorstep.h"  //Описание моторов
#include "irq_robot.h"  //Работа с прерываниями

void setup() {
  //Инициализируем моторы
  setup_motor_system();
  //Выключаем моторы.
  motor_off();
  // Инициализируем генерацию шагов моторов
  timer_setup();
  Serial.begin(115200);
  giroscop_setup();
  delay(100);
  Calc_CompensatorZ(4000);
  t0 = micros() - 5000;
  t2 = micros();
  Speed = 0;
}

uint32_t micros_;
void loop()
{
  int32_t speed_L ;
  int32_t speed_R;
  t_period = 5000;
  static int i = 0; // для торможения вывода
  micros_ = micros();
  if (micros_ < t2) return;//Если период < t2 анализ. прошло 5000 если да, измеряем угловую скорость.
  //Опрос гироприбора:
  Data_mpu6050();
  dt = micros() - t0; // Длительность предыдущего периода регулирования.
  t0 += dt; //Точка начала нового периода регулирования.
  double Dt = double(dt) * 0.000001;
  // Нужно поймать угол, рядом с которым угловая скорость колеблется в районе нуля.
  //Расчет угла по показаниям акселерометра
  Acsel = (atan2(AcX, AcZ)) - PI / 2.0; // * RAD_TO_DEG;
  // скорость угловая В радианах - падения
  Gyro = - (double(GyY) - CompensatorY)  * _1_d_131;

  //Комплементарный фильтр
  AcYsum = ONE_ALFA * (AcYsum + Gyro * Dt) + ALFA * Acsel;
  t2 = t0 + t_period;

  Ka = 1200.0 / ( PI );
  Kg = 300.0 / ( PI);
  //1
  Speed = AcYsum  * Ka;
  //2
  //Speed = Gyro  * Kg;
  //3
  //Speed = AcYsum  * Ka + Gyro  * Kg;
  //4
  //Speed = AcYsum  * Ka + Gyro  * Kg + XSpeed;

  Speed *= 400.0;

  speed_L = (Speed);
  speed_R = (Speed);
  speed_L = constrain(speed_L, -maxSPEED, maxSPEED);
  speed_R = constrain(speed_R, -maxSPEED, maxSPEED);
  //Speed - это число шагов за 100 секунд
  SetSpeed(speed_L, speed_R);
  //Возвращаем обработанное значение скорости в первоначальную формулу
  XSpeed = (speed_L + speed_R) / 800.0;
}
