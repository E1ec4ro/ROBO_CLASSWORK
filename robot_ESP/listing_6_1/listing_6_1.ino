// Подключаем внутренние программные файлы.
#include "motorstep.h"  //Описание моторов
#include "irq_robot.h"  //Работа с прерываниями
#include "parallelsonar.h"
//=====================================
//+++++++++++++++++++++++++++++++++++++
//=====================================
void setup() {
  Sonar_init();
  //Инициализируем моторы
  setup_motor_system();
  //Выключаем моторы.
  motor_on();
  // Инициализируем генерацию шагов моторов
  timer_setup();
  //Открываем Serial порт
  Serial.begin(115200);
}
//++++++++++++++++++++++++++++++++++++++
//Дистанция сканирования
const int32_t SCAN_MAX_DIST = 40;
//Дистанция останова - разворота
const int32_t stop_DIST = 15;
//Коэффициент поворота
const  int32_t Krot1 = maxSPEED / 10;
//Коэффициент движения вперед
const  int32_t Movation1 = maxSPEED / 2; // / 40;
const  int32_t half_scan_res = SCAN_MAX_DIST / 2;
int32_t RighPower, LeftPower, Rotation1;

void loop()
{

  Sonar(SCAN_MAX_DIST);
  if (long_sm[2].long_sm > stop_DIST && long_sm[1].long_sm > stop_DIST)
  {
    Rotation1 = -(long_sm[3].long_sm - half_scan_res) * Krot1;
    RighPower = Rotation1 + Movation1;
    LeftPower = -Rotation1 + Movation1;
  }
  else
  {
    RighPower = maxSPEED / 2;
    LeftPower = -maxSPEED / 2;
  }
  set_new_speed(RighPower, LeftPower);

  delay(50);
}
//=============================================

void set_new_speed(int32_t s_r, int32_t s_l)
{
  speed_right = (s_r, -maxSPEED, maxSPEED);
  speed_left = (s_l, -maxSPEED, maxSPEED);
  // если требуемая скорость не равна с текущей.
  if (speed_last_L != speed_left)
  {
    LEFT_STEPS_counter  = 2;
  }
  // если требуемая скорость не равна с текущей.
  if (speed_last_R != speed_right)
  {
    RIGHT_STEPS_counter  = 2;
  }
}
