#include <BluetoothSerial.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

#include "defin.h"
#include "gyro_acsel.h"
#include "motorstep.h"  //Описание моторов
#include "irq_robot.h"  //Работа с прерываниями
#include "move_case.h"  //Реагирование на команды
#include "servo_hand.h"  //Подъемный рычаг

void setup() {
  //Инициализация рычага
  servo_hand_setup();

  //Инициализируем моторы
  setup_motor_system();
  //Выключаем моторы.
  motor_off();
  // Инициализируем генерацию шагов моторов
  timer_setup();

  Serial.begin(115200);
  Serial.println();

  SerialBT.begin("BHV_M"); //Имя робота в BT
  giroscop_setup();
  delay(100);
  Calc_CompensatorZ(4000);
  t0 = micros() - 5000;
  t2 = micros();
  Speed = 0;
  flag_crash = true;
  Serial.print("GYRO="); Serial.print(Gyro);
  Serial.print("    Acsel="); Serial.print(Acsel);
  Serial.print("    AcYsum ="); Serial.print(AcYsum);
  Serial.println("=================================================");
  time_start_move = millis();
  time_stop_move = millis();
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
  BT_input();
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

  if ( flag_crash)
  {

    if ((time_stop_move - 3000) < millis())
    {
      //запускаем подъемный рычаг
      servo_hand_bottom(AcYsum * RAD_TO_DEG);
    }
    if ((time_stop_move - 1000) < millis())
    {
      if (abs(AcYsum * RAD_TO_DEG) < 40) //Если поднят
      {
        motor_on();
      }
      if (time_stop_move  < millis())
      {
        if (abs(AcYsum * RAD_TO_DEG) < 40) //Если поднят
        {
          flag_crash = false;
          counter_stepR = 0;
          counter_stepL = 0;
        }
        else {
          //Move = MoveLimit * 2;
        }
      }
    }
    counter_stepR = 0;
     counter_stepL = 0;
    return;
  }
  // Если вошли в критичный режим - , робот упал, но из-за ошибки
  // гироскопа показал достаточное отклонение.
  if (abs(Move) > MoveLimit) //Если вошли в критичный режим)
  {
    flag_crash = true;
    XSpeed = 0;
    Speed = 0;
    OldCommandSpeed = 0;
    CommandSpeed = 0;
    speed_L = 0;
    speed_R = 0;
    counter_stepR = 0;
    counter_stepL = 0;
    speed_xxx_L = 0;
    newSpeedflag_L  = true;
    speed_xxx_R = 0;
    newSpeedflag_R  = true;
    dGyro = 0;
    Move = 0;
    dMove = 0;
    ddMove = 0;
    dMoveOld = 0;
    motor_off();
    Turn = 0;
    time_stop_move = millis() + 6000; //Секунда на стабилизацию
    GyroOld = 0;
    servo_hand_up(); 
    return;
  }
  if ((time_stop_move + 2000) < millis()) servo_hand_up(); 

  XL = counter_stepL; counter_stepL = 0;
  XR = counter_stepR; counter_stepR = 0;

  double dMoveX = double(XL + XR) * 0.5 / Dt;
  OldCommandSpeed_dMove = OldCommandSpeed;

  dMove = dMoveX - OldCommandSpeed_dMove;
  ddMove = ddMove * 0.9 + (dMove - dMoveOld) * 0.1 / Dt;
  dMoveOld = dMove;
  Move += (XL + XR) / 2.0 + Dt * ( - OldCommandSpeed); //Это как раз пройденный путь

  dGyro = 0.9 * dGyro + 0.1 * ((Gyro - GyroOld) / Dt);
  GyroOld = Gyro;

  Speed = (AcYsum)  * Ka + Gyro  * Kg + dGyro * Kdg + XSpeed + (dMove) * Kdm + Move * Km + ddMove * Kddm;
  OldCommandSpeed = CommandSpeed / 100.0;
  Speed *= 400.0;

  speed_L = (Speed + Turn);
  speed_R = (Speed - Turn);
  speed_L = constrain(speed_L, -maxSPEED, maxSPEED);
  speed_R = constrain(speed_R, -maxSPEED, maxSPEED);

  //Speed - это число шагов за 100 секунд
  SetSpeed(speed_L, speed_R);
  //Возвращаем обработанное значение скорости в первоначальную формулу
  XSpeed = (speed_L + speed_R) / 800.0;
}
