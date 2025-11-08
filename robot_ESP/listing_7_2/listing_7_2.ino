//Подключаем библиотеку для создания дополнительных Serial портов
#include <HardwareSerial.h>
//Создаем объект порт-Serial1
HardwareSerial Serial_1(1);

// Подключаем внутренние программные файлы.
#include "motorstep.h"  //Описание моторов
#include "irq_robot.h"  //Работа с прерываниями
#include "move_case.h"  //Реагирование на команды
//=====================================
//+++++++++++++++++++++++++++++++++++++
//=====================================
void setup() {
  //Инициализируем моторы
  setup_motor_system();
  //Выключаем моторы.
  motor_off();
  // Инициализируем генерацию шагов моторов
  timer_setup();
  //Открываем Serial порт
  Serial.begin(115200);
  //Открываем Serial1 порт.
  Serial_1.begin(115200, SERIAL_8N1, 14, 19);//RX-14 , TX -19
  Serial.println("Do");
}
//++++++++++++++++++++++++++++++++++++++
int32_t statisticO;

int i = 0;
void loop()
{
  //Проверяем наличие команд от смартфона
  if (Serial_1.available())
  {
    //Читаем из буфера очередной символ
    char bt_input = (char)Serial_1.read();
    if (com_calc == 0) if (bt_input == 'C') {
        com_calc++;
        return;
      } else return;
    if (com_calc == 1)
      if (bt_input == 'F' || bt_input == 'T' || bt_input == 'P')
      {
        com_calc++;
        COMMAND_ser = bt_input;
        return;
      } else {
        com_calc = 0;
        return;
      }
    if (com_calc == 2) {
      com_calc = 0;
    }
    // обрабатываем принятую команду
    move_case(bt_input, COMMAND_ser);
  }
  else // Если нет новых команд, остановиться, отключить мотор
  {
    if (flagTimeStop < millis())
    {
      char bt_input = 0;
      COMMAND_ser = 'F';
      move_case(bt_input, COMMAND_ser);
    }
    if (flagTimeOff < millis()) motor_off();
  }
}
