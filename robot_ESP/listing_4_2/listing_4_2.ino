//Подключаем библиотеку для работы с Bluetooth
#include <BluetoothSerial.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
//Создаем Bluetooth порт
BluetoothSerial SerialBT;
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
  //Открываем Bluetooth порт.
  SerialBT.begin("Hacker1"); //Bluetooth имя робота.
  Serial.println("Do");
}
//++++++++++++++++++++++++++++++++++++++
int32_t statisticO;
//int32_t masL[4000];

//int32_t masR[4000];
int i=0;
void loop()
{
  //Проверяем наличие команд от смартфона
  if (SerialBT.available())
  {
    //Читаем из буфера очередной символ
    char bt_input = (char)SerialBT.read();
    // обрабатываем принятую команду
    move_case(bt_input);
    Serial.println(ESP.getFreeHeap());
    //if(i=1000) i=0;
//masL[i]=ESP.getFreeHeap();
//masR[i]=ESP.getFreeHeap();
  //  i++;
  }
  else // Если нет связи со смартфоном, отключить драйвера
    if (flagTimeOff < millis()) motor_off();
    
}
