//Подключаем дополнительную библиотеку для работы с энергонезависимой памятью
#include <nvs_flash.h>
//Подключаем основную библиотеку для работы с энергонезависимой памятью
#include <Preferences.h>
//Подключаем библиотеку для работы с адресными светодиодами
#include <Adafruit_NeoPixel.h>
//Адресный светодиод GPIO
#define NeoPixelPin 5
//Количество светодиодов
#define NUMPIXELS   1
//Создание объекта управления светодиодами
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NeoPixelPin, NEO_GRB + NEO_KHZ800);

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

Preferences preferences;
//=====================================
//+++++++++++++++++++++++++++++++++++++
//=====================================
void setup() {
  // Старт NeoPixel
  pixels.begin(); // This initializes the NeoPixel library.
  // Яркий белый свет
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  //Передать на светодиод
  pixels.show();

  //Инициализируем моторы
  setup_motor_system();
  //Выключаем моторы.
  motor_off();
  // Инициализируем генерацию шагов моторов
  timer_setup();
  //Открываем Serial порт
  Serial.begin(115200);
  Serial.println("Do");
}
//++++++++++++++++++++++++++++++++++++++
int32_t statisticO;
int i = 0;

#define BUTTON_GPIO 32

const int delta = 50;//Разброс
const int button_Do = 149;////49; //Нажато воспроизведение
const int button_Write = 1810; //Нажата запись маршрута

void loop()
{
  if (operating_mode == NO_MODE)
  {
    int button = analogRead(BUTTON_GPIO);
    // Если нажата кнопка воспроизведения пути.
    if ((button > (button_Do - delta)) && (button < (button_Do + delta)))
    {
      timerAlarmDisable(Timer);
      if (read_from_flash())
      {
        operating_mode = READ_MODE;
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show();
        motor_on();
        timer_counter = 0;
      }
      else
      {
        //Чтение не удалось
       red_blink();
      }
      timerAlarmEnable(Timer);
      //      delay(50);
    }
    else
      // Если нажата кнопка записи пути.
      if ((button > (button_Write - delta)) && (button < (button_Write + delta)) )
      {
        //Открываем Bluetooth порт.
        SerialBT.begin("Hacker1"); //Bluetooth имя робота.
        operating_mode = WRITE_MODE;
        //Диод светит голубым
        pixels.setPixelColor(0, pixels.Color(0, 0, 255));
        pixels.show();
      }
      else return;
  } else if (operating_mode == WRITE_MODE)
  {
    //Проверяем наличие команд от смартфона
    if (SerialBT.available())
    {
      //Читаем из буфера очередной символ
      char bt_input = (char)SerialBT.read();
      // обрабатываем принятую команду
      move_case(bt_input);
    }
    // Если нет связи со смартфоном или простаивает робот - отключить драйвера
    if (flagTimeOff < millis())
    {
      if (!STEPER_EN_level)
      {
        timerAlarmDisable(Timer);
        if (write_to_flash())
        {
          motor_off();
          gren_blue_blink();
        }
        else
        {
          timerAlarmDisable(Timer);
          red_blink();
        }
        timerAlarmEnable(Timer);
      }
    }
  }
  else if (operating_mode == READ_MODE)
  {
    if ((write_counter_Rmax <= write_counter_R) && (write_counter_Lmax <= write_counter_L))
    {
      motor_off();
      pixels.setPixelColor(0, pixels.Color(255, 255, 255));
      pixels.show();
      Serial.print(write_counter_Lmax); Serial.println(write_counter_L);
      Serial.print(write_counter_Rmax); Serial.println(write_counter_R);
      delay(1000);
    }
  }
}
//===========================================================
//===========================================================
bool write_to_flash()
{
  //Очищаем хранилище
  //nvs_flash_erase();
  preferences.begin("robo_steps", false);
  preferences.clear();
  //  Serial.println(preferences.freeEntries());
  if (preferences.putBytes("ar_t_c_left", array_timer_counter_left, sizeof(array_timer_counter_left)) == 0 ) return false;
  //  Serial.println(preferences.freeEntries());
  if (preferences.putBytes("ar_speed_left", array_speed_left, sizeof(array_speed_left)) == 0 ) return false;
  //  Serial.println(preferences.freeEntries());
  if (preferences.putBytes("ar_t_c_right", array_timer_counter_right, sizeof(array_timer_counter_right)) == 0 ) return false;
  //  Serial.println(preferences.freeEntries());
  if (preferences.putBytes("ar_speed_right", array_speed_right, sizeof(array_speed_right)) == 0 ) return false;
  //  Serial.println(preferences.freeEntries());
  preferences.putUShort("write_counter_L", write_counter_L);
  //  Serial.println(preferences.freeEntries());
  preferences.putUShort("write_counter_R", write_counter_R);
  //  Serial.println(preferences.freeEntries());
  preferences.end();
  return true;
}
//===========================================================
//===========================================================
bool read_from_flash()
{
  //Открываем хранилище на чтение
  preferences.begin("robo_steps", false);
  if (preferences.getBytes("ar_t_c_left", array_timer_counter_left, sizeof(array_timer_counter_left)) == 0 ) return false;
  if (preferences.getBytes("ar_speed_left", array_speed_left, sizeof(array_speed_left)) == 0 ) return false;
  if (preferences.getBytes("ar_t_c_right", array_timer_counter_right, sizeof(array_timer_counter_right)) == 0 ) return false;
  if (preferences.getBytes("ar_speed_right", array_speed_right, sizeof(array_speed_right)) == 0 ) return false;
  write_counter_Lmax = preferences.getUShort("write_counter_L");
  write_counter_Rmax = preferences.getUShort("write_counter_R");
  //  Serial.println(write_counter_Lmax);
  //  Serial.println(write_counter_Rmax);
  preferences.end();
  //  for (int i = 0; i < max_i; i++)
  //  {
  //    Serial.print("Lc="); Serial.print(array_timer_counter_left[i]); Serial.print("  LS="); Serial.print(array_speed_left[i]);
  //    Serial.print("  Rc="); Serial.print(array_timer_counter_right[i]); Serial.print("  RS="); Serial.println(array_speed_right[i]);
  //  }
  return true;
}
//===========================================================
//===========================================================
void red_blink()
{
  for (int i = 0; i < 5; i++)
  {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    delay(100);
  }
}
//===========================================================
//===========================================================

void gren_blue_blink()
{
  for (int i = 0; i < 5; i++)
  {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
    delay(100);
  }
}
