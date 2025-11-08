#include <Dynamixel2Arduino.h>  // подключение библиотеки Dynamixel

// Последовательный порт DXL для платы STEM с OpenCM 9.04.
#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial // последовательный порт, подключаемый к компьютеру

const uint8_t DXL_DIR_PIN = 22; // номер информационного пина сервоприводов
const float DXL_PROTOCOL_VERSION = 1.0; // протокол передачи данных от OpenCM9.04 к сервоприводам

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // инициализация указателя на команды из библиотеки Dynamixel

// Импортируем имена для доступа к таблице Управления сервоприводов
using namespace ControlTableItem;

#define jointN 5           // количество сервоприводов

// Параметры для преобразования градусов в единицы Dynamixel (0-1023 для 300°)
const float DEGREE_TO_UNIT = 1023.0 / 300.0; // ≈3.41 единиц на градус

// Функция для преобразования градусов в единицы Dynamixel
int degreesToUnits(float degrees) {
  return (int)(degrees * DEGREE_TO_UNIT);
}

// Начальные позиции для каждого сервопривода в градусах (согласно заданию)
float initialDegrees[jointN+1] = {0, 150.0, 48.0, 254.0, 150.0, 150.0};

// Преобразование начальных позиций в единицы Dynamixel
int initialPositions[jointN+1] = {0, 
  degreesToUnits(initialDegrees[1]),
  degreesToUnits(initialDegrees[2]),
  degreesToUnits(initialDegrees[3]),
  degreesToUnits(initialDegrees[4]),
  degreesToUnits(initialDegrees[5])
};

// Массив позиций для последовательности движений манипулятора
// Формат: {серво1, серво2, серво3, серво4, серво5}
// серво5 - схват (150° - открыт, 180° - закрыт)
int movementSequence[][6] = {
  // Начальная позиция
  {0, initialPositions[1], initialPositions[2], initialPositions[3], initialPositions[4], initialPositions[5]},
  
  // Плавное опускание манипулятора
  {0, degreesToUnits(140), degreesToUnits(60), degreesToUnits(240), degreesToUnits(140), degreesToUnits(150)},
  
  // Подход к предмету (еще ниже)
  {0, degreesToUnits(130), degreesToUnits(70), degreesToUnits(230), degreesToUnits(130), degreesToUnits(150)},
  
  // Схват предмета (серво5 закрывается)
  {0, degreesToUnits(130), degreesToUnits(70), degreesToUnits(230), degreesToUnits(130), degreesToUnits(180)},
  
  // Поднятие предмета (первая фаза)
  {0, degreesToUnits(140), degreesToUnits(60), degreesToUnits(240), degreesToUnits(140), degreesToUnits(180)},
  
  // Поднятие предмета (вторая фаза - выше)
  {0, degreesToUnits(160), degreesToUnits(50), degreesToUnits(250), degreesToUnits(160), degreesToUnits(180)},
  
  // Перемещение с предметом
  {0, degreesToUnits(170), degreesToUnits(45), degreesToUnits(255), degreesToUnits(170), degreesToUnits(180)},
  
  // Опускание предмета в целевое положение
  {0, degreesToUnits(140), degreesToUnits(60), degreesToUnits(240), degreesToUnits(140), degreesToUnits(180)},
  
  // Отпускание предмета (серво5 открывается)
  {0, degreesToUnits(140), degreesToUnits(60), degreesToUnits(240), degreesToUnits(140), degreesToUnits(150)},
  
  // Возврат в начальное положение
  {0, initialPositions[1], initialPositions[2], initialPositions[3], initialPositions[4], initialPositions[5]}
};

#define sequenceLength 10 // количество позиций в последовательности

void setup() {
  DEBUG_SERIAL.begin(57600); // установка скорости обмена данными по последовательному порту компьютера
  DEBUG_SERIAL.println("Инициализация манипулятора...");
  
  dxl.begin(1000000);        // установка скорости обмена данными по последовательному порту манипулятора
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);  // выбор протокола обмена данными
  
  // Инициализация сервоприводов
  for (int i=1; i<=jointN; i++) {
    dxl.torqueOff(i); // отключаем крутящий момент для настройки
    dxl.setOperatingMode(i, OP_POSITION); // установка режима работы сервопривода в качестве шарнира
    dxl.writeControlTableItem(PROFILE_VELOCITY, i, 80); // установка скорости движения
    dxl.torqueOn(i); // включаем крутящий момент
    
    // Установка в начальное положение
    dxl.setGoalPosition(i, initialPositions[i]);
    DEBUG_SERIAL.print("Сервопривод ");
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(" установлен в положение: ");
    DEBUG_SERIAL.print(initialDegrees[i]);
    DEBUG_SERIAL.println("°");
    delay(500);
  }
  
  DEBUG_SERIAL.println("Манипулятор готов к работе!");
  delay(2000); // задержка для перехода в начальное положение
}

void loop() {
  DEBUG_SERIAL.println("Начало цикла: подъем и перемещение предмета");
  
  // Выполнение последовательности движений
  for (int step = 0; step < sequenceLength; step++) {
    DEBUG_SERIAL.print("Шаг ");
    DEBUG_SERIAL.println(step);
    
    // Установка позиций для всех сервоприводов на текущем шаге
    for (int servo = 1; servo <= jointN; servo++) {
      dxl.setGoalPosition(servo, movementSequence[step][servo]);
    }
    
    // Задержка между шагами (регулируйте для изменения скорости)
    switch(step) {
      case 2: // перед схватом - небольшая пауза
      case 3: // схват - пауза для надежного захвата
        delay(2000);
        break;
      case 7: // перед отпусканием - пауза
      case 8: // отпускание - пауза
        delay(2000);
        break;
      default:
        delay(1500); // стандартная пауза
    }
  }
  
  DEBUG_SERIAL.println("Цикл завершен. Пауза перед следующим циклом...");
  delay(5000); // пауза перед следующим циклом
}