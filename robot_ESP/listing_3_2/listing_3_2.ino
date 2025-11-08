//#define EN 2 // GPIO ESP32 На пин драйвера включение - удержание.
//#define DIR 16 // GPIO ESP32 На пин драйвера - направление вращения.
//#define STEP 4 // GPIO ESP32 На пин драйвера - управления шагами.

#define EN 13 // GPIO ESP32 На пин драйвера включение - удержание.
#define DIR 2 // GPIO ESP32 На пин драйвера - направление вращения.
#define STEP 15 // GPIO ESP32 На пин драйвера - управления шагами.

int i = 0; //Текущий номер шага.
int64_t MAXi = 6000; //Количество шагов.
int64_t min_steplong = 200; // Длительность после разгона шага в мик.сек.
int64_t max_steplong = 3000; // Длительность начального шага в мик.сек.
int64_t msec_steplong;
int64_t start_time;
int8_t stag;
double acceleration = 10000; //Ускорение в шагах на секунду^2.
double accelerationT; //Текущее ускорение
double speed0; //Стартовая скорость.
double dt;//Приращение времени от начала ускорения/торможения.
int64_t timer;
int64_t steps_to_stop; //Количество шагов требуемое для торможения.
int64_t mediana; //Середина цикла
void setup()
{
  pinMode(EN, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  digitalWrite(EN, 1); // Пока двигатель отключен.
  digitalWrite(STEP, 0);
  digitalWrite(DIR, 1);
  msec_steplong = max_steplong;
  accelerationT = acceleration;
  speed0 = 1000000.0 / double(max_steplong);
  stag = 5;
  Serial.begin(115200);
  Serial.println("Boot");
  Serial.println("Wait command from UART");
}
void loop()
{
  if (stag == 5) //Двигатель не работает
  {
    if (Serial.available()) //Если что-то есть на входе
    {
      Serial.end();//На время движения отк.UART
      digitalWrite(EN, 0); // Двигатель включен.
      delay(100); //Время на включение драйвера.
      stag = 1;
      steps_to_stop = 0;
      speed0 = 1000000.0 / double(max_steplong);
      start_time = micros();
      i = 0;
      mediana = MAXi / 2;
      timer = micros(); //Текущее время.
    }
  }
  long timeT = micros(); //Текущее время.
  if ((timeT >= timer) && (stag != 5))
  {
    i++;
    switch (stag) {
      case 1:  //Разгон
        //Считаем количество шагов на разгон/торможение.
        steps_to_stop++;
        digitalWrite(STEP, 1); //Поднимаем шаг
        dt = double(timeT - start_time) * 0.000001; //Переводим в секунды
        msec_steplong = long(1000000.0 / (speed0 + acceleration * dt));
        // msec_steplong = max_steplong - acceleration * dt;
        if (msec_steplong <= min_steplong)
        {
          msec_steplong = min_steplong; //Ограничиваем скорость.
          stag = 2; // Переходим в фазу поступательного движения
          //Изменяю значение стартовой скорости.
          speed0 = 1000000.0 / msec_steplong;
        }
        else
          //Если не достигли макс.скорости, но уже половина пути...
          if (steps_to_stop >= mediana)
          {
            stag = 3; // Переходим в фазу торможения.
            //Изменяю значение стартовой скорости.
            speed0 = 1000000.0 / msec_steplong;
            start_time = timeT;
          }
        delayMicroseconds(10); // Держим сигнал шага поднятым 10 мик.сек.
        digitalWrite(STEP, 0);
        break;
      case 2: // Поступатльное движение - без ускорения.
        digitalWrite(STEP, 1); //Поднимаем шаг
        delayMicroseconds(10); // Держим сигнал шага поднятым 10 мик.сек.
        digitalWrite(STEP, 0);
        //Если оставшиеся шаги равны тормозному пути....
        if ((MAXi - i) <= steps_to_stop)
        {
          stag = 3;
          start_time = timeT;
        }
        break;
      case 3: // Торможение.
        digitalWrite(STEP, 1); //Поднимаем шаг
        dt = double(timeT - start_time) * 0.000001; //Переводим в секунды
        msec_steplong = long(1000000.0 / (speed0 - acceleration * dt));
        // Если рассчитанный шаг больше, чем достаточный для остановки.
        if (msec_steplong >= max_steplong)
        {
          msec_steplong = max_steplong; //Ограничиваем скорость.
        }
        delayMicroseconds(10); // Держим сигнал шага поднятым 10 мик.сек.
        digitalWrite(STEP, 0);
        if (i >= MAXi) //Если все шаги сделаны.
        {
          stag = 4; // Остановка.
          // Удерживаем вал в течении данного времени.
          msec_steplong = 500000;
        }
        break;
      case 4: // Отключение мотора.
        digitalWrite(EN, 1);
        stag = 5;
        Serial.begin(115200);
        break;
    }
    //Здесь хранится время начала следующего шага.
    timer += msec_steplong;
  }
}
