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
      if (data_fromUART())
      {
        //Serial.end();//На время движения отк.UART
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
        break;
    }
    //Здесь хранится время начала следующего шага.
    timer += msec_steplong;
  }
}

char chisloM[10], chisloS[10], chisloA[10];
int indexS = 0, indexA = 0, indexM = 0;
char inpDATA;
bool end_input = false;
bool data_fromUART()
{
  /*Примеры ввода данных
  S1000M5000A1000E
  M5000A1000E
  M-5000A1000E
  E
  */
  //sDATAaDATAdData
  while (Serial.available()) //Принимаю число
  {
    char b = Serial.read();
    //Начало ввода параметра
    //S - скорость.
    //М - перемещение, кол.шагов.
    //A - ускорение
    if ((b == 'S') || (b == 'M') || (b == 'A'))
    {
      inpDATA = b;
      continue;
    }
    else if (b == 'E') //Окончание ввода
    {
      end_input = true;
      break;
    }
    if (((b == '-')) || (b == '1')
        || (b == '2') || (b == '3') || (b == '4')
        || (b == '5') || (b == '6') || (b == '7')
        || (b == '8') || (b == '9') || (b == '0'))
    {
      switch (inpDATA) {
        case 'S':
          if (indexS < 8)
          {
            chisloS[indexS] =  b;
            indexS++;
          }
          break;
        case 'M':
          if (indexM < 8)
          {
            chisloM[indexM] =  b;
            indexM++;
          }
          break;
        case 'A':
          if (indexA < 8)
          {
            chisloA[indexA] =  b;
            indexA++;
          }
          break;
      }
    }
  }
  if (end_input)
  {
    if (indexM > 0)
    {
      MAXi = atol(&chisloM[0]); //Из строки в число
      for (indexM = 0; indexM < 8; indexM++)
        chisloM[indexM] = '\0';
      indexM = 0;
      Serial.print("MAXi="); Serial.println(int(MAXi));
      if (MAXi < 0) {
        MAXi = -MAXi;
        digitalWrite(DIR, 0);
      }
      else digitalWrite(DIR, 1);
    }
    if (indexS > 0)
    {
      double Speed = double(abs(atol(&chisloS[0]))); //Из строки в число
      for (indexS = 0; indexS < 8; indexS++)
        chisloS[indexS] = '\0';
      indexS = 0;
      Serial.print("Speed="); Serial.println(Speed);
      min_steplong = int64_t(1000000.0 / Speed);
    }
    if (indexA > 0)
    {
      acceleration = double(abs(atol(&chisloA[0]))); //Из строки в число
      for (indexA = 0; indexA < 8; indexA++)
        chisloA[indexA] = '\0';
      indexA = 0;
      Serial.print("Acceleration="); Serial.println(acceleration);
    }
    end_input = false;
    return true; // Ввод осуществлен
  }
  return false; //Ввода не было
}
