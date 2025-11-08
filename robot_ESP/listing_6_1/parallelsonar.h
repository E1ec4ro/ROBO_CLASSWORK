//Количество используемых сонаров
#define NUMBER_SONARS 4

//Номера GPIO сонаров.
#define SONAR_TRIG  27
#define SONAR_RIGHT 17
#define SONAR_RIGHT_FRONT 18
#define SONAR_LEFT_FRONT 25
#define SONAR_LEFT 26

struct sruct_sonar_data
{
  uint32_t start_time = 0;
  uint32_t stop_time = 0;
  double long_sm = 0;
  uint32_t GPIO;
  bool startFlag = false;
  bool stopFlag = false;
};

sruct_sonar_data long_sm[NUMBER_SONARS];
uint32_t time_, duration;

//=Режим пинов/портов ===============================
void Sonar_init()
{
  // Этот участок подразумевает заплнений полей с номерами GPIO по колчеству сонаров
  // При изменении количества соннаров его требуется поправить
  long_sm[0].GPIO = 1 << SONAR_LEFT;
  long_sm[1].GPIO = 1 << SONAR_LEFT_FRONT;
  long_sm[2].GPIO = 1 << SONAR_RIGHT_FRONT;
  long_sm[3].GPIO = 1 << SONAR_RIGHT;
  // конец участка который нужно править вручную при изменении числа сонаров
  
  //Задаем режимы работы пинов.
  pinMode(SONAR_TRIG, OUTPUT);
  for (int i = 0; i < NUMBER_SONARS; i++)
    pinMode(long_sm[i].GPIO, INPUT);
}
//= Измерение расстояния ============================
double Sonar(uint32_t Limit = 40)
{
  uint32_t registr_in;
  for (int i = 0; i < NUMBER_SONARS; i++)
  {
    long_sm[i].start_time = 0;
    long_sm[i].stop_time = 0;
    long_sm[i].long_sm = Limit;
    long_sm[i].startFlag = false;
    long_sm[i].stopFlag = false;
  }
  // Переводим лимит из сантиметров в относительные величины (микросек.)
  uint32_t Lim = Limit * 58.8;
  //Генерируем импульс
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);
  //Ждем отражения импульса.
  // Пересчитываем время в расстояние (по скорости звуку).
  int all_close = 0;
  time_ = micros();
  duration = time_ + Lim;
  int iteracion = 0;
  while ((time_ < duration) && (all_close < NUMBER_SONARS))
  {
    time_ = micros();
    registr_in = REG_READ(GPIO_IN_REG);
    for (int i = 0; i < NUMBER_SONARS; i++)
    {
      if (long_sm[i].stopFlag == false)
      {
        if (long_sm[i].startFlag == false)
        {
          if ((registr_in & long_sm[i].GPIO) != 0) {
            long_sm[i].start_time = time_;
            long_sm[i].startFlag = true;
          }
        }
        else
        {
          if ((registr_in & long_sm[i].GPIO) == 0)
          {
            long_sm[i].stop_time = time_;
            long_sm[i].stopFlag = true;
            all_close++;
          }
        }
      }
    }
    iteracion++;
  }
  //Serial.println(iteracion);
  //Если импульс нормально начался и завершился
  // в заданное время
  for (int i = 0; i < NUMBER_SONARS; i++)
    if (long_sm[i].stopFlag)
    {
      long_sm[i].long_sm = double(long_sm[i].stop_time - long_sm[i].start_time) * 0.017;
    }
}
