//Создаем указатель на таймер
//генерации шагов для моторов робота
hw_timer_t * sonar_timer_t = NULL;

#define FLAG_SONAR_START1 1
#define FLAG_SONAR_START2 2
#define FLAG_SONAR_REGread 3
#define FLAG_SONAR_STOP 4

//Номер GPIO сонара.
#define SONAR_GPIO 14
#define SONAR_TRIG  27
struct sruct_sonar_data
{
  uint32_t start_time = 0;
  uint32_t stop_time = 0;
  double long_sm = 0;
  uint32_t GPIO;
  bool startFlag = false;
  bool stopFlag = false;
};

volatile double dist_sm;
sruct_sonar_data long_sm;
uint32_t time_, duration;

//= Измерение расстояния ============================
uint32_t registr_in;
uint32_t Lim;
bool all_close;
int iteracion;
uint32_t Limit = 40;
int flag_sonar;
int sonar_delay_time;

void IRAM_ATTR Sonar()
{
  if (flag_sonar == FLAG_SONAR_START1)
  {
    long_sm.start_time = 0;
    long_sm.stop_time = 0;
    //long_sm.long_sm = Limit;
    long_sm.startFlag = false;
    long_sm.stopFlag = false;
    // Переводим лимит из сантиметров в относительные величины (микросек.)
    Lim = (Limit * 588)/10;
    //Генерируем импульс
    digitalWrite(SONAR_TRIG, HIGH);
    flag_sonar = FLAG_SONAR_START2;
  }
  else if (flag_sonar == FLAG_SONAR_START2)
  {
    digitalWrite(SONAR_TRIG, LOW);
    //Ждем отражения импульса.
    // Пересчитываем время в расстояние (по скорости звуку).
    time_ = 0;
    duration = Lim;
    long_sm.stop_time = duration;
    iteracion = 10;
    all_close = false;
    flag_sonar = FLAG_SONAR_REGread;
  }
  else if (flag_sonar == FLAG_SONAR_REGread)
  {
    if ((time_ < duration) && (all_close == false))
    {
     
      registr_in = REG_READ(GPIO_IN_REG);
      if (long_sm.stopFlag == false)
      {
        if (long_sm.startFlag == false)
        {
          if ((registr_in & long_sm.GPIO) != 0) {
            long_sm.start_time = time_;
            long_sm.startFlag = true;
          }
        }
        else
        {
           time_ += 10;
          if ((registr_in & long_sm.GPIO) == 0)
          {
            long_sm.stop_time = time_;
            long_sm.stopFlag = true;
            all_close = true;

          }
        }
      }
    }
    else
    {
      flag_sonar = FLAG_SONAR_STOP;
      long_sm.long_sm = double(long_sm.stop_time) * 0.017;
      sonar_delay_time = 0;
    }
  }
  else if (flag_sonar == FLAG_SONAR_STOP)
  {
    if (sonar_delay_time < 8000) //100 миллисекунд пауза
      sonar_delay_time++;
    else
      flag_sonar = FLAG_SONAR_START1;
  }
  dist_sm = long_sm.long_sm;
}

void sonar_setup()
{
  //=Режим пинов/портов ===============================
  long_sm.GPIO = 1 << SONAR_GPIO;
  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(long_sm.GPIO, INPUT);
  // Создаем таймер
  sonar_timer_t = timerBegin(1, 80, true);
  // Прикрепляем функциию обработчики
  timerAttachInterrupt(sonar_timer_t, &Sonar, true);
  //Прерывание повторяется каждые 10 микросекунд
  timerAlarmWrite(sonar_timer_t, 10, true);
  //Старт таймера, теперь прерывания будут генерироваться
  timerAlarmEnable(sonar_timer_t);
  flag_sonar = FLAG_SONAR_START1;
}
