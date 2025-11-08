//Создаем указатель на таймер
//генерации шагов для моторов робота
hw_timer_t * Timer = NULL;

// Объявляем переменные, хранящие текущее
// состояние уровня сигнала на GPIO шагов для моторов
//=======================================
// Функция, вызываемая по прерыванию –
// генерирует сигнал шага для левого мотора
// генерирует сигнал шага для правого мотора

volatile bool newSpeedflag_R = false;
volatile bool newSpeedflag_L = false;
//+++++++++++++++++++++++++++++++++++++++
//Маска выключения высокого состояния шагов всех моторов
uint32_t off_all_step;
//Маски высокого состояния шагов для каждого мотора
uint32_t LEFTSTEP_mask_on;
uint32_t RIGHTSTEP_mask_on;
//Количество шагов, которые нужно сделать
volatile int   LEFT_STEPS_counter = 0;
volatile int   RIGHT_STEPS_counter = 0;
//================================================
//================================================
uint32_t sost;
//Скорость для расчета длительности следующего шага
volatile int32_t speed0Ltec = 0;
volatile int32_t speed0Rtec = 0;
//Скорость начала расчета
int32_t speed0L = 0;
int32_t speed0R = 0;
//Направление шагов
int32_t DIR_L_tec = DIR_L_FORWARD;
int32_t DIR_R_tec = DIR_R_FORWARD;
// Новое время - длительность шага в 10микСек
int32_t newLEFTcounter_do_step = 100;
int32_t newRIGHTcounter_do_step = 100;
//Установленное время - длительность шага в 10микСек
int32_t  LEFTcounter_do_step = 100;
int32_t  RIGHTcounter_do_step = 100;
//Пройденное время в 10х мк.сек
int32_t dt_L = 0;
int32_t dt_R = 0;
//Середина шага
int32_t half_LEFT_STEPS = 50;
int32_t half_RIGHT_STEPS = 50;
int32_t newhalf_LEFT_STEPS = 50;
int32_t newhalf_RIGHT_STEPS = 50;

//Прошедшее время текущего шага в 10микСек
int32_t LEFTcounter = 0;
int32_t RIGHTcounter = 0;
bool old_sost = false;

int32_t LIM_maxL = 0;
int32_t LIM_minL = 0;

int32_t LIM_maxR = 0;
int32_t LIM_minR = 0;

//++++++++++++++++++++++++++++++++++++++++//
//==== ==//
//++++++++++++++++++++++++++++++++++++++++//
void IRAM_ATTR step_timer() {
  if (newSpeedflag_L)
  {
    speed_left = speed_xxx_L;
    if (speed_last_L != speed_left)
    {
      // = Если пришла новая скорость и шагов нет, то переходим в режим изменения шагов
      if (LEFT_STEPS_counter == 0)
      {
        LEFTcounter = 0;
        half_LEFT_STEPS = 0;
      }
    }
    newSpeedflag_L = false;
  }

  if (newSpeedflag_R)
  {
    speed_right = speed_xxx_R;
    if (speed_last_R != speed_right)
    {
      // = Если пришла новая скорость и шагов нет, то переходим в режим изменения шагов
      if (RIGHT_STEPS_counter == 0)
      {
        RIGHTcounter = 0;
        half_RIGHT_STEPS = 0;
      }
    }
    newSpeedflag_R = false;
  }

  
  //Опустить шаг для всех моторов
  // Заменяем множество опускающих команд на
  sost = 0;
  if (LEFT_STEPS_counter > 0) //Если есть шаги
  {
    if (LEFTcounter >= LEFTcounter_do_step)
    {
      sost |= LEFTSTEP_mask_on; //Поднять шаг -  пока только флаг
      //Счетчик длительности шага - обнуляем для начала нового шага
      LEFTcounter = 0;
      LEFT_STEPS_counter--; //Счетчик количества шагов – уменьшаем
      if(DIR_L_tec == DIR_L_FORWARD) counter_stepL ++; else counter_stepL --;
      // Принимаем следующую длительность шага
      LEFTcounter_do_step = newLEFTcounter_do_step;
      // Устанавливаем новую середину шага
      half_LEFT_STEPS = newhalf_LEFT_STEPS;
    }
    LEFTcounter++; //Счетчик длительности шага
  }
  if (RIGHT_STEPS_counter > 0 )
  {
    if (RIGHTcounter >= RIGHTcounter_do_step)
    {
      sost |= RIGHTSTEP_mask_on; //Поднять шаг -  пока только флаг
      //Счетчик длительности шага - обнуляем для начала нового шага
      RIGHTcounter = 0;
      RIGHT_STEPS_counter--;//Счетчик шагов – уменьшаем
           if(DIR_R_tec == DIR_R_FORWARD) counter_stepR ++; else counter_stepR --;
      // Принимаем следующую длительность шага
      RIGHTcounter_do_step = newRIGHTcounter_do_step;
      // Устанавливаем новую середину шага
      half_RIGHT_STEPS = newhalf_RIGHT_STEPS;
    }
    RIGHTcounter++; //Счетчик длительности шага
  }
  //Если шаг нужно делать
  //Опускаем сигналы шагов (пока только в переменной)
  //Поднимаем шаги для моторов, которым это нужно
  //Записываем значение с измененными битами шагов в регистр GPIO (0-31)
  if (sost)
  { old_sost = true;
    REG_WRITE(GPIO_OUT_REG, ((REG_READ(GPIO_OUT_REG)& off_all_step) | sost));
  }
  // Если шаги делать не нужно, но нужно опустить
  // поднятые при прошлом вызове импульсы шагов
  else if (old_sost) {
    old_sost = false;
    // Опускаем все значения на шагов на 0
    REG_WRITE(GPIO_OUT_REG, (REG_READ(GPIO_OUT_REG)& off_all_step));
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //================================================================//
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //Шаг сделан рассчитываем параметры следующего шага
  if (LEFTcounter == half_LEFT_STEPS) // если мы на середине шага
  {
    // acceleration ускорение в относительных единицах (шагов/сек^2)/1000
    //dt_L - количество 10ов мк.сек с начала действия ускорения.
    // speed0L скорость с начала действия данного ускорения
    //speed0L - шагов за 100 секунд
    // Если пришла новая скорость

    if (speed_last_L != speed_left)
    {
      //Выбираем, какое направление ускорения требуется
      // для достижения новой скорости
      if (speed0Ltec < speed_left)
      {
        accelerationL = acceleration;
        LIM_maxL = speed_left;
        LIM_minL = -maxSPEED;
      }
      else
      {
        accelerationL = - acceleration;
        LIM_maxL = maxSPEED;
        LIM_minL =  speed_left;
      }
      speed_last_L = speed_left;
      //Начинается новый участок - время обнуляется.
      dt_L = 0;
      // Новая базовая скорость задана.
      speed0L = speed0Ltec;
    }
    if(accelerationL!=0)
    {
    // Изменяется значение времени от начала отчета в 10-х мк.Сек
    dt_L += LEFTcounter_do_step;
    //Расчет скорости для следующего шага
    speed0Ltec = speed0L  + accelerationL * dt_L;
    // Проверка вхождения в пределы и корректировка
    speed0Ltec = constrain(speed0Ltec, LIM_minL, LIM_maxL);
    if(speed0Ltec == speed_last_L) accelerationL =0;
    }
    // Если полученная скорость больше остановочной
    int32_t abs_speed = abs(speed0Ltec);
    if (abs_speed > minSPEED)
    {
      //Находим длительность следующего шага
      newLEFTcounter_do_step = 10000000 / abs_speed;
      // Добавляем 2 в счетчик шагов, иначе не будет новых шагов
      LEFT_STEPS_counter = 2;
    }
    else
    {
      // Ограничиваем длительность шага
      newLEFTcounter_do_step = 1000; //или 1000 мк.сек
      //Если скорость стремилась к 0
      //Не делаем больше шагов
      if (speed_last_L  == 0) {
        speed0L = 0;
        speed0Ltec = 0;
        accelerationL = 0;
        LEFT_STEPS_counter = 0;
      }
      else LEFT_STEPS_counter = 2;
    }
    //Анализируем знак скорости
    //при необходимости изменяем направление
    //вращения мотора
    if (speed0Ltec < 0) {
      if (DIR_L_tec == DIR_L_FORWARD)
      {
        digitalWrite(DIR_L, DIR_L_BACKWARD);
        DIR_L_tec = DIR_L_BACKWARD;
      }
    }
    else {
      if (DIR_L_tec == DIR_L_BACKWARD)
      {
        digitalWrite(DIR_L, DIR_L_FORWARD);
        DIR_L_tec = DIR_L_FORWARD;
      }
    }
    //Находим середину длительности нового шага
    //Для скорости деление заменяем на сдвиг
    newhalf_LEFT_STEPS = newLEFTcounter_do_step >> 1;
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //================================================================//
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  
  // если мы на середине шага
  if (RIGHTcounter == half_RIGHT_STEPS) // если мы на середине шага
  {
    // acceleration ускорение в относительных единицах (шагов/сек^2)/1000
    //dt_R - количество 10ов мк.сек с начала действия ускорения.
    // speed0R скорость с начала действия данного ускорения
    //speed0R - шагов за 100 секунд
    // Если пришла новая скорость

    if (speed_last_R != speed_right)
    {
      //Выбираем, какое направление ускорения требуется
      // для достижения новой скорости
      if (speed0Rtec < speed_right)
      {
        accelerationR = acceleration;
        LIM_maxR = speed_right;
        LIM_minR = -maxSPEED;
      }
      else
      {
        accelerationR = - acceleration;
        LIM_maxR = maxSPEED;
        LIM_minR =  speed_right;
      }
      speed_last_R = speed_right;
      //Начинается новый участок - время обнуляется.
      dt_R = 0;
      // Новая базовая скорость задана.
      speed0R = speed0Rtec;
    }
    if(accelerationR !=0)
    {
    // Изменяется значение времени от начала отчета в 10-х мк.Сек
    dt_R += RIGHTcounter_do_step;
    //Расчет скорости для следующего шага
    speed0Rtec = speed0R  + accelerationR * dt_R;
    // Проверка вхождения в пределы и корректировка
    speed0Rtec = constrain(speed0Rtec, LIM_minR, LIM_maxR);
   if(speed0Rtec == speed_last_R) accelerationR =0;
   }

    // Если полученная скорость больше остановочной
    int32_t abs_speed = abs(speed0Rtec);
    if (abs_speed > minSPEED)
    {
      //Находим длительность следующего шага
      newRIGHTcounter_do_step = 10000000 / abs_speed;
      // Добавляем 2 в счетчик шагов, иначе не будет новых шагов
      RIGHT_STEPS_counter = 2;
    }
    else
    {
      // Ограничиваем длительность шага
      newRIGHTcounter_do_step = 1000; //или 1000 мк.сек
      //Если скорость стремилась к 0
      //Не делаем больше шагов
      if (speed_last_R  == 0) {
        speed0R = 0;
        speed0Rtec = 0;
        accelerationR = 0;
        RIGHT_STEPS_counter = 0;
      }
      else RIGHT_STEPS_counter = 2;
    }
    //Анализируем знак скорости
    //при необходимости изменяем направление
    //вращения мотора
    if (speed0Rtec < 0) {
      if (DIR_R_tec == DIR_R_FORWARD)
      {
        digitalWrite(DIR_R, DIR_R_BACKWARD);
        DIR_R_tec = DIR_R_BACKWARD;
      }
    }
    else {
      if (DIR_R_tec == DIR_R_BACKWARD)
      {
        digitalWrite(DIR_R, DIR_R_FORWARD);
        DIR_R_tec = DIR_R_FORWARD;
      }
    }
    //Находим середину длительности нового шага
    //Для скорости деление заменяем на сдвиг
    newhalf_RIGHT_STEPS = newRIGHTcounter_do_step >> 1;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++//
//+++++++++++++++++++++++++++++++++++++++++++++++//
//Готовим состояния масок
// Создание таймеров, прикрепление к ним
// функций обработчиков прерываний
//+++++++++++++++++++++++++++++++++++++++++++++++//
void timer_setup()
{
  off_all_step = 0xFFFFFFFF; //Все биты в 1
  //Маски подъема сигналов на GPIO шагов моторов
  LEFTSTEP_mask_on = uint32_t(1) << STEP_L;
  RIGHTSTEP_mask_on = uint32_t(1) << STEP_R;

  off_all_step = off_all_step ^ LEFTSTEP_mask_on;
  //Теперь имеем маску, которой можно убрать
  //STEP сигналы сразу с обоих моторов
  off_all_step = off_all_step ^ RIGHTSTEP_mask_on;
  digitalWrite(STEP_L, false);//STEP_L_level);
  digitalWrite(STEP_R, false);//STEP_R_level);
  // Создаем таймер
  Timer = timerBegin(0, 80, true);
  // Прикрепляем функциию обработчики
  timerAttachInterrupt(Timer, &step_timer, true);
  //Прерывание повторяется каждые 10 микросекунд
  timerAlarmWrite(Timer, 10, true);
  //Старт таймера, теперь прерывания будут генерироваться
  timerAlarmEnable(Timer);
}


void SetSpeed(int32_t speed_L,int32_t speed_R)
{
  motor_on();
    // если требуемая скорость не равна с текущей.
  speed_xxx_L = constrain(speed_L, -maxSPEED, maxSPEED);
  if (abs(speed_xxx_L) < minSPEED) speed_xxx_L = 0;
  newSpeedflag_L  = true;

  speed_xxx_R = constrain(speed_R, -maxSPEED, maxSPEED);
  if (abs(speed_xxx_R) < minSPEED) speed_xxx_R = 0;
  newSpeedflag_R  = true;
}
