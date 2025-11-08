#define STEP_L 15 //Пин левого шага 
#define STEP_R 4 //Пин правого шага
#define STEPER_EN 2 //Пин врключения моторов
// Функция инициализации уравления моторами.//
#define DIR_L 13 //Пин направления левого шага 
#define DIR_R 16 //Пин направления правого шага
bool STEP_L_level = false;
bool STEP_R_level = false;
bool DIR_L_level = false;
bool DIR_R_level = false;
bool STEPER_EN_level = true;

// Ссылка на таймер.
hw_timer_t * timer = NULL;

//Функция обработки прерывания по таймеру.
void IRAM_ATTR onTimer()
{
  STEP_L_level = !STEP_L_level;
  digitalWrite(STEP_L, STEP_L_level);
  STEP_R_level = !STEP_R_level;
  digitalWrite(STEP_R, STEP_R_level);
}

void setup() {
  pinMode(STEP_L, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(STEPER_EN, OUTPUT);
  // Начальное значение уровня сигнала
  // на шаговых контактах.
  STEP_L_level = false;
  STEP_R_level = false;
  //Задаем направление вращения.
  DIR_L_level = false;
  DIR_R_level = false;
  //Активируем моторы.
  STEPER_EN_level = false;

  digitalWrite(STEP_L, STEP_L_level);
  digitalWrite(STEP_R, STEP_R_level);
  digitalWrite(DIR_L, DIR_L_level);
  digitalWrite(DIR_R, DIR_R_level);
  digitalWrite(STEPER_EN, STEPER_EN_level);

  Serial.begin(115200);
  // Создаем таймер, с предделителем 80,
  // что для нашего контроллера дает отсчеет 1 раз в микросекунду.
  timer = timerBegin(0, 80, true);
  // Прикрепляем функцию onTimer в качестве обработчика прерывания.
  timerAttachInterrupt(timer, &onTimer, true);
  // Для таймера timer создаем "будильник"
  // срабатывающий через 100000 отчетов.
  // последний перамерт true - периодичное срабатывание.
  // false - разовое срабатывание.
  timerAlarmWrite(timer, 100000, true);
  // Активируем будильник.
  timerAlarmEnable(timer);
}

void loop() {
  //Ничего не делаем.
  delay(1000);
}
