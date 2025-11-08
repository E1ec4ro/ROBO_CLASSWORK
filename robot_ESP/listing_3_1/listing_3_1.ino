#define EN 2 // GPIO ESP32 На пин драйвера включение - удержание.
#define DIR 16 // GPIO ESP32 На пин драйвера - направление вращения.
#define STEP 4 // GPIO ESP32 На пин драйвера - управления шагами.

int i = 0; //Текущий номер шага.
int MAXi = 600; //Количество шагов.
int microsec_steplong = 3000; // Длительность шага в мик.сек.
long timer;

void setup() {
  pinMode(EN, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  digitalWrite(EN, 1); // Пока двигатель отключен.
  digitalWrite(STEP, 0);
  digitalWrite(DIR, 1);
  timer = micros() + microsec_steplong;
  digitalWrite(EN, 0); // Двигатель включен.
  Serial.println("BEGIN");
}
void loop()
{
  long timeT;
  timeT = micros(); //Текущее время.
  if (timeT >= timer)
  {
    if (i < MAXi)
    {
      timer += microsec_steplong; //Здесь хранится время начала следующего шага.
      digitalWrite(STEP, 1);
      delayMicroseconds(10); // Держим сигнал шага поднятым 10 мик.сек.
      digitalWrite(STEP, 0);
      i++;
    }
    else if (i == MAXi)
    {
      Serial.println("motor stoped");
      delay(500); //Гасим инерцию движения.
      digitalWrite(EN, 1); // Отключаем мотор.
      Serial.println("END");
      i++;
    }
  }
}
