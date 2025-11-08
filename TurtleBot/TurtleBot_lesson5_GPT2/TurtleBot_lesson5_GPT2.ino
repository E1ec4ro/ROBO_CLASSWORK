#include <DynamixelWorkbench.h>

// Конфигурация
#define BAUDRATE 1000000
#define ACCELERATION 50
#define DXL_ID1 1
#define DXL_ID2 2

// Геометрия робота
double L = 0.155; // Расстояние между колесами
double R = 0.05;  // Радиус колеса

// ID моторов
uint8_t id1 = DXL_ID1;
uint8_t id2 = DXL_ID2;

// Динамические переменные
double goalWheelVel[2] = {0, 0};
double realWheelVel[2] = {0, 0};
double goalRobotVel[2] = {0, 0}; // [линейная, угловая]
double realRobotVel[4] = {0, 0, 0, 0}; // [текущая лин., текущая угл., предыдущая лин., предыдущая угл.]
double odometry[2] = {0, 0}; // [пройденный путь, угол]

// Позиция робота
double X = 0, Y = 0, A = 0;

// Состояния автомата
enum State {
  FORWARD,
  TURN_90_DEG
};
State state = FORWARD;

// Временные метки
unsigned long millRequest = 0;
unsigned long millSerial = 0;

DynamixelWorkbench dx;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Инициализация драйвера
  dx.init("", BAUDRATE);

  // Проверка подключения
  dx.ping(id1);
  dx.ping(id2);

  // Установка режима "колесо"
  dx.wheelMode(id1, ACCELERATION);
  dx.wheelMode(id2, ACCELERATION);

  // Сброс временных меток
  millRequest = millis();
  millSerial = millis();

  Serial.println("Робот готов к движению.");
}

void loop() {
  switch (state) {
    case FORWARD:
      if (odometry[0] < 0.2) { // <-- Сторона квадрата: теперь 20 см вместо 50 см
        goalRobotVel[0] = 0.1; // Линейная скорость
        goalRobotVel[1] = 0.0; // Угловая скорость
      } else {
        state = TURN_90_DEG;
        Serial.println("Перехожу к повороту на 90 градусов");
        odometry[0] = 0; // Сбрасываем пройденный путь
      }
      break;

    case TURN_90_DEG:
      if (abs(odometry[1]) < 1.57) { // ~90 градусов в радианах
        goalRobotVel[0] = 0.0;
        goalRobotVel[1] = -0.4; // Угловая скорость против часовой
      } else {
        state = FORWARD;
        Serial.println("Перехожу к следующей стороне квадрата");
        odometry[1] = 0; // Сбрасываем угол
      }
      break;
  }

  // Вычисление целевой скорости колес
  setGoalRobotVel(goalRobotVel, goalWheelVel);

  // Управление моторами
  turtleMove(id1, goalWheelVel[0]);
  turtleMove(id2, goalWheelVel[1]);

  // Обновление реальных скоростей и одометрии каждые 100 мс
  if (millis() - millRequest > 100) {
    realWheelVel[0] = getRealWheelVel(id1);
    realWheelVel[1] = getRealWheelVel(id2);
    calcOdometry(realWheelVel, realRobotVel, odometry);
    millRequest = millis();
  }

  // Отладочный вывод раз в 500 мс
  if (millis() - millSerial > 500) {
    serialPrint(realWheelVel, realRobotVel, odometry);
    millSerial = millis();
  }
}

bool turtleMove(uint8_t id, float vel) {
  return dx.goalVelocity(id, vel);
}

double getRealWheelVel(uint8_t id) {
  int32_t tempVel = 0;
  dx.itemRead(id, "Present_Velocity", &tempVel);
  double vel = tempVel * 0.0247; // Коэффициент зависит от модели сервы
  return vel;
}

void setGoalRobotVel(double *goalRobotVel, double *goalWheelVel) {
  goalWheelVel[0] = (goalRobotVel[0] - (L / 2) * goalRobotVel[1]) / R;
  goalWheelVel[1] = (goalRobotVel[0] + (L / 2) * goalRobotVel[1]) / R;
}

void calcOdometry(double *realWheelVel, double *realRobotVel, double *odometry) {
  unsigned long currentMillis = millis();
  static unsigned long prevMillis = 0;
  double dt = (currentMillis - prevMillis) / 1000.0;
  prevMillis = currentMillis;

  // Текущие значения
  realRobotVel[0] = (R * (realWheelVel[0] + realWheelVel[1])) / 2;
  realRobotVel[1] = (R * (realWheelVel[1] - realWheelVel[0])) / L;

  // Усреднение значений
  double delta_s = ((realRobotVel[0] + realRobotVel[2]) / 2) * dt;
  double delta_a = ((realRobotVel[1] + realRobotVel[3]) / 2) * dt;

  // Обновление позиции
  odometry[0] += delta_s;
  odometry[1] += delta_a;

  A += delta_a;
  X += delta_s * cos(A);
  Y += delta_s * sin(A);

  // Сохраняем предыдущие значения
  realRobotVel[2] = realRobotVel[0];
  realRobotVel[3] = realRobotVel[1];
}

void serialPrint(double *realWheelVel, double *realRobotVel, double *odometry) {
  Serial.print("Состояние: ");
  switch (state) {
    case FORWARD: Serial.println("Движение вперёд"); break;
    case TURN_90_DEG: Serial.println("Поворот на 90 градусов"); break;
  }

  Serial.print("Левое колесо: "); Serial.println(realWheelVel[0]);
  Serial.print("Правое колесо: "); Serial.println(realWheelVel[1]);

  Serial.print("Пройдено: "); Serial.println(odometry[0]);
  Serial.print("Угол (рад): "); Serial.println(odometry[1]);
  Serial.print("Угол (град): "); Serial.println(odometry[1] * 57.3);

  Serial.print("X: "); Serial.println(X);
  Serial.print("Y: "); Serial.println(Y);
  Serial.println("------------------------");
}