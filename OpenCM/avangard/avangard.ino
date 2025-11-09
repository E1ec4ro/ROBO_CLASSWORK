#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial

const uint8_t DXL_DIR_PIN = 22; 
const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

#define jointN 5

const float DEGREE_TO_UNIT = 1023.0 / 300.0;

int degreesToUnits(float degrees) {
  return (int)(degrees * DEGREE_TO_UNIT);
}

float initialDegrees[jointN+1] = {0, 150.0, 48.0, 254.0, 150.0, 150.0};

int initialPositions[jointN+1] = {0, 
  degreesToUnits(initialDegrees[1]),
  degreesToUnits(initialDegrees[2]),
  degreesToUnits(initialDegrees[3]),
  degreesToUnits(initialDegrees[4]),
  degreesToUnits(initialDegrees[5])
};

int keyPositions[][6] = {
  {0, initialPositions[1], initialPositions[2], initialPositions[3], initialPositions[4], initialPositions[5]},
  
  {0, degreesToUnits(140), degreesToUnits(105), degreesToUnits(300), initialPositions[4], degreesToUnits(150)},
  
  {0, degreesToUnits(130), degreesToUnits(125), degreesToUnits(310), initialPositions[4], degreesToUnits(150)},
  
  {0, degreesToUnits(130), degreesToUnits(125), degreesToUnits(310), initialPositions[4], degreesToUnits(195)},
  
  {0, degreesToUnits(140), degreesToUnits(95), degreesToUnits(290), initialPositions[4], degreesToUnits(195)},
  
  {0, degreesToUnits(160), degreesToUnits(80), degreesToUnits(270), initialPositions[4], degreesToUnits(195)},
  
  {0, degreesToUnits(170), degreesToUnits(70), degreesToUnits(260), initialPositions[4], degreesToUnits(195)},
  
  {0, degreesToUnits(140), degreesToUnits(105), degreesToUnits(300), initialPositions[4], degreesToUnits(195)},
  
  {0, degreesToUnits(140), degreesToUnits(105), degreesToUnits(300), initialPositions[4], degreesToUnits(150)},
  
  {0, initialPositions[1], initialPositions[2], initialPositions[3], initialPositions[4], initialPositions[5]}
};

#define keyPositionsCount 10

const int INTERMEDIATE_STEPS = 35;
const int STEP_DELAY = 40;
const int MOVEMENT_SPEED = 100;
const int GRIPPER_SPEED = 150;

int currentPositions[jointN+1] = {0, 0, 0, 0, 0, 0};

void smoothMove(int startPositions[], int targetPositions[], int steps, int stepDelay) {
  for (int step = 1; step <= steps; step++) {
    float t = (float)step / steps;
    
    for (int servo = 1; servo <= jointN; servo++) {
      if (servo == 4) {
        dxl.setGoalPosition(servo, initialPositions[4]);
        currentPositions[servo] = initialPositions[4];
        continue;
      }
      
      int currentPosition = startPositions[servo] + (int)((targetPositions[servo] - startPositions[servo]) * t);
      dxl.setGoalPosition(servo, currentPosition);
      currentPositions[servo] = currentPosition;
    }
    
    delay(stepDelay);
  }
  
  for (int servo = 1; servo <= jointN; servo++) {
    if (servo == 4) {
      dxl.setGoalPosition(servo, initialPositions[4]);
      currentPositions[servo] = initialPositions[4];
    } else {
      dxl.setGoalPosition(servo, targetPositions[servo]);
      currentPositions[servo] = targetPositions[servo];
    }
  }
  delay(30);
}

void smoothGoToHome() {
  DEBUG_SERIAL.println("Плавный переход в начальное положение");
  
  int currentServoPositions[jointN+1] = {0};
  for (int servo = 1; servo <= jointN; servo++) {
    if (servo == 4) {
      currentServoPositions[servo] = initialPositions[4];
    } else {
      currentServoPositions[servo] = dxl.getPresentPosition(servo);
      DEBUG_SERIAL.print("Серво ");
      DEBUG_SERIAL.print(servo);
      DEBUG_SERIAL.print("текущая");
      DEBUG_SERIAL.print(currentServoPositions[servo]);
      DEBUG_SERIAL.print("целева");
      DEBUG_SERIAL.println(initialPositions[servo]);
    }
  }
  
  smoothMove(currentServoPositions, initialPositions, INTERMEDIATE_STEPS * 2, STEP_DELAY * 2);
  
  DEBUG_SERIAL.println("Все сервоприводы в начальном положении");
  
  for (int servo = 1; servo <= jointN; servo++) {
    int actualPosition = dxl.getPresentPosition(servo);
    DEBUG_SERIAL.print("Серво ");
    DEBUG_SERIAL.print(servo);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.print(actualPosition);
    DEBUG_SERIAL.print(" ед. (");
    DEBUG_SERIAL.print(initialDegrees[servo]);
    DEBUG_SERIAL.println("°)");
  }
  
  delay(1000);
}

void fastGripperMove(int targetPosition) {
  dxl.writeControlTableItem(PROFILE_VELOCITY, 5, GRIPPER_SPEED);
  dxl.setGoalPosition(5, targetPosition);
  delay(300);
  dxl.writeControlTableItem(PROFILE_VELOCITY, 5, 80);
  currentPositions[5] = targetPosition;
}

void setup() {
  DEBUG_SERIAL.begin(57600);
  DEBUG_SERIAL.println("Инициализация манипулятора");
  
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  for (int i=1; i<=jointN; i++) {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_POSITION);
    
    if (i == 5) {
      dxl.writeControlTableItem(PROFILE_VELOCITY, i, 80);
    } else if (i == 4) {
      dxl.writeControlTableItem(PROFILE_VELOCITY, i, 10);
    } else {
      dxl.writeControlTableItem(PROFILE_VELOCITY, i, MOVEMENT_SPEED);
    }
    
    dxl.writeControlTableItem(PROFILE_ACCELERATION, i, 20);
    dxl.torqueOn(i);
    
    currentPositions[i] = initialPositions[i];
    
    DEBUG_SERIAL.print("Сервопривод ");
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(" целевая позиция: ");
    DEBUG_SERIAL.print(initialDegrees[i]);
    DEBUG_SERIAL.println("°");
    delay(100);
  }
  
  smoothGoToHome();
  
  DEBUG_SERIAL.println("Манипулятор готов к работе");
  DEBUG_SERIAL.println("Начальные положения установлены:");
  DEBUG_SERIAL.println("Серво 1: 150° - основание");
  DEBUG_SERIAL.println("Серво 2: 48° - наклон");
  DEBUG_SERIAL.println("Серво 3: 254° - основное плечо");
  DEBUG_SERIAL.println("Серво 4: 150° - фиксирован");
  DEBUG_SERIAL.println("Серво 5: 150° - схват (открыт)");
  DEBUG_SERIAL.println("Изменения в цикле");
  DEBUG_SERIAL.println("Серво 2: уменьшен угол на 15° во всех позициях");
  DEBUG_SERIAL.println("Серво 5: увеличен угол захвата на 15° (до 195°)");
  delay(1000);
}

void loop() {
  DEBUG_SERIAL.println("Начало цикла: плавное перемещение предмета");
  
  smoothGoToHome();
  
  for (int step = 0; step < keyPositionsCount - 1; step++) {
    DEBUG_SERIAL.print("Движение: шаг ");
    DEBUG_SERIAL.print(step);
    DEBUG_SERIAL.println(step + 1);
    
    for (int servo = 1; servo <= jointN; servo++) {
      if (servo != 4) {
        currentPositions[servo] = keyPositions[step][servo];
      }
    }
    
    if ((step == 2 && step + 1 == 3) || (step == 7 && step + 1 == 8)) {
      smoothMove(keyPositions[step], keyPositions[step + 1], INTERMEDIATE_STEPS, STEP_DELAY);
      DEBUG_SERIAL.println("Быстрое движение схвата");
      fastGripperMove(keyPositions[step + 1][5]);
      delay(200);
    } else {
      smoothMove(keyPositions[step], keyPositions[step + 1], INTERMEDIATE_STEPS, STEP_DELAY);
    }
    
    switch(step + 1) {
      case 3:
        DEBUG_SERIAL.println("Пауза перед захватом");
        delay(500);
        break;
      case 4:
        DEBUG_SERIAL.println("Захват завершен");
        delay(600);
        break;
      case 8:
        DEBUG_SERIAL.println("Пауза перед отпусканием");
        delay(500);
        break;
      case 9:
        DEBUG_SERIAL.println("Предмет отпущен");
        delay(600);
        break;
    }
  }
  
  DEBUG_SERIAL.println("Цикл завершен");
  
  smoothGoToHome();
  
  delay(2000);
}