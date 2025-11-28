#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial3
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

float initialDegrees[jointN + 1] = { 0, 150.0, 48.0, 254.0, 150.0, 150.0 };

int initialPositions[jointN + 1] = { 0,
                                     degreesToUnits(initialDegrees[1]),
                                     degreesToUnits(initialDegrees[2]),
                                     degreesToUnits(initialDegrees[3]),
                                     degreesToUnits(initialDegrees[4]),
                                     degreesToUnits(initialDegrees[5]) };

int keyPositions[][6] = {
  { 0, initialPositions[1], initialPositions[2], initialPositions[3], initialPositions[4], initialPositions[5] },

  { 0, degreesToUnits(140), degreesToUnits(105), degreesToUnits(300), initialPositions[4], degreesToUnits(150) },

  { 0, degreesToUnits(130), degreesToUnits(125), degreesToUnits(310), initialPositions[4], degreesToUnits(150) },

  { 0, degreesToUnits(130), degreesToUnits(125), degreesToUnits(310), initialPositions[4], degreesToUnits(195) },

  { 0, degreesToUnits(140), degreesToUnits(95), degreesToUnits(290), initialPositions[4], degreesToUnits(195) },

  { 0, degreesToUnits(160), degreesToUnits(80), degreesToUnits(270), initialPositions[4], degreesToUnits(195) },

  { 0, degreesToUnits(170), degreesToUnits(70), degreesToUnits(260), initialPositions[4], degreesToUnits(195) },

  { 0, degreesToUnits(140), degreesToUnits(105), degreesToUnits(300), initialPositions[4], degreesToUnits(195) },

  { 0, degreesToUnits(140), degreesToUnits(105), degreesToUnits(300), initialPositions[4], degreesToUnits(150) },

  { 0, initialPositions[1], initialPositions[2], initialPositions[3], initialPositions[4], initialPositions[5] }
};

#define keyPositionsCount 10

const int INTERMEDIATE_STEPS = 35;
const int STEP_DELAY = 40;
const int MOVEMENT_SPEED = 100;
const int GRIPPER_SPEED = 150;

int durationB = 0;
int durationF = 0;

bool booton;

int currentPositions[jointN + 1] = { 0, 0, 0, 0, 0, 0 };

#define IN1_PIN 13
#define IN2_PIN 14
#define IN3_PIN 16
#define IN4_PIN 17

#define PIN_TRIG_F 0
#define PIN_ECHO_F 2
#define PIN_TRIG_B 3
#define PIN_ECHO_B 8

#define PIN_BUTTON 10

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void runAllMotorsForward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}

void runAllMotorsBackward() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}

void goMotorsLEFT() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void goMotorsRIGHT() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

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
  int currentServoPositions[jointN + 1] = { 0 };
  for (int servo = 1; servo <= jointN; servo++) {
    if (servo == 4) {
      currentServoPositions[servo] = initialPositions[4];
    } else {
      currentServoPositions[servo] = dxl.getPresentPosition(servo);
    }
  }

  smoothMove(currentServoPositions, initialPositions, INTERMEDIATE_STEPS * 2, STEP_DELAY * 2);

  for (int servo = 1; servo <= jointN; servo++) {
    int actualPosition = dxl.getPresentPosition(servo);
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

void soundDatchikF() {
  digitalWrite(PIN_TRIG_F, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG_F, LOW);

  durationF = pulseIn(PIN_ECHO_F, HIGH) / 58;
}

void soundDatchikB() {
  digitalWrite(PIN_TRIG_B, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG_B, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG_B, LOW);

  durationB = pulseIn(PIN_ECHO_B, HIGH) / 58;
}

void setup() {
  DEBUG_SERIAL.begin(57600);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  pinMode(PIN_TRIG_F, OUTPUT);
  pinMode(PIN_ECHO_F, INPUT);
  pinMode(PIN_TRIG_B, OUTPUT);
  pinMode(PIN_ECHO_B, INPUT);

  pinMode(PIN_BUTTON, INPUT);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 1; i <= jointN; i++) {
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

    delay(100);
  }

  smoothGoToHome();

  delay(1000);
}

// void move() {
//   soundDatchikF();
//   soundDatchikB();
//   runAllMotorsForward();
//   if (durationF <= 5) {
//     stopMotors();
//     goMotorsLeft
//   }
// }

void loop() {
  int buttonState = digitalRead(PIN_BUTTON);

  if (buttonState == HIGH) {
    booton = !booton;
  }

  if (booton == true) {
    delay(5000);
    smoothGoToHome();

    for (int step = 0; step < keyPositionsCount - 1; step++) {
      soundDatchikF();

      if (durationF <= 5) {
        goMotorsLEFT();
        stopMotors();
      } else {
        runAllMotorsForward();
      }

      for (int servo = 1; servo <= jointN; servo++) {
        if (servo != 4) {
          currentPositions[servo] = keyPositions[step][servo];
        }
      }

      if ((step == 2 && step + 1 == 3) || (step == 7 && step + 1 == 8)) {
        smoothMove(keyPositions[step], keyPositions[step + 1], INTERMEDIATE_STEPS, STEP_DELAY);
        fastGripperMove(keyPositions[step + 1][5]);
        delay(200);
      } else {
        smoothMove(keyPositions[step], keyPositions[step + 1], INTERMEDIATE_STEPS, STEP_DELAY);
      }

      switch (step + 1) {
        case 3:
          delay(500);
          break;
        case 4:
          delay(600);
          break;
        case 8:
          delay(500);
          break;
        case 9:
          delay(600);
          break;
      }
    }

    stopMotors();

    smoothGoToHome();

    delay(2000);
    // } else if (booton == 2) {

    //   povorot45();
    //   delay(5000);
    // }
  }
}