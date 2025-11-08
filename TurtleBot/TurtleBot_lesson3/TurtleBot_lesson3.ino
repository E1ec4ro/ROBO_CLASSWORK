#include <DynamixelWorkbench.h>

#define BAUDRATE 1000000
#define ACCELERATION 50
#define DXL_ID1 1
#define DXL_ID2 2

uint8_t id1 = DXL_ID1;
uint8_t id2 = DXL_ID2;
int32_t acc = ACCELERATION;

DynamixelWorkbench dx;

double* goalWheelVel;
double* goalRobotVel;

double L = 0.155;
double R = 0.05;

void setup(){
  Serial.begin(9600);
  Serial.setTimeout(3);
  while(!Serial);

  goalWheelVel = new double[2];
  goalWheelVel [0] = 0.0;
  goalWheelVel [1] = 0.0;
  
  goalWheelVel = new double [2];
  goalWheelVel [0] = 0.0;
  goalWheelVel [1] = 0.0;

  dx.init("", BAUDRATE);

  dx.ping(id1);
  dx.ping(id2);

  dx.wheelMode(id1, acc);
  dx.wheelMode(id2, acc);
}

void loop()
{
  if (Serial.available() > 0){
    String str = Serial.readString();
    parsing(str);
    setGoalRobotVel();
  }
}

void setGoalRobotVel()
{
  goalWheelVel [0] = (2 * goalRobotVel[0] - L * goalRobotVel[1])/(2 * R);
  goalWheelVel [1] = (2 * goalRobotVel[0] + L * goalRobotVel[1])/(2 * R);

  turtleMove(id1, goalWheelVel[0]);
  turtleMove(id2, goalWheelVel[1]);

  Serial.println(goalWheelVel[0]);
  Serial.println(goalWheelVel[1]);

  dx.ping(id1);
  dx.ping(id2);
}

bool turtleMove(uint8_t id, float vel)
{
  return dx.goalVelocity(id, vel);
}

void parsing(String str){
  int divider = str.indexOf(';');
  String buf = str.substring(0, divider);
  goalRobotVel[0] = buf.toFloat();
  buf = str.substring(divider + 1);
  goalRobotVel[1] = buf.toFloat();
}
