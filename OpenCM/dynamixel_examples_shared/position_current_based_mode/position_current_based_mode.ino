#include <Dynamixel2Arduino.h>
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_ID = 1;
double goal_position=90.0;
double velocity_rpm2=20; //rpm
double max_current=20; //mA
double acceleration_rpm2=6500; //rpm2

Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(DXL_ID);
  while(!dxl.torqueOff(DXL_ID));
  dxl.setOperatingMode(DXL_ID,OP_CURRENT_BASED_POSITION);
  //Velocity profile
  //dxl.writeControlTableItem(DRIVE_MODE,DXL_ID,dxl.readControlTableItem(DRIVE_MODE,DXL_ID)&0x0B);
  //dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID,constrain((uint32_t)(velocity_rpm2/0.229),0,32737));
  //dxl.writeControlTableItem(PROFILE_ACCELERATION,DXL_ID,constrain((uint32_t)(acceleration_rpm2/214.577),0,32737));
  //Time profile
  dxl.writeControlTableItem(DRIVE_MODE,DXL_ID,dxl.readControlTableItem(DRIVE_MODE,DXL_ID)|0x04);
  dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID,2000); //Trajectory time (ms)
  dxl.writeControlTableItem(PROFILE_ACCELERATION,DXL_ID,100); //Acceleration time (ms)
  while(!dxl.torqueOn(DXL_ID));

  dxl.setGoalPosition(DXL_ID,0,UNIT_DEGREE);
  delay(5000);
  dxl.setGoalCurrent(DXL_ID,max_current,UNIT_MILLI_AMPERE);
  dxl.setGoalPosition(DXL_ID,goal_position,UNIT_DEGREE);
}

void loop() {
    Serial.print(dxl.getPresentPosition(DXL_ID,UNIT_DEGREE));
    Serial.print(",");
    Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,DXL_ID)*360/4096);
    Serial.print(",");
    Serial.print(dxl.readControlTableItem(GOAL_POSITION,DXL_ID)*360/4096);
    Serial.print(",");
    Serial.println(dxl.getPresentCurrent(DXL_ID,UNIT_MILLI_AMPERE));
}
