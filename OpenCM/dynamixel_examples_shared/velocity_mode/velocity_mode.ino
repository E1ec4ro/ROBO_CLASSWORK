#include <Dynamixel2Arduino.h>
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_ID = 1;
double max_velocity=50.0;
double goal_velocity=20.0;
double acceleration_rpm2=6500; //rpm2
double acceleration_time=1000; //msec

Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(DXL_ID);
  while(!dxl.torqueOff(DXL_ID));
  dxl.setOperatingMode(DXL_ID,OP_VELOCITY);
  dxl.writeControlTableItem(VELOCITY_LIMIT,DXL_ID,constrain((uint32_t)(max_velocity/0.229),0,2047));
  //Velocity profile
  //dxl.writeControlTableItem(DRIVE_MODE,DXL_ID,dxl.readControlTableItem(DRIVE_MODE,DXL_ID)&0x0B);
  //dxl.writeControlTableItem(PROFILE_ACCELERATION,DXL_ID,constrain((uint32_t)(acceleration_rpm2/214.577),0,32737));
  //Time profile
  dxl.writeControlTableItem(DRIVE_MODE,DXL_ID,dxl.readControlTableItem(DRIVE_MODE,DXL_ID)|0x04);
  dxl.writeControlTableItem(PROFILE_ACCELERATION,DXL_ID,1000); //Acceleration time (ms)
  
  while(!dxl.torqueOn(DXL_ID));

  dxl.setGoalVelocity(DXL_ID,0,UNIT_RPM);
  delay(5000);  
  dxl.setGoalVelocity(DXL_ID,goal_velocity,UNIT_RPM);
}

void loop() {
    Serial.print(dxl.getPresentVelocity(DXL_ID,UNIT_RPM));
    Serial.print(",");
    Serial.print(dxl.readControlTableItem(VELOCITY_TRAJECTORY,DXL_ID)*0.229);
    Serial.print(",");
    Serial.println(dxl.readControlTableItem(GOAL_VELOCITY,DXL_ID)*0.229);
}
