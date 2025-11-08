#include <Dynamixel2Arduino.h>
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_ID = 1;

Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(DXL_ID);
  while(!dxl.torqueOff(DXL_ID));
  dxl.setOperatingMode(DXL_ID,OP_POSITION);
  
  //Time profile
  dxl.writeControlTableItem(DRIVE_MODE,DXL_ID,dxl.readControlTableItem(DRIVE_MODE,DXL_ID)|0x04);
  dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID,1500); //Trajectory time (ms)
  dxl.writeControlTableItem(PROFILE_ACCELERATION,DXL_ID,200); //Acceleration time (ms)
  while(!dxl.torqueOn(DXL_ID));

  //PID Gains
  dxl.writeControlTableItem(POSITION_P_GAIN,DXL_ID,400);
  dxl.writeControlTableItem(POSITION_I_GAIN,DXL_ID,50);
  dxl.writeControlTableItem(POSITION_D_GAIN,DXL_ID,100);
  dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN,DXL_ID,500);
  dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN,DXL_ID,100);

  dxl.writeControlTableItem(GOAL_POSITION,DXL_ID,2048);
  delay(5000);
  dxl.writeControlTableItem(GOAL_POSITION,DXL_ID,1023);
}

void loop() {
    Serial.print(dxl.readControlTableItem(PRESENT_POSITION,DXL_ID));
    Serial.print(",");
    Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,DXL_ID));
    Serial.print(",");
    Serial.println(dxl.readControlTableItem(GOAL_POSITION,DXL_ID));
}
