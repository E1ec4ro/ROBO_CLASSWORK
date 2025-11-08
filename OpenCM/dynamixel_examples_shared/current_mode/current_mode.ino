#include <Dynamixel2Arduino.h>
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_ID = 1;
double duty_cycle=100.0;
double goal_current=20.0;

Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(DXL_ID);
  while(!dxl.torqueOff(DXL_ID));
  dxl.writeControlTableItem(CURRENT_LIMIT,DXL_ID,100);
  dxl.setOperatingMode(DXL_ID,OP_CURRENT);
  while(!dxl.torqueOn(DXL_ID));

  dxl.setGoalCurrent(DXL_ID,0,UNIT_MILLI_AMPERE);
  delay(5000);
  dxl.setGoalCurrent(DXL_ID,goal_current,UNIT_MILLI_AMPERE);
}

void loop() {
    Serial.print(dxl.getPresentPWM(DXL_ID,UNIT_PERCENT));
    Serial.print(",");
    Serial.print(dxl.getPresentCurrent(DXL_ID,UNIT_MILLI_AMPERE));
    Serial.print(",");
    Serial.println(dxl.getPresentVelocity(DXL_ID,UNIT_RPM));
}
