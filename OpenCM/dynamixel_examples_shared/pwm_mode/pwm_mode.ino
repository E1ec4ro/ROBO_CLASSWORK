#include <Dynamixel2Arduino.h>
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_ID = 1;
double duty_cycle=100.0;
double pwm_target=0.0;

Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(DXL_ID);
  while(!dxl.torqueOff(DXL_ID));
  dxl.writeControlTableItem(PWM_LIMIT,DXL_ID,constrain((uint32_t)(8.85*duty_cycle),0,885));
  dxl.setOperatingMode(DXL_ID,OP_PWM);
  while(!dxl.torqueOn(DXL_ID));

  dxl.setGoalPWM(DXL_ID,0,UNIT_PERCENT);
  delay(5000);
  dxl.setGoalPWM(DXL_ID,pwm_target,UNIT_PERCENT);
}

void loop() {
    Serial.print(dxl.getPresentPWM(DXL_ID,UNIT_PERCENT));
    Serial.print(",");
    Serial.print(dxl.getPresentCurrent(DXL_ID,UNIT_MILLI_AMPERE));
    Serial.print(",");
    Serial.println(dxl.getPresentVelocity(DXL_ID,UNIT_RPM));
}
