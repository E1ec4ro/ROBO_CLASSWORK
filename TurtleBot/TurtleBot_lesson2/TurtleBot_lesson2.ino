#include <DynamixelWorkbench.h>

#define BAUDRATE 1000000
#define ACCELERATION 50

uint8_t id1 = 1;
uint8_t id2 = 2;

DynamixelWorkbench wb;
uint16_t model_number = 0;

void setup(){
  Serial.begin(9600);
  while(!Serial);

  const char *log;

  bool result = wb.init("", BAUDRATE);
  if (result == false){
    Serial.println(log);
    Serial.println("Failed to init");  
  }
  else{
    Serial.print("Succeeded to init :");
    Serial.println(BAUDRATE);
  }

  wb.ping(id1, &model_number, &log);
  wb.ping(id2, &model_number, &log);
  
  wb.wheelMode(id1, 0, &log);
  wb.jointMode(id2, 0, 0, &log);

  for(int count = 0; count < 3; count++){
    wb.goalVelocity(id1, (int32_t)200);
    wb.goalPosition(id2, (int32_t)0);
    delay(3000);

    wb.goalVelocity(id1, (int32_t)-200);
    wb.goalPosition(id2, (int32_t)1023);
    delay(3000);
  }
  wb.goalVelocity(id1, (int32_t)0);
}
