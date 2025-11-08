#include <DynamixelWorkbench.h>

#define BAUDRATE  1000000
#define CAM_ID    51
#define DEVICE_NAME "3"

struct TrackingCamBlobInfo_t
{
  uint8_t type;
  uint8_t dummy;
  uint16_t cx;
  uint16_t cy;
  uint32_t area;
  uint16_t left;
  uint16_t right;
  uint16_t top;
  uint16_t bottom;
};

DynamixelWorkbench dxl_wb;
unsigned long previousMillis = 0; // stores last time cam was updated

void setup()
{
  Serial.begin(57600);
  while (!dxl_wb.init(DEVICE_NAME, BAUDRATE))
  {}
  while (!dxl_wb.ping(CAM_ID))
  {}
}

void loop()
{
  int max_n = 5;
  int n = 0;
  TrackingCamBlobInfo_t blob[10];
  for (int i = 0; i < max_n; i++)
  {
    uint32_t resp[16];
    if (!dxl_wb.readRegister(CAM_ID, 16 + i * 16, 16, resp))
      break;
      
    int idx = 0;
    blob[i].type = resp[idx++];
    if (blob[i].type == 0xFF)
      break;
    blob[i].dummy = resp[idx++];
    blob[i].cx = resp[idx] + (resp[idx + 1] << 8);
    idx += 2;
    blob[i].cy = resp[idx] + (resp[idx + 1] << 8);
    idx += 2;
    blob[i].area = (resp[idx] + (resp[idx + 1] << 8)) * 4;
    idx += 2;
    blob[i].left = resp[idx] + (resp[idx + 1] << 8);
    idx += 2;
    blob[i].right = resp[idx] + (resp[idx + 1] << 8);
    idx += 2;
    blob[i].top = resp[idx] + (resp[idx + 1] << 8);
    idx += 2;
    blob[i].bottom = resp[idx] + (resp[idx + 1] << 8);
    idx += 2;
    n++;
  }

  Serial.println("n = " + String(n));
  for (int i = 0; i < n; i++)
  {
    Serial.print( String(blob[i].type) + " " + String(blob[i].cx) + " " + String(blob[i].cy) + " " +
                  String(blob[i].area) + " " + String(blob[i].left) + " " + String(blob[i].right) + " " +
                  String(blob[i].top) + " " + String(blob[i].bottom) + "\n");
  }

  // wait for the next frame
  while (millis() - previousMillis < 33)
  {};
  previousMillis = millis();
}
