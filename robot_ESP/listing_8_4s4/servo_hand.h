#include <ESP32Servo.h>
Servo one_hand;
// Published values for SG90 servos; adjust if needed
int minUs = 800;
int maxUs = 2500;
int one_handGPIO = 19;
bool one_handattach = false;

void servo_hand_setup()
{
  one_hand.setPeriodHertz(50);      // Standard 50hz servo
  //one_hand.attach(one_handGPIO, minUs, maxUs);
  //one_hand.write(90);
  one_handattach = false;
}

void servo_hand_bottom(double test_angle)
{
  if (test_angle < - 20.0) //Если вошли в критичный режим
  {
    if (one_handattach == false)
    {
      one_handattach = true;
      one_hand.attach(one_handGPIO, minUs, maxUs);
    }
    one_hand.write(200);
  }
  else if (test_angle >  20.0) //Если вошли в критичный режим
  {
    if (one_handattach == false)
    {
      one_handattach = true;
      one_hand.attach(one_handGPIO, minUs, maxUs);
    }
    one_hand.write(0);
  }
}
void servo_hand_up()
{
  static uint32_t timess = 0;

  if (one_hand.attached())
  {
    if (one_hand.read() != 100)
    {
      one_hand.write(100);
      timess = millis() + 4000;
    }
    else
    {
      if (timess < millis())
      {
        one_hand.detach();
        one_handattach = false;
      }
    }
  }
}
