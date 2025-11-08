unsigned long flagTimeOff;
unsigned long flagTimeStop;
char COMMAND_ser;
int com_calc = 0;
//==============================================
void move_case(char &bt_input, char &COMMAND_ser)
{
  if (COMMAND_ser == 'P')
  {
    uint8_t Kp = uint8_t(bt_input);
    maxSPEED = minSPEED + (minSPEED / 255) * 7 * int32_t(Kp);
    maxSPEED = constrain(maxSPEED, -8 * minSPEED, 8 * minSPEED);
    //    motor_on();
  }
  if (COMMAND_ser == 'T')
  {
    int8_t Kt = int8_t(bt_input);
    Kt = constrain(Kt, -2, 2);
    speed_left = int32_t(Kt) * maxSPEED / 2;
    speed_right = -int32_t(Kt) * maxSPEED / 2;
    if (Kt != 0)
    {
      flagTimeOff = millis() + 10000;
      flagTimeStop = millis() + 3000;
      motor_on();
    }
  }
  if (COMMAND_ser == 'F')
  {
    int8_t Kf = int8_t(bt_input);
    Kf = constrain(Kf, -2, 2);
    speed_left = int32_t(Kf) * maxSPEED / 2;
    speed_right = int32_t(Kf) * maxSPEED / 2;
    if (Kf != 0)
    {
      flagTimeOff = millis() + 10000;
      flagTimeStop = millis() + 3000;
      motor_on();
    }
  }

  // если требуемая скорость не равна с текущей.
  if (speed_last_L != speed_left)
  {
    LEFT_STEPS_counter  = 2;
  }
  // если требуемая скорость не равна с текущей.
  if (speed_last_R != speed_right)
  {
    RIGHT_STEPS_counter  = 2;
  }
}
