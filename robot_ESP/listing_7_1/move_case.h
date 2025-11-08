unsigned long flagTimeOff;
unsigned long flagTimeStop;
//==============================================
void move_case(char &bt_input)
{
  if (bt_input != 'S')
  {
    flagTimeOff = millis() + 2500;
    flagTimeStop = millis() + 200;
    motor_on();
  }
  switch (bt_input) {
    // Вперед
    case 'A':
      //
      speed_left = maxSPEED;
      speed_right = maxSPEED;
      break;
    // Назад
    case 'E':
      speed_left = -maxSPEED;
      speed_right = -maxSPEED;
      break;
    // Влево
    case 'C':
      speed_left = -maxSPEED;
      speed_right = maxSPEED;
      break;
    // Вправо
    case 'B':
      speed_left  = maxSPEED;
      speed_right = -maxSPEED;
      break;
    // Стоп
    case 'S':
      speed_left  = 0;
      speed_right = 0;
      break;
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
