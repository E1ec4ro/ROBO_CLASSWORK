unsigned long flagTimeOff;
//==============================================
void move_case(char &bt_input)
{
  if (bt_input != 'S')
  {
    flagTimeOff = millis() + 2500;
    motor_on();
  }
  switch (bt_input) {
    // Вперед
    case 'F':
      //
      speed_left = maxSPEED;
      speed_right = maxSPEED;
      break;
    // Назад
    case 'B':
      speed_left = -maxSPEED;
      speed_right = -maxSPEED;
      break;
    // Влево
    case 'L':
      speed_left = -maxSPEED;
      speed_right = maxSPEED;
      break;
    // Вправо
    case 'R':
      speed_left  = maxSPEED;
      speed_right = -maxSPEED;
      break;
    // Прямо и влево
    case 'G':
      speed_left  = 0;
      speed_right = maxSPEED;
      break;
    // Прямо и вправо
    case 'I':
      speed_left   = maxSPEED;
      speed_right  = 0;
      break;
    // Назад и влево
    case 'H':
      speed_left  = -maxSPEED;
      speed_right  = 0;
      break;
    // Назад и вправо
    case 'J':
      speed_right  = -maxSPEED;
      speed_left  = 0;
      break;
    // Стоп
    case 'S':
      speed_left  = 0;
      speed_right = 0;
      break;
    // Скорость 0%
    case '0':
      maxSPEED = 300000;
      break;
    // Скорость 10%
    case '1':
      maxSPEED = 350000;
      break;
    // Скорость 20%
    case '2':
      maxSPEED = 400000;
      break;
    // Скорость 30%
    case '3':
      maxSPEED = 450000;
      break;
    // Скорость 40%
    case '4':
      maxSPEED = 500000;
      break;
    // Скорость 50%
    case '5':
      maxSPEED = 550000;
      break;
    // Скорость 60%
    case '6':
      maxSPEED = 600000;
      break;
    // Скорость 70%
    case '7':
      maxSPEED = 650000;
      break;
    case '8':
      // Скорость 80%
      maxSPEED = 700000;
      break;
    // Скорость 90%
    case '9':
      maxSPEED = 750000;
      break;
    // Скорость 100%
    case 'q':
      maxSPEED = 1000000;
      break;
    case 'V':
      break;
    case 'v':
      break;
    case 'X':
      break;
    case 'x':
      break;
    case 'D':
      break;
  }
    // если требуемая скорость не равна с текущей.
  if (speed_last_L != speed_left)
  {
    LEFT_STEPS_counter  = 2;
   // flag_new_speed_left = true;
  }
  // если требуемая скорость не равна с текущей.
  if (speed_last_R != speed_right)
  {
    RIGHT_STEPS_counter  = 2;
    //flag_new_speed_right = true;
  }
}
