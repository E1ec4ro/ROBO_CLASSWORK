# Управление тестовым роботом

## Способы отправки команд движения

### Способ 1: Через команду rostopic pub (быстрый способ)

**Движение вперед:**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

**Движение назад:**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: -0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

**Поворот влево (против часовой стрелки):**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

**Поворот вправо (по часовой стрелке):**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.5"
```

**Движение вперед с поворотом:**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3"
```

**Остановка:**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Способ 2: Использование Python скрипта

Сначала сделайте скрипт исполняемым:
```bash
chmod +x /workspace/catkin_ws/src/test_robot/scripts/move_robot.py
```

Затем используйте:
```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Движение вперед
python3 /workspace/catkin_ws/src/test_robot/scripts/move_robot.py forward

# Движение назад
python3 /workspace/catkin_ws/src/test_robot/scripts/move_robot.py backward

# Поворот влево
python3 /workspace/catkin_ws/src/test_robot/scripts/move_robot.py left

# Поворот вправо
python3 /workspace/catkin_ws/src/test_robot/scripts/move_robot.py right

# Остановка
python3 /workspace/catkin_ws/src/test_robot/scripts/move_robot.py stop
```

### Способ 3: Непрерывное движение (одна команда)

Для непрерывного движения вперед:
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Флаг `-r 10` означает публикацию 10 раз в секунду. Для остановки нажмите `Ctrl+C`.

### Способ 4: Интерактивное управление (teleop)

Создайте простой скрипт для интерактивного управления:

```bash
# Сохраните как teleop.py
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == '__main__':
    rospy.init_node('teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    print("Управление:")
    print("  w - вперед")
    print("  s - назад")
    print("  a - влево")
    print("  d - вправо")
    print("  x - остановка")
    print("  q - выход")
    
    while not rospy.is_shutdown():
        key = get_key()
        twist = Twist()
        
        if key == 'w':
            twist.linear.x = 0.5
        elif key == 's':
            twist.linear.x = -0.5
        elif key == 'a':
            twist.angular.z = 0.5
        elif key == 'd':
            twist.angular.z = -0.5
        elif key == 'x':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif key == 'q':
            break
        
        pub.publish(twist)
        rate.sleep()
```

## Параметры скорости

- **linear.x**: линейная скорость (м/с)
  - Положительное значение = вперед
  - Отрицательное значение = назад
  - Диапазон: обычно -1.0 до 1.0 м/с

- **angular.z**: угловая скорость (рад/с)
  - Положительное значение = поворот против часовой стрелки (влево)
  - Отрицательное значение = поворот по часовой стрелке (вправо)
  - Диапазон: обычно -1.0 до 1.0 рад/с

## Проверка работы

### Просмотр топика cmd_vel
```bash
rostopic echo /cmd_vel
```

### Просмотр одометрии
```bash
rostopic echo /odom
```

### Просмотр всех топиков
```bash
rostopic list
```

## Примеры команд

### Квадрат (движение по квадрату)
```bash
# Вперед 2 секунды
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -1 &
sleep 2
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" -1

# Поворот 90 градусов (примерно 1.5 секунды при скорости 1 рад/с)
rostopic pub /cmd_vel geometry_msgs/Twist "angular: {z: 1.0}" -1 &
sleep 1.5
rostopic pub /cmd_vel geometry_msgs/Twist "angular: {z: 0.0}" -1
```

### Круг (движение по кругу)
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.3}, angular: {z: 0.5}"
```

Для остановки нажмите `Ctrl+C`.

