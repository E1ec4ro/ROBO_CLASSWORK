# Решение проблем с управлением роботом

## Проблема: Команды отправляются, но робот не двигается

### Шаг 1: Проверка базовых компонентов

Выполните в терминале:

```bash
# 1. Проверка ROS Master
rostopic list
# Если ошибка - запустите: roscore

# 2. Проверка топика /cmd_vel
rostopic info /cmd_vel
# Должен показать подписчиков (Subscribers)

# 3. Проверка подписчиков
rostopic info /cmd_vel | grep Subscribers
# Должно быть: Subscribers: /gazebo (или похожее)
```

### Шаг 2: Проверка Gazebo

```bash
# Проверка, что Gazebo запущен
rosnode list | grep gazebo

# Проверка модели робота
rosparam get /robot_description

# Проверка, что робот заспавнен
rostopic list | grep odom
# Должен быть топик /odom
```

### Шаг 3: Проверка плагина

```bash
# Проверка логов Gazebo
rosnode info /gazebo

# Проверка параметров плагина
rosparam list | grep diff_drive
```

### Шаг 4: Диагностика с помощью скрипта

```bash
chmod +x /workspace/catkin_ws/src/test_robot/scripts/check_robot.sh
/workspace/catkin_ws/src/test_robot/scripts/check_robot.sh
```

## Частые проблемы и решения

### Проблема 1: Нет подписчиков на /cmd_vel

**Симптомы:**
```bash
rostopic info /cmd_vel
# Показывает: Subscribers: None
```

**Решение:**
1. Убедитесь, что Gazebo запущен
2. Убедитесь, что робот заспавнен
3. Перезапустите launch файл:
   ```bash
   roslaunch test_robot gazebo_world.launch
   ```

### Проблема 2: Плагин не загружен

**Симптомы:**
- Нет топика /odom
- В логах Gazebo ошибки о плагине

**Решение:**
1. Проверьте URDF файл на наличие плагина
2. Убедитесь, что пакет gazebo_ros установлен:
   ```bash
   rospack find gazebo_ros
   ```
3. Пересоберите пакет:
   ```bash
   cd /workspace/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

### Проблема 3: Робот не заспавнен

**Симптомы:**
- В Gazebo нет робота
- Нет топика /odom

**Решение:**
1. Проверьте launch файл
2. Заспавните робота вручную:
   ```bash
   rosrun gazebo_ros spawn_model -urdf -model simple_robot -param robot_description -x 0 -y 0 -z 0.1
   ```

### Проблема 4: Неправильный топик

**Симптомы:**
- Плагин подписан на другой топик

**Решение:**
Проверьте конфигурацию плагина в URDF. Топик должен быть:
```xml
<command_topic>cmd_vel</command_topic>
```

## Полная перезагрузка

Если ничего не помогает:

```bash
# 1. Остановите все
pkill -f gazebo
pkill -f roscore

# 2. Запустите заново
roscore &
sleep 2
roslaunch test_robot gazebo_world.launch

# 3. Подождите 5 секунд для загрузки

# 4. Проверьте
rostopic list
rostopic info /cmd_vel
```

## Проверка работы плагина

```bash
# Просмотр сообщений одометрии
rostopic echo /odom

# Если сообщения приходят - плагин работает
# Если нет - плагин не загружен
```

## Альтернативный способ управления

Если плагин не работает, можно управлять напрямую через joint_state_publisher:

```bash
# Установка скорости для левого колеса
rostopic pub /left_wheel_joint/command std_msgs/Float64 "data: 1.0"

# Установка скорости для правого колеса
rostopic pub /right_wheel_joint/command std_msgs/Float64 "data: 1.0"
```

Но это требует дополнительной настройки контроллеров.

