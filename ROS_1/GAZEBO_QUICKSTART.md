# Быстрый старт с Gazebo и тестовым роботом

## Подготовка

### 1. Сборка пакета test_robot

**Через веб-интерфейс:**
1. Откройте `http://localhost:6080/vnc.html`
2. Откройте терминал
3. Выполните:

```bash
cd /workspace/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

**Через терминал хоста:**
```powershell
docker exec -it ros_gazebo_rviz bash
cd /workspace/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Запуск Gazebo с тестовым роботом

### Вариант 1: Полный запуск (рекомендуется)

В терминале контейнера выполните:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Запуск всего сразу
roslaunch test_robot gazebo_world.launch
```

Это запустит:
- Gazebo с пустым миром
- Тестового робота (синий куб с двумя колесами)
- Все необходимые узлы ROS

### Вариант 2: Пошаговый запуск

**Терминал 1 - ROS Master:**
```bash
source /opt/ros/noetic/setup.bash
roscore
```

**Терминал 2 - Gazebo:**
```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash
roslaunch test_robot gazebo_world.launch
```

## Управление роботом

После запуска можно управлять роботом через топики:

### Просмотр топиков
```bash
rostopic list
```

### Управление движением
```bash
# Публикация команд движения
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Просмотр данных одометрии
```bash
rostopic echo /odom
```

## Запуск RVIZ для визуализации

В новом терминале:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash
rosrun rviz rviz
```

В RVIZ:
1. Добавьте `RobotModel` (Fixed Frame: `base_link`)
2. Добавьте `TF` для просмотра системы координат
3. Добавьте `Odometry` для просмотра траектории

## Альтернативные примеры

### Запуск TurtleBot3 (если установлен)

```bash
# Установка TurtleBot3 (если нужно)
sudo apt-get install ros-noetic-turtlebot3*

# Запуск
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

### Запуск простого мира Gazebo

```bash
# Просто пустой мир
rosrun gazebo_ros gazebo

# Мир с объектами
rosrun gazebo_ros gazebo worlds/robocup14_spl_field.world
```

## Решение проблем

### Gazebo не запускается

1. Проверьте, что ROS master запущен: `rostopic list`
2. Проверьте логи: `rosnode info /gazebo`
3. Попробуйте запустить с verbose: `rosrun gazebo_ros gazebo --verbose`

### Робот не появляется в Gazebo

1. Проверьте, что пакет собран: `rospack find test_robot`
2. Проверьте URDF файл: `check_urdf $(rospack find test_robot)/urdf/simple_robot.urdf`
3. Проверьте логи спавна в терминале

### Нет управления роботом

1. Проверьте топики: `rostopic list | grep cmd_vel`
2. Проверьте, что плагин загружен: `rostopic echo /odom`
3. Убедитесь, что публикуете в правильный топик

## Полезные команды

```bash
# Просмотр всех узлов
rosnode list

# Просмотр всех топиков
rostopic list

# Просмотр информации о топике
rostopic info /cmd_vel

# Просмотр сообщений в реальном времени
rostopic echo /cmd_vel

# Просмотр параметров
rosparam list

# Просмотр модели робота
rosrun xacro xacro $(rospack find test_robot)/urdf/simple_robot.urdf
```

