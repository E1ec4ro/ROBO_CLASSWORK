# Быстрое исправление: Робот не реагирует на команды

## Проблема: `Subscribers: None` на топике /cmd_vel

Это означает, что плагин дифференциального привода не загружен или не подключен.

## Решение 1: Перезапуск Gazebo с роботом

```bash
# 1. Остановите текущий Gazebo
pkill -f gazebo

# 2. Убедитесь, что ROS master запущен
roscore &
sleep 2

# 3. Пересоберите пакет (если изменили URDF)
cd /workspace/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 4. Запустите Gazebo с роботом
roslaunch test_robot gazebo_world.launch
```

**Подождите 10-15 секунд** для полной загрузки, затем проверьте:

```bash
rostopic info /cmd_vel
```

Должно показать: `Subscribers: /gazebo` (или похожее).

## Решение 2: Ручной заспавн робота

Если launch файл не работает:

```bash
# 1. Запустите пустой Gazebo
rosrun gazebo_ros gazebo &

# 2. Подождите 5 секунд
sleep 5

# 3. Загрузите модель робота
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

rosparam set /robot_description "$(cat /workspace/catkin_ws/src/test_robot/urdf/simple_robot.urdf)"

# 4. Заспавньте робота
rosrun gazebo_ros spawn_model -urdf \
    -model simple_robot \
    -param robot_description \
    -x 0 -y 0 -z 0.1

# 5. Подождите 3 секунды
sleep 3

# 6. Проверьте
rostopic info /cmd_vel
```

## Решение 3: Проверка плагина

Если плагин все еще не работает:

```bash
# 1. Проверьте, что пакет gazebo_ros установлен
rospack find gazebo_ros

# 2. Проверьте наличие плагина
ls /opt/ros/noetic/lib/libgazebo_ros_diff_drive.so

# 3. Проверьте логи Gazebo на ошибки
rosnode info /gazebo
```

## Решение 4: Альтернативный плагин

Если стандартный плагин не работает, можно использовать другой подход. Создайте упрощенную версию без плагина и управляйте через joint_state_publisher, но это сложнее.

## Проверка после исправления

После применения любого из решений:

```bash
# 1. Проверьте подписчиков
rostopic info /cmd_vel
# Должно быть: Subscribers: /gazebo

# 2. Проверьте одометрию
rostopic echo /odom
# Должны приходить сообщения

# 3. Отправьте тестовую команду
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -1
```

## Автоматическое исправление

Используйте скрипт:

```bash
chmod +x /workspace/catkin_ws/src/test_robot/scripts/fix_robot.sh
/workspace/catkin_ws/src/test_robot/scripts/fix_robot.sh
```

## Если ничего не помогает

1. Проверьте, что в Gazebo виден робот (синий куб с колесами)
2. Проверьте логи Gazebo на ошибки загрузки плагина
3. Убедитесь, что URDF файл правильный:
   ```bash
   check_urdf /workspace/catkin_ws/src/test_robot/urdf/simple_robot.urdf
   ```

