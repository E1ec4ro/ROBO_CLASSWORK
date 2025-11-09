# Примеры использования ROS в Docker

## Базовые примеры

### 1. Запуск ROS Master

```bash
roscore
```

### 2. Запуск простого узла (talker)

В новом терминале:

```bash
rosrun rospy_tutorials talker
```

### 3. Запуск узла listener

В еще одном терминале:

```bash
rosrun rospy_tutorials listener
```

### 4. Просмотр активных топиков

```bash
rostopic list
```

### 5. Просмотр сообщений в топике

```bash
rostopic echo /chatter
```

## Работа с Gazebo

### Запуск пустого мира Gazebo

```bash
rosrun gazebo_ros gazebo
```

### Запуск мира с роботом TurtleBot

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

### Управление роботом

```bash
# В новом терминале
rosrun turtlebot3_teleop turtlebot3_teleop_key
```

## Работа с RVIZ

### Запуск RVIZ

```bash
rosrun rviz rviz
```

### Запуск RVIZ с конфигурацией TurtleBot

```bash
roslaunch turtlebot3_bringup turtlebot3_model.launch
```

## Создание собственного ROS пакета

### 1. Создание пакета

```bash
cd /workspace/catkin_ws/src
catkin_create_pkg my_robot_package rospy roscpp std_msgs
```

### 2. Сборка пакета

```bash
cd /workspace/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Создание простого узла Python

Создайте файл `my_robot_package/scripts/my_node.py`:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

Сделайте файл исполняемым:

```bash
chmod +x my_robot_package/scripts/my_node.py
```

### 4. Запуск узла

```bash
rosrun my_robot_package my_node.py
```

## Полезные команды ROS

```bash
# Список всех узлов
rosnode list

# Информация о узле
rosnode info /node_name

# Список всех топиков
rostopic list

# Информация о топике
rostopic info /topic_name

# Тип сообщения топика
rostopic type /topic_name

# Публикация в топик
rostopic pub /topic_name std_msgs/String "data: 'Hello'"

# Список всех сервисов
rosservice list

# Вызов сервиса
rosservice call /service_name "request"

# Список всех параметров
rosparam list

# Получение параметра
rosparam get /parameter_name

# Установка параметра
rosparam set /parameter_name value
```

## Работа с catkin workspace

### Структура workspace

```
/workspace/catkin_ws/
├── src/              # Исходный код пакетов
├── build/            # Временные файлы сборки
├── devel/            # Разработанные файлы
└── install/          # Установленные файлы (опционально)
```

### Добавление пакета из GitHub

```bash
cd /workspace/catkin_ws/src
git clone https://github.com/ros/package_name.git
cd /workspace/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Отладка

### Просмотр логов ROS

```bash
# Включение отладочных сообщений
export ROSCONSOLE_CONFIG_FILE=/opt/ros/noetic/share/ros/config/rosconsole.config
```

### Проверка подключения к ROS Master

```bash
rostopic list
# Если выводит список топиков - подключение работает
```

### Проверка переменных окружения ROS

```bash
echo $ROS_MASTER_URI
echo $ROS_PACKAGE_PATH
```

## Интеграция с внешними системами

### Подключение к ROS Master на хосте

Если ROS Master запущен на хосте (не в контейнере):

```bash
export ROS_MASTER_URI=http://host.docker.internal:11311
```

Или измените в `docker-compose.yml`:

```yaml
environment:
  - ROS_MASTER_URI=http://host.docker.internal:11311
```

