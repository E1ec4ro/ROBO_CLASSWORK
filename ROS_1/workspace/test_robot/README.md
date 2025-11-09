# Тестовый робот для Gazebo

Простой пример запуска робота в Gazebo.

## Запуск

### Вариант 1: Через веб-интерфейс

1. Откройте браузер: `http://localhost:6080/vnc.html`
2. Откройте терминал в веб-интерфейсе
3. Выполните:

```bash
# Инициализация ROS
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Запуск ROS master
roscore
```

4. В новом терминале:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Запуск Gazebo с пустым миром
roslaunch test_robot gazebo_world.launch
```

### Вариант 2: Через терминал хоста

```powershell
# Вход в контейнер
docker exec -it ros_gazebo_rviz bash

# Внутри контейнера
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Запуск ROS master
roscore
```

В другом терминале:

```powershell
docker exec -it ros_gazebo_rviz bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Запуск Gazebo
roslaunch test_robot gazebo_world.launch
```

## Структура пакета

```
test_robot/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── gazebo_world.launch
├── urdf/
│   └── simple_robot.urdf
└── README.md
```

