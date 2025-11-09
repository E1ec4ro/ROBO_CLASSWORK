#!/bin/bash
# Скрипт для исправления проблемы с роботом

echo "=== Проверка и исправление робота ==="

# Проверка ROS Master
if ! rostopic list &>/dev/null; then
    echo "❌ ROS Master не запущен!"
    echo "Запустите: roscore"
    exit 1
fi

echo "✅ ROS Master работает"

# Проверка Gazebo
if ! rosnode list | grep -q "gazebo"; then
    echo "❌ Gazebo не запущен!"
    echo "Запустите: roslaunch test_robot gazebo_world.launch"
    exit 1
fi

echo "✅ Gazebo запущен"

# Проверка модели робота
if ! rosparam get /robot_description &>/dev/null; then
    echo "❌ Модель робота не загружена!"
    echo "Загружаю модель..."
    source /opt/ros/noetic/setup.bash
    source /workspace/catkin_ws/devel/setup.bash
    rosparam set /robot_description "$(catkin_find test_robot urdf/simple_robot.urdf | xargs cat)"
fi

echo "✅ Модель робота загружена"

# Проверка спавна робота
if ! rostopic list | grep -q "/odom"; then
    echo "⚠️  Робот не заспавнен или плагин не работает"
    echo "Попытка заспавнить робота..."
    
    # Удаление старой модели (если есть)
    rosservice call /gazebo/delete_model "model_name: 'simple_robot'" 2>/dev/null
    
    # Заспавн робота
    rosrun gazebo_ros spawn_model -urdf \
        -model simple_robot \
        -param robot_description \
        -x 0 -y 0 -z 0.1
    
    echo "Подождите 3 секунды для загрузки плагина..."
    sleep 3
fi

# Проверка подписчиков
echo ""
echo "=== Проверка подписчиков на /cmd_vel ==="
rostopic info /cmd_vel | grep Subscribers

if rostopic info /cmd_vel | grep -q "Subscribers: None"; then
    echo ""
    echo "❌ Плагин все еще не подключен!"
    echo ""
    echo "Попробуйте:"
    echo "1. Перезапустить Gazebo:"
    echo "   pkill -f gazebo"
    echo "   roslaunch test_robot gazebo_world.launch"
    echo ""
    echo "2. Проверить логи Gazebo на наличие ошибок плагина"
    echo "3. Убедиться, что пакет gazebo_ros установлен:"
    echo "   rospack find gazebo_ros"
else
    echo "✅ Плагин подключен!"
    echo ""
    echo "Теперь можно отправлять команды:"
    echo "rostopic pub /cmd_vel geometry_msgs/Twist \"linear: {x: 0.5}\""
fi

