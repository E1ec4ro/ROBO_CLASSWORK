#!/bin/bash
# Полная диагностика проблемы с роботом

echo "=== ДИАГНОСТИКА РОБОТА ==="
echo ""

# 1. Проверка ROS Master
echo "1. Проверка ROS Master..."
if rostopic list &>/dev/null; then
    echo "   ✅ ROS Master работает"
else
    echo "   ❌ ROS Master НЕ работает!"
    echo "   Запустите: roscore"
    exit 1
fi

# 2. Проверка Gazebo
echo ""
echo "2. Проверка Gazebo..."
if rosnode list | grep -q "gazebo"; then
    echo "   ✅ Gazebo запущен"
    GAZEBO_RUNNING=true
else
    echo "   ❌ Gazebo НЕ запущен!"
    echo "   Запустите: roslaunch test_robot gazebo_world.launch"
    GAZEBO_RUNNING=false
fi

# 3. Проверка модели робота
echo ""
echo "3. Проверка модели робота..."
if rosparam get /robot_description &>/dev/null; then
    echo "   ✅ Модель робота загружена"
    ROBOT_LOADED=true
else
    echo "   ❌ Модель робота НЕ загружена!"
    ROBOT_LOADED=false
fi

# 4. Проверка спавна робота
echo ""
echo "4. Проверка спавна робота в Gazebo..."
if rostopic list | grep -q "/odom"; then
    echo "   ✅ Робот заспавнен (есть топик /odom)"
    ROBOT_SPAWNED=true
else
    echo "   ❌ Робот НЕ заспавнен или плагин не работает!"
    echo "   Нет топика /odom"
    ROBOT_SPAWNED=false
fi

# 5. Проверка топика /cmd_vel
echo ""
echo "5. Проверка топика /cmd_vel..."
if rostopic list | grep -q "/cmd_vel"; then
    echo "   ✅ Топик /cmd_vel существует"
    SUBSCRIBERS=$(rostopic info /cmd_vel 2>/dev/null | grep "Subscribers:" | awk '{print $2}')
    if [ "$SUBSCRIBERS" = "None" ]; then
        echo "   ❌ НЕТ ПОДПИСЧИКОВ на /cmd_vel!"
        echo "   Плагин дифференциального привода не подключен"
        CMD_VEL_OK=false
    else
        echo "   ✅ Есть подписчики: $SUBSCRIBERS"
        CMD_VEL_OK=true
    fi
else
    echo "   ❌ Топик /cmd_vel не существует"
    CMD_VEL_OK=false
fi

# 6. Проверка узлов
echo ""
echo "6. Активные узлы ROS:"
rosnode list

# 7. Проверка топиков
echo ""
echo "7. Активные топики:"
rostopic list

# 8. Рекомендации
echo ""
echo "=== РЕКОМЕНДАЦИИ ==="
if [ "$GAZEBO_RUNNING" = false ]; then
    echo "❌ Запустите Gazebo:"
    echo "   roslaunch test_robot gazebo_world.launch"
    echo ""
fi

if [ "$ROBOT_LOADED" = false ]; then
    echo "❌ Загрузите модель робота:"
    echo "   source /opt/ros/noetic/setup.bash"
    echo "   source /workspace/catkin_ws/devel/setup.bash"
    echo "   rosparam set /robot_description \"\$(cat /workspace/catkin_ws/src/test_robot/urdf/simple_robot.urdf)\""
    echo ""
fi

if [ "$ROBOT_SPAWNED" = false ] && [ "$GAZEBO_RUNNING" = true ]; then
    echo "❌ Заспавньте робота:"
    echo "   rosrun gazebo_ros spawn_model -urdf -model simple_robot -param robot_description -x 0 -y 0 -z 0.1"
    echo ""
fi

if [ "$CMD_VEL_OK" = false ] && [ "$GAZEBO_RUNNING" = true ]; then
    echo "❌ Плагин не подключен. Попробуйте:"
    echo "   1. Перезапустить Gazebo:"
    echo "      pkill -f gazebo"
    echo "      roslaunch test_robot gazebo_world.launch"
    echo "   2. Подождать 10-15 секунд после запуска"
    echo "   3. Проверить логи Gazebo на ошибки плагина"
    echo ""
fi