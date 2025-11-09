#!/bin/bash
# Скрипт для проверки состояния робота

echo "=== Проверка ROS Master ==="
if rostopic list &>/dev/null; then
    echo "✅ ROS Master работает"
else
    echo "❌ ROS Master не запущен! Запустите: roscore"
    exit 1
fi

echo ""
echo "=== Проверка топиков ==="
echo "Активные топики:"
rostopic list

echo ""
echo "=== Проверка топика /cmd_vel ==="
if rostopic list | grep -q "/cmd_vel"; then
    echo "✅ Топик /cmd_vel существует"
    echo "Подписчики:"
    rostopic info /cmd_vel | grep "Subscribers:"
else
    echo "❌ Топик /cmd_vel не найден"
fi

echo ""
echo "=== Проверка узлов ==="
echo "Активные узлы:"
rosnode list

echo ""
echo "=== Проверка модели робота ==="
if rosparam get /robot_description &>/dev/null; then
    echo "✅ Модель робота загружена"
else
    echo "❌ Модель робота не загружена"
fi

echo ""
echo "=== Проверка Gazebo ==="
if rosnode list | grep -q "gazebo"; then
    echo "✅ Gazebo запущен"
else
    echo "❌ Gazebo не запущен"
fi

echo ""
echo "=== Тест отправки команды ==="
echo "Отправка тестовой команды..."
timeout 1 rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -1 &>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Команда отправлена успешно"
else
    echo "❌ Ошибка при отправке команды"
fi

