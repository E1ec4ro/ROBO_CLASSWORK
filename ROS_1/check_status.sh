#!/bin/bash
# Скрипт для проверки статуса сервисов в контейнере

echo "=== Проверка статуса контейнера ==="
docker ps | grep ros_gazebo_rviz

echo ""
echo "=== Проверка логов supervisor ==="
docker exec ros_gazebo_rviz tail -n 20 /var/log/supervisor/supervisord.log

echo ""
echo "=== Проверка статуса процессов ==="
docker exec ros_gazebo_rviz supervisorctl status

echo ""
echo "=== Проверка портов ==="
docker exec ros_gazebo_rviz netstat -tlnp | grep -E "6080|5900"

echo ""
echo "=== Проверка Xvfb ==="
docker exec ros_gazebo_rviz ps aux | grep Xvfb

echo ""
echo "=== Проверка x11vnc ==="
docker exec ros_gazebo_rviz ps aux | grep x11vnc

echo ""
echo "=== Проверка websockify ==="
docker exec ros_gazebo_rviz ps aux | grep websockify

echo ""
echo "=== Проверка noVNC файлов ==="
docker exec ros_gazebo_rviz ls -la /opt/novnc/ | head -10

