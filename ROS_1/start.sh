#!/bin/bash

# Скрипт для запуска ROS Docker контейнера

echo "=== ROS Docker Environment ==="
echo ""

# Проверка наличия Docker
if ! command -v docker &> /dev/null; then
    echo "Ошибка: Docker не установлен!"
    exit 1
fi

# Проверка наличия Docker Compose
if ! command -v docker-compose &> /dev/null; then
    echo "Ошибка: Docker Compose не установлен!"
    exit 1
fi

# Создание необходимых директорий
echo "Создание рабочих директорий..."
mkdir -p workspace
mkdir -p config

# Сборка образа (если нужно)
echo ""
read -p "Пересобрать образ? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Сборка Docker образа..."
    docker-compose build
fi

# Запуск контейнера
echo ""
echo "Запуск контейнера..."
docker-compose up -d

# Ожидание запуска
echo ""
echo "Ожидание запуска сервисов..."
sleep 5

# Проверка статуса
if docker ps | grep -q ros_gazebo_rviz; then
    echo ""
    echo "✅ Контейнер успешно запущен!"
    echo ""
    echo "🌐 Откройте браузер и перейдите по адресу:"
    echo "   http://localhost:6080/vnc.html"
    echo ""
    echo "📝 Для доступа к терминалу контейнера:"
    echo "   docker exec -it ros_gazebo_rviz bash"
    echo ""
    echo "🛑 Для остановки контейнера:"
    echo "   docker-compose down"
else
    echo ""
    echo "❌ Ошибка при запуске контейнера!"
    echo "Проверьте логи: docker-compose logs"
fi

