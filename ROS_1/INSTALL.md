# Инструкция по установке и запуску

## Для новых пользователей

Этот проект содержит все необходимые файлы для запуска ROS Docker контейнера с Gazebo и тестовым роботом.

## Что включено в репозиторий

✅ **Все необходимые файлы:**
- `Dockerfile` - образ Docker
- `docker-compose.yml` - конфигурация Docker Compose
- `supervisord.conf` - конфигурация процессов
- `workspace/test_robot/` - тестовый ROS пакет с роботом
- Все скрипты запуска и документация

## Быстрый старт

### 1. Клонирование репозитория

```bash
git clone <repository_url>
cd ROS_1
```

### 2. Сборка образа

```bash
docker-compose build
```

**Время сборки:** ~15-30 минут (первый раз)

### 3. Запуск контейнера

```bash
docker-compose up -d
```

### 4. Доступ к веб-интерфейсу

Откройте браузер: `http://localhost:6080/vnc.html`

### 5. Запуск Gazebo с роботом

В веб-интерфейсе откройте терминал и выполните:

```bash
cd /workspace/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch test_robot gazebo_world.launch
```

### 6. Управление роботом

В новом терминале:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Движение вперед
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"
```

## Структура проекта

```
ROS_1/
├── Dockerfile              # Образ Docker
├── docker-compose.yml      # Docker Compose конфигурация
├── supervisord.conf        # Конфигурация процессов
├── workspace/              # ROS пакеты (включено в репозиторий)
│   └── test_robot/         # Тестовый робот
│       ├── urdf/           # Модель робота
│       ├── launch/         # Launch файлы
│       └── scripts/        # Скрипты управления
├── README.md               # Основная документация
├── QUICKSTART.md           # Быстрый старт
└── GAZEBO_QUICKSTART.md    # Работа с Gazebo
```

## Требования

- Docker (версия 20.10+)
- Docker Compose (версия 1.29+)
- Минимум 4GB RAM
- Минимум 10GB свободного места

## Дополнительная документация

- [README.md](README.md) - полная документация
- [QUICKSTART.md](QUICKSTART.md) - быстрый старт
- [GAZEBO_QUICKSTART.md](GAZEBO_QUICKSTART.md) - работа с Gazebo
- [EXAMPLES.md](EXAMPLES.md) - примеры использования ROS

## Решение проблем

Если возникают проблемы, см.:
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - решение проблем с веб-интерфейсом
- [workspace/test_robot/TROUBLESHOOTING.md](workspace/test_robot/TROUBLESHOOTING.md) - решение проблем с роботом

## Важно

Все необходимые файлы включены в репозиторий. После клонирования проект готов к использованию без дополнительных настроек.

