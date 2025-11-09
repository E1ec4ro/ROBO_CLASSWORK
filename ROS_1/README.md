# ROS Docker Environment с Gazebo, RVIZ и веб-интерфейсом

Этот проект предоставляет полноценную среду разработки ROS (Robot Operating System) с Gazebo, RVIZ и веб-интерфейсом Ubuntu, работающую в Docker контейнере.

## Возможности

- ✅ ROS Noetic (полная версия desktop-full)
- ✅ Gazebo (симулятор роботов)
- ✅ RVIZ (визуализация данных ROS)
- ✅ Веб-интерфейс Ubuntu через браузер (noVNC)
- ✅ Полноценный рабочий стол XFCE
- ✅ Готовый catkin workspace

## Требования

- Docker (версия 20.10 или выше)
- Docker Compose (версия 1.29 или выше)
- Минимум 4GB свободной RAM
- Минимум 10GB свободного места на диске

## Быстрый старт

### Windows

В PowerShell выполните:

```powershell
# Сборка образа
docker-compose build

# Запуск контейнера
docker-compose up -d
```

### Linux/Mac

```bash
# Сборка образа
docker-compose build

# Или напрямую через Docker:
docker build -t ros-gazebo:latest .

# Запуск контейнера
docker-compose up -d
```

**Примечание:** Скрипты `start.sh` и `stop.sh` работают только в Linux/Mac. В Windows используйте команды `docker-compose` напрямую.

### 3. Доступ к веб-интерфейсу

Откройте браузер и перейдите по адресу:

```
http://localhost:6080/vnc.html
```

Или просто:

```
http://localhost:6080
```

### 4. Остановка контейнера

**Windows (PowerShell):**
```powershell
docker-compose down
```

**Linux/Mac:**
```bash
docker-compose down
```

Или используйте скрипты:
- Windows: `.\start.ps1` и `.\stop.ps1`
- Linux/Mac: `./start.sh` и `./stop.sh`

## Использование

### Доступ к контейнеру через терминал

```bash
docker exec -it ros_gazebo_rviz bash
```

### Запуск ROS команд

После входа в контейнер, все команды ROS доступны:

```bash
# Инициализация ROS master
roscore

# Запуск Gazebo
rosrun gazebo_ros gazebo

# Запуск RVIZ
rosrun rviz rviz

# Или через веб-интерфейс - просто откройте терминал в браузере
```

### Работа с catkin workspace

Рабочая директория находится в `/workspace/catkin_ws/src`. 

Файлы из локальной папки `workspace` автоматически монтируются в контейнер.

Создание нового пакета:

```bash
cd /workspace/catkin_ws/src
catkin_create_pkg my_package rospy roscpp std_msgs
cd /workspace/catkin_ws
catkin_make
source devel/setup.bash
```

## Структура проекта

```
ROS_1/
├── Dockerfile              # Образ Docker с ROS, Gazebo, RVIZ
├── docker-compose.yml      # Конфигурация Docker Compose
├── supervisord.conf        # Конфигурация supervisor для управления процессами
├── .dockerignore          # Исключения для Docker
├── .gitignore             # Исключения для Git
├── README.md              # Основная документация
├── QUICKSTART.md          # Быстрое руководство по первому запуску
├── EXAMPLES.md            # Примеры использования ROS
├── start.sh / start.ps1   # Скрипты запуска (Linux/Mac и Windows)
├── stop.sh / stop.ps1     # Скрипты остановки (Linux/Mac и Windows)
├── workspace/             # Ваши ROS пакеты (создается автоматически)
└── config/                # Конфигурационные файлы (создается автоматически)
```

## Дополнительная документация

- **[QUICKSTART.md](QUICKSTART.md)** - Быстрое руководство по первому запуску
- **[EXAMPLES.md](EXAMPLES.md)** - Примеры использования ROS, Gazebo и RVIZ

## Порты

- **6080** - noVNC веб-интерфейс (основной доступ)
- **5900** - VNC порт (для прямого подключения через VNC клиент)
- **11311** - ROS master
- **11312** - ROS дополнительный порт

## Решение проблем

### Контейнер не запускается

Проверьте логи:

```bash
docker-compose logs
```

### Веб-интерфейс не открывается

1. Убедитесь, что контейнер запущен: `docker ps`
2. Проверьте, что порт 6080 не занят другим приложением
3. Попробуйте пересобрать образ: `docker-compose build --no-cache`

### Gazebo не запускается

Gazebo требует GPU для некоторых функций. Если возникают проблемы:

1. Убедитесь, что установлены драйверы NVIDIA (если используете GPU)
2. Попробуйте запустить с флагом `--verbose` для диагностики

### Проблемы с правами доступа

Если возникают проблемы с записью файлов, проверьте права доступа к папке `workspace`.

## Дополнительные ресурсы

- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Gazebo Documentation](http://gazebosim.org/docs)
- [RVIZ Documentation](http://wiki.ros.org/rviz)

## Лицензия

Этот проект создан для образовательных целей.

