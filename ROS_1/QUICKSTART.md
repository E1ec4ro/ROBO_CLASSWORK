# Быстрый старт - ROS Docker Environment

## Первый запуск (5 минут)

### Шаг 1: Установка Docker

**Windows:**
1. Скачайте [Docker Desktop для Windows](https://www.docker.com/products/docker-desktop)
2. Установите и перезагрузите компьютер
3. Запустите Docker Desktop

**Linux:**
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install docker.io docker-compose
sudo usermod -aG docker $USER
# Выйдите и войдите снова
```

### Шаг 2: Клонирование/Переход в папку проекта

```bash
cd ROS_1
```

### Шаг 3: Сборка образа (первый раз, ~15-30 минут)

**Windows (PowerShell):**
```powershell
docker-compose build
```

**Linux/Mac:**
```bash
docker-compose build
```

⚠️ **Важно:** Первая сборка займет 15-30 минут, так как загружается полный образ ROS Noetic (~3GB).

### Шаг 4: Запуск контейнера

**Windows (PowerShell):**
```powershell
docker-compose up -d
```

**Linux/Mac:**
```bash
docker-compose up -d
```

### Шаг 5: Открытие веб-интерфейса

Подождите 10-15 секунд после запуска, затем откройте браузер:

```
http://localhost:6080/vnc.html
```

Вы увидите рабочий стол Ubuntu с Fluxbox.

### Шаг 6: Первый запуск ROS

В веб-интерфейсе откройте терминал (правый клик на рабочем столе → Terminal или через меню).

Выполните:

```bash
# Инициализация ROS master
roscore
```

В новом терминале:

```bash
# Запуск Gazebo
rosrun gazebo_ros gazebo
```

В третьем терминале:

```bash
# Запуск RVIZ
rosrun rviz rviz
```

## Проверка установки

Выполните в терминале контейнера:

```bash
# Проверка ROS
echo $ROS_DISTRO
# Должно вывести: noetic

# Проверка Gazebo
gazebo --version

# Проверка RVIZ
rviz --version
```

## Остановка

```bash
docker-compose down
```

## Полезные команды

```bash
# Просмотр логов
docker-compose logs -f

# Перезапуск контейнера
docker-compose restart

# Вход в контейнер через терминал хоста
docker exec -it ros_gazebo_rviz bash

# Очистка (удаление контейнера и образа)
docker-compose down
docker rmi ros-gazebo:latest
```

## Решение проблем

### Порт 6080 занят

Измените порт в `docker-compose.yml`:
```yaml
ports:
  - "6081:6080"  # Используйте другой порт
```

### Контейнер не запускается

```bash
# Проверьте логи
docker-compose logs

# Проверьте статус
docker ps -a
```

### Медленная работа веб-интерфейса

Это нормально для первого запуска. После полной загрузки интерфейс будет работать быстрее.

### Gazebo не запускается

Gazebo может требовать больше ресурсов. Убедитесь, что у вас достаточно RAM (минимум 4GB свободно).

