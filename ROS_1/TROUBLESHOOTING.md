# Решение проблем с веб-интерфейсом

## Проблема: Не видно рабочего стола в браузере

### Шаг 1: Проверка статуса контейнера

**Windows (PowerShell):**
```powershell
.\check_status.ps1
```

**Linux/Mac:**
```bash
chmod +x check_status.sh
./check_status.sh
```

Или вручную:
```powershell
# Проверка, что контейнер запущен
docker ps | Select-String "ros_gazebo_rviz"

# Проверка статуса процессов
docker exec ros_gazebo_rviz supervisorctl status

# Проверка логов
docker exec ros_gazebo_rviz tail -n 50 /var/log/supervisor/supervisord.log
```

### Шаг 2: Проверка портов

```powershell
# Проверка, что порты открыты
docker exec ros_gazebo_rviz netstat -tlnp | Select-String -Pattern "6080|5900"
```

### Шаг 3: Проверка логов noVNC

```powershell
# Логи websockify
docker exec ros_gazebo_rviz cat /var/log/novnc.log
docker exec ros_gazebo_rviz cat /var/log/novnc_error.log

# Логи x11vnc
docker exec ros_gazebo_rviz cat /var/log/x11vnc.log
```

### Шаг 4: Проверка правильного URL

Попробуйте следующие URL:

1. **Основной (рекомендуется):**
   ```
   http://localhost:6080/vnc.html
   ```

2. **Альтернативный:**
   ```
   http://localhost:6080/vnc_lite.html
   ```

3. **Простой:**
   ```
   http://localhost:6080
   ```

### Шаг 5: Проверка процессов

```powershell
# Проверка Xvfb
docker exec ros_gazebo_rviz ps aux | Select-String "Xvfb"

# Проверка x11vnc
docker exec ros_gazebo_rviz ps aux | Select-String "x11vnc"

# Проверка websockify
docker exec ros_gazebo_rviz ps aux | Select-String "websockify"
```

### Шаг 6: Перезапуск сервисов

Если процессы не запущены:

```powershell
# Перезапуск всех сервисов
docker exec ros_gazebo_rviz supervisorctl restart all

# Или перезапуск конкретного сервиса
docker exec ros_gazebo_rviz supervisorctl restart novnc
docker exec ros_gazebo_rviz supervisorctl restart x11vnc
```

### Шаг 7: Проверка файлов noVNC

```powershell
# Проверка наличия файлов noVNC
docker exec ros_gazebo_rviz ls -la /opt/novnc/

# Проверка наличия vnc.html
docker exec ros_gazebo_rviz ls -la /opt/novnc/vnc.html
```

## Частые проблемы

### Проблема: "Connection refused" в браузере

**Решение:**
1. Убедитесь, что контейнер запущен: `docker ps`
2. Проверьте, что порт 6080 не занят: `netstat -an | findstr 6080`
3. Перезапустите контейнер: `docker-compose restart`

### Проблема: Черный экран в браузере

**Решение:**
1. Проверьте, что Xvfb запущен
2. Проверьте, что fluxbox запущен
3. Попробуйте перезапустить x11vnc: `docker exec ros_gazebo_rviz supervisorctl restart x11vnc`

### Проблема: "WebSocket connection failed"

**Решение:**
1. Проверьте логи websockify: `docker exec ros_gazebo_rviz cat /var/log/novnc_error.log`
2. Убедитесь, что x11vnc работает: `docker exec ros_gazebo_rviz supervisorctl status x11vnc`
3. Перезапустите novnc: `docker exec ros_gazebo_rviz supervisorctl restart novnc`

### Проблема: Страница загружается, но нет изображения

**Решение:**
1. Подождите 10-15 секунд после запуска контейнера
2. Обновите страницу (F5)
3. Попробуйте другой браузер
4. Проверьте консоль браузера (F12) на наличие ошибок

## Полная перезагрузка

Если ничего не помогает:

```powershell
# Остановка контейнера
docker-compose down

# Пересборка образа
docker-compose build

# Запуск заново
docker-compose up -d

# Подождите 15 секунд, затем откройте браузер
Start-Sleep -Seconds 15
Start-Process "http://localhost:6080/vnc.html"
```

## Проверка через терминал контейнера

Войдите в контейнер и проверьте вручную:

```powershell
docker exec -it ros_gazebo_rviz bash
```

Внутри контейнера:

```bash
# Проверка DISPLAY
echo $DISPLAY
# Должно быть: :1

# Проверка процессов
ps aux | grep -E "Xvfb|fluxbox|x11vnc|websockify"

# Проверка портов
netstat -tlnp | grep -E "6080|5900"

# Ручной запуск для тестирования
export DISPLAY=:1
Xvfb :1 -screen 0 1920x1080x24 &
fluxbox &
x11vnc -display :1 -nopw -listen localhost -xkb -forever -shared -rfbport 5900 &
cd /opt/novnc/utils && websockify --web=/opt/novnc 6080 localhost:5900 &
```

