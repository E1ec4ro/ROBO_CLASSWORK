# Контроллер удаленного управления UR10e

Этот контроллер поддерживает **4 способа** отправки команд роботу:

## 🎮 Способы управления

### 1. Консоль Webots
- Откройте окно Controller Console в Webots
- Введите команду и нажмите Enter

### 2. TCP/IP сервер (порт 10001)
Отправляйте команды через TCP сокет.

**Пример на Python:**
```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 10001))
sock.sendall(b'vel 1 0.5\n')
response = sock.recv(1024).decode('utf-8')
result = json.loads(response)
print(result)
sock.close()
```

**Пример на командной строке (Linux/Mac):**
```bash
echo "vel 1 0.5" | nc localhost 10001
```

**Пример на Windows (PowerShell):**
```powershell
$client = New-Object System.Net.Sockets.TcpClient("localhost", 10001)
$stream = $client.GetStream()
$data = [System.Text.Encoding]::UTF8.GetBytes("vel 1 0.5`n")
$stream.Write($data, 0, $data.Length)
$buffer = New-Object byte[] 1024
$bytesRead = $stream.Read($buffer, 0, 1024)
$response = [System.Text.Encoding]::UTF8.GetString($buffer, 0, $bytesRead)
Write-Host $response
$client.Close()
```

### 3. HTTP REST API (порт 10002)

#### Веб-интерфейс
Откройте в браузере: **http://localhost:10002/**

#### REST API endpoints

**GET /command?cmd=<команда>**
- Отправка команды через HTTP GET
- Пример: `http://localhost:10002/command?cmd=vel%201%200.5`

**GET /status**
- Получить текущий статус робота
- Пример: `http://localhost:10002/status`

**Пример на Python:**
```python
import requests

# Отправить команду
response = requests.get('http://localhost:10002/command?cmd=vel%201%200.5')
print(response.json())

# Получить статус
response = requests.get('http://localhost:10002/status')
print(response.json())
```

**Пример на JavaScript (браузер):**
```javascript
fetch('http://localhost:10002/command?cmd=vel%201%200.5')
    .then(r => r.json())
    .then(data => console.log(data));
```

**Пример с curl:**
```bash
# Отправить команду
curl "http://localhost:10002/command?cmd=vel%201%200.5"

# Получить статус
curl "http://localhost:10002/status"
```

### 4. Файл commands.txt
Создайте файл `commands.txt` в папке контроллера и запишите команду (одна на строку).

## 📋 Доступные команды

### Базовые
- `help` - показать справку
- `stop` - остановить все моторы
- `status` - показать состояние всех суставов
- `home` - вернуть робота в исходное положение

### Управление скоростью
- `vel <j> <v>` - установить скорость сустава j (1-6) в рад/с
  - Пример: `vel 1 0.5`
- `vel_all <v1> <v2> <v3> <v4> <v5> <v6>` - установить скорости всех суставов
  - Пример: `vel_all 0.5 0.3 0.4 0.2 0.3 0.25`

### Управление позицией
- `pos <j> <p>` - установить позицию сустава j в радианах
  - Пример: `pos 1 1.57` (90 градусов)
- `pos_all <p1> <p2> <p3> <p4> <p5> <p6>` - установить позиции всех суставов
  - Пример: `pos_all 0 0.5 0 0 0 0`

## 🔧 Использование

### 1. Подключение контроллера
В файле мира (.wbt) укажите:
```
UR10e {
  controller "ur10e_remote_control"
  ...
}
```

### 2. Запуск симуляции
Запустите симуляцию в Webots. Контроллер автоматически запустит:
- TCP сервер на порту **10001**
- HTTP сервер на порту **10002**

### 3. Отправка команд
Используйте любой из 4 способов для отправки команд.

## 📡 Формат ответов

Все команды возвращают JSON ответ:

**Успех:**
```json
{
  "status": "ok",
  "command": "vel",
  "message": "Сустав 1: скорость = 0.500 рад/с"
}
```

**Ошибка:**
```json
{
  "status": "error",
  "message": "Номер сустава должен быть от 1 до 6"
}
```

**Статус:**
```json
{
  "status": "ok",
  "command": "status",
  "data": {
    "motors": [
      {
        "joint": 1,
        "name": "shoulder_pan_joint",
        "mode": "velocity",
        "velocity": 0.5,
        "target_position": null
      },
      ...
    ],
    "time": 12.345
  }
}
```

## 🛠️ Примеры интеграции

### Python скрипт для автоматизации
```python
import socket
import time

def control_robot(command):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', 10001))
    sock.sendall((command + '\n').encode('utf-8'))
    response = sock.recv(1024).decode('utf-8')
    sock.close()
    return response

# Последовательность движений
control_robot('home')
time.sleep(2)
control_robot('vel 1 0.5')
time.sleep(3)
control_robot('stop')
```

### Node.js клиент
```javascript
const net = require('net');

function sendCommand(command) {
    return new Promise((resolve, reject) => {
        const client = new net.Socket();
        client.connect(10001, 'localhost', () => {
            client.write(command + '\n');
        });
        client.on('data', (data) => {
            resolve(JSON.parse(data.toString()));
            client.destroy();
        });
        client.on('error', reject);
    });
}

// Использование
sendCommand('vel 1 0.5').then(result => console.log(result));
```

## ⚠️ Примечания

- TCP и HTTP серверы работают только на `localhost` (безопасность)
- Для доступа с других компьютеров измените `'localhost'` на `'0.0.0.0'` в коде
- Порты 10001 и 10002 должны быть свободны
- Все команды выполняются асинхронно в отдельном потоке

