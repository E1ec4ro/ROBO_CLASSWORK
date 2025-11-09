"""
Контроллер для управления роботом UR10e через различные интерфейсы
Поддерживает: консоль, TCP/IP, HTTP REST API
"""

from controller import Robot
import threading
import queue
import socket
import json
import time
import sys
try:
    from urllib.parse import unquote
except ImportError:
    # Python 2 compatibility
    from urllib import unquote

# Создаем экземпляр робота
robot = Robot()

# Получаем временной шаг симуляции
timeStep = int(robot.getBasicTimeStep())

# Очередь для команд
command_queue = queue.Queue()

# Имена суставов UR10e
joint_name_variants = [
    [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ],
    [
        'UR10e::shoulder_pan_joint',
        'UR10e::shoulder_lift_joint',
        'UR10e::elbow_joint',
        'UR10e::wrist_1_joint',
        'UR10e::wrist_2_joint',
        'UR10e::wrist_3_joint'
    ]
]

# Получаем моторы
motors = []
joint_names_used = []

for variant in joint_name_variants:
    motors = []
    joint_names_used = []
    found_count = 0
    
    for joint_name in variant:
        motor = robot.getDevice(joint_name)
        if motor:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            motors.append(motor)
            joint_names_used.append(joint_name)
            found_count += 1
        else:
            motors.append(None)
            joint_names_used.append(joint_name)
    
    if found_count > 0:
        break

if len([m for m in motors if m is not None]) == 0:
    motors = []
    joint_names_used = []
    num_devices = robot.getNumberOfDevices()
    for i in range(num_devices):
        device = robot.getDeviceByIndex(i)
        if device and device.getNodeType() == device.MOTOR:
            device.setPosition(float('inf'))
            device.setVelocity(0.0)
            motors.append(device)
            joint_names_used.append(device.getName())

num_motors = len([m for m in motors if m is not None])

# Текущие скорости и позиции
current_velocities = [0.0] * num_motors
target_positions = [None] * num_motors
position_mode = [False] * num_motors

# Настройки серверов
TCP_PORT = 10001
HTTP_PORT = 10002

def process_command(cmd):
    """Обрабатывает команду"""
    global current_velocities, target_positions, position_mode
    
    parts = cmd.strip().split()
    if not parts:
        return {"status": "error", "message": "Пустая команда"}
    
    command = parts[0].lower()
    result = {"status": "ok", "command": command}
    
    try:
        if command == 'help':
            help_text = """
Доступные команды:
  help              - показать справку
  stop              - остановить все моторы
  vel <j> <v>       - установить скорость сустава j (1-6) в рад/с
  vel_all <v1> ... <v6> - установить скорости всех суставов
  pos <j> <p>       - установить позицию сустава j в радианах
  pos_all <p1> ... <p6> - установить позиции всех суставов
  home              - вернуть робота в исходное положение
  status            - показать текущее состояние суставов
"""
            result["message"] = help_text
            print(help_text)
        
        elif command == 'stop':
            current_velocities = [0.0] * num_motors
            target_positions = [None] * num_motors
            position_mode = [False] * num_motors
            motor_index = 0
            for motor in motors:
                if motor is not None:
                    motor.setVelocity(0.0)
                    motor_index += 1
            result["message"] = "Все моторы остановлены"
            print(result["message"])
        
        elif command == 'vel':
            if len(parts) < 3:
                result = {"status": "error", "message": "Используйте: vel <номер_сустава> <скорость>"}
                return result
            joint_num = int(parts[1]) - 1
            velocity = float(parts[2])
            if 0 <= joint_num < num_motors:
                current_velocities[joint_num] = velocity
                target_positions[joint_num] = None
                position_mode[joint_num] = False
                motor_index = 0
                for i, motor in enumerate(motors):
                    if motor is not None:
                        if motor_index == joint_num:
                            motor.setVelocity(velocity)
                            result["message"] = f"Сустав {joint_num+1}: скорость = {velocity:.3f} рад/с"
                            print(result["message"])
                            break
                        motor_index += 1
            else:
                result = {"status": "error", "message": f"Номер сустава должен быть от 1 до {num_motors}"}
                return result
        
        elif command == 'vel_all':
            if len(parts) < num_motors + 1:
                result = {"status": "error", "message": f"Нужно указать {num_motors} скоростей"}
                return result
            velocities = [float(v) for v in parts[1:num_motors+1]]
            current_velocities = velocities
            target_positions = [None] * num_motors
            position_mode = [False] * num_motors
            motor_index = 0
            for motor in motors:
                if motor is not None:
                    motor.setVelocity(velocities[motor_index])
                    motor_index += 1
            result["message"] = f"Установлены скорости: {velocities}"
            print(result["message"])
        
        elif command == 'pos':
            if len(parts) < 3:
                result = {"status": "error", "message": "Используйте: pos <номер_сустава> <позиция>"}
                return result
            joint_num = int(parts[1]) - 1
            position = float(parts[2])
            if 0 <= joint_num < num_motors:
                target_positions[joint_num] = position
                position_mode[joint_num] = True
                current_velocities[joint_num] = 0.0
                motor_index = 0
                for i, motor in enumerate(motors):
                    if motor is not None:
                        if motor_index == joint_num:
                            motor.setPosition(position)
                            motor.setVelocity(1.0)
                            result["message"] = f"Сустав {joint_num+1}: позиция = {position:.3f} рад"
                            print(result["message"])
                            break
                        motor_index += 1
            else:
                result = {"status": "error", "message": f"Номер сустава должен быть от 1 до {num_motors}"}
                return result
        
        elif command == 'pos_all':
            if len(parts) < num_motors + 1:
                result = {"status": "error", "message": f"Нужно указать {num_motors} позиций"}
                return result
            positions = [float(p) for p in parts[1:num_motors+1]]
            target_positions = positions
            position_mode = [True] * num_motors
            current_velocities = [0.0] * num_motors
            motor_index = 0
            for motor in motors:
                if motor is not None:
                    motor.setPosition(positions[motor_index])
                    motor.setVelocity(1.0)
                    motor_index += 1
            result["message"] = f"Установлены позиции: {positions}"
            print(result["message"])
        
        elif command == 'home':
            home_positions = [0.0] * num_motors
            target_positions = home_positions
            position_mode = [True] * num_motors
            current_velocities = [0.0] * num_motors
            motor_index = 0
            for motor in motors:
                if motor is not None:
                    motor.setPosition(home_positions[motor_index])
                    motor.setVelocity(0.5)
                    motor_index += 1
            result["message"] = "Робот возвращается в исходное положение..."
            print(result["message"])
        
        elif command == 'status':
            status_data = {
                "motors": [],
                "time": robot.getTime()
            }
            motor_index = 0
            for i, motor in enumerate(motors):
                if motor is not None:
                    joint_name = joint_names_used[i] if i < len(joint_names_used) else f"Сустав {motor_index+1}"
                    motor_status = {
                        "joint": motor_index + 1,
                        "name": joint_name,
                        "mode": "position" if position_mode[motor_index] else "velocity",
                        "velocity": current_velocities[motor_index],
                        "target_position": target_positions[motor_index] if target_positions[motor_index] is not None else None
                    }
                    status_data["motors"].append(motor_status)
                    motor_index += 1
            result["data"] = status_data
            result["message"] = "Статус получен"
            print(f"Статус: {json.dumps(status_data, indent=2)}")
        
        else:
            result = {"status": "error", "message": f"Неизвестная команда: {command}"}
            return result
    
    except ValueError as e:
        result = {"status": "error", "message": f"Неверный формат числа: {e}"}
        return result
    except Exception as e:
        result = {"status": "error", "message": f"Ошибка: {e}"}
        return result
    
    return result

# TCP/IP сервер
def tcp_server():
    """TCP/IP сервер для приема команд"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('localhost', TCP_PORT))
        sock.listen(5)
        print(f"✅ TCP сервер запущен на порту {TCP_PORT}")
        sys.stdout.flush()  # Принудительно выводим сообщение
        
        while True:
            try:
                conn, addr = sock.accept()
                print(f"TCP подключение от {addr}")
                sys.stdout.flush()
                threading.Thread(target=handle_tcp_client, args=(conn,), daemon=True).start()
            except Exception as e:
                print(f"Ошибка при принятии TCP подключения: {e}")
                sys.stdout.flush()
                break
    except Exception as e:
        print(f"❌ ОШИБКА: Не удалось запустить TCP сервер на порту {TCP_PORT}: {e}")
        print(f"   Возможно, порт занят или нет прав доступа")
        sys.stdout.flush()

def handle_tcp_client(conn):
    """Обработка TCP клиента"""
    try:
        while True:
            data = conn.recv(1024).decode('utf-8')
            if not data:
                break
            cmd = data.strip()
            if cmd:
                result = process_command(cmd)
                response = json.dumps(result) + "\n"
                conn.sendall(response.encode('utf-8'))
    except:
        pass
    finally:
        conn.close()

# HTTP REST API сервер
def http_server():
    """HTTP REST API сервер"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('localhost', HTTP_PORT))
        sock.listen(5)
        print(f"✅ HTTP сервер запущен на порту {HTTP_PORT}")
        print(f"🌐 Откройте в браузере: http://localhost:{HTTP_PORT}/")
        sys.stdout.flush()  # Принудительно выводим сообщение
        
        while True:
            try:
                conn, addr = sock.accept()
                threading.Thread(target=handle_http_client, args=(conn,), daemon=True).start()
            except Exception as e:
                print(f"Ошибка при принятии HTTP подключения: {e}")
                sys.stdout.flush()
                break
    except Exception as e:
        print(f"❌ ОШИБКА: Не удалось запустить HTTP сервер на порту {HTTP_PORT}: {e}")
        print(f"   Возможно, порт занят или нет прав доступа")
        sys.stdout.flush()

def handle_http_client(conn):
    """Обработка HTTP запроса"""
    try:
        request = conn.recv(1024).decode('utf-8')
        if not request:
            return
        
        lines = request.split('\n')
        if not lines:
            return
        
        request_line = lines[0]
        parts = request_line.split()
        if len(parts) < 2:
            return
        
        method = parts[0]
        path = parts[1]
        
        # Главная страница
        if path == '/' or path == '/index.html':
            html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>UR10e Remote Control</title>
    <meta charset="utf-8">
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }}
        .container {{ max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }}
        h1 {{ color: #333; }}
        .command {{ margin: 10px 0; padding: 10px; background: #f0f0f0; border-radius: 5px; }}
        input, button {{ padding: 8px; margin: 5px; }}
        button {{ background: #4CAF50; color: white; border: none; cursor: pointer; }}
        button:hover {{ background: #45a049; }}
        .result {{ margin: 10px 0; padding: 10px; background: #e8f5e9; border-radius: 5px; }}
        .error {{ background: #ffebee; color: #c62828; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Управление UR10e</h1>
        <p>HTTP REST API на порту {HTTP_PORT}</p>
        
        <div class="command">
            <h3>Быстрые команды:</h3>
            <button onclick="sendCommand('home')">Home</button>
            <button onclick="sendCommand('stop')">Stop</button>
            <button onclick="sendCommand('status')">Status</button>
        </div>
        
        <div class="command">
            <h3>Управление скоростью:</h3>
            Сустав: <input type="number" id="joint" value="1" min="1" max="6" style="width: 60px;">
            Скорость: <input type="number" id="velocity" value="0.5" step="0.1" style="width: 100px;">
            <button onclick="sendVel()">Установить скорость</button>
        </div>
        
        <div class="command">
            <h3>Управление позицией:</h3>
            Сустав: <input type="number" id="joint2" value="1" min="1" max="6" style="width: 60px;">
            Позиция (рад): <input type="number" id="position" value="0" step="0.1" style="width: 100px;">
            <button onclick="sendPos()">Установить позицию</button>
        </div>
        
        <div class="command">
            <h3>Произвольная команда:</h3>
            <input type="text" id="cmd" placeholder="например: vel_all 0.5 0.3 0.4 0.2 0.3 0.25" style="width: 400px;">
            <button onclick="sendCustom()">Отправить</button>
        </div>
        
        <div id="result"></div>
    </div>
    
    <script>
        function sendCommand(cmd) {{
            fetch('/command?cmd=' + encodeURIComponent(cmd))
                .then(r => r.json())
                .then(data => showResult(data))
                .catch(e => showResult({{status: 'error', message: e}}));
        }}
        
        function sendVel() {{
            const j = document.getElementById('joint').value;
            const v = document.getElementById('velocity').value;
            sendCommand('vel ' + j + ' ' + v);
        }}
        
        function sendPos() {{
            const j = document.getElementById('joint2').value;
            const p = document.getElementById('position').value;
            sendCommand('pos ' + j + ' ' + p);
        }}
        
        function sendCustom() {{
            const cmd = document.getElementById('cmd').value;
            sendCommand(cmd);
        }}
        
        function showResult(data) {{
            const div = document.getElementById('result');
            div.className = 'result ' + (data.status === 'error' ? 'error' : '');
            div.innerHTML = '<strong>Результат:</strong><pre>' + JSON.stringify(data, null, 2) + '</pre>';
        }}
    </script>
</body>
</html>
"""
            response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {len(html)}\r\n\r\n{html}"
            conn.sendall(response.encode('utf-8'))
        
        # API endpoint для команд
        elif path.startswith('/command'):
            # Извлекаем команду из query string
            cmd = ""
            if '?' in path:
                query = path.split('?')[1]
                params = query.split('&')
                for param in params:
                    if param.startswith('cmd='):
                        cmd = param.split('=', 1)[1]
                        cmd = unquote(cmd)
            
            if cmd:
                result = process_command(cmd)
            else:
                result = {"status": "error", "message": "Команда не указана"}
            
            response_json = json.dumps(result, ensure_ascii=False)
            response = f"HTTP/1.1 200 OK\r\nContent-Type: application/json; charset=utf-8\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {len(response_json)}\r\n\r\n{response_json}"
            conn.sendall(response.encode('utf-8'))
        
        # API endpoint для статуса
        elif path == '/status':
            result = process_command('status')
            response_json = json.dumps(result, ensure_ascii=False)
            response = f"HTTP/1.1 200 OK\r\nContent-Type: application/json; charset=utf-8\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {len(response_json)}\r\n\r\n{response_json}"
            conn.sendall(response.encode('utf-8'))
        
        else:
            response = "HTTP/1.1 404 Not Found\r\n\r\n"
            conn.sendall(response.encode('utf-8'))
    
    except Exception as e:
        print(f"HTTP ошибка: {e}")
    finally:
        conn.close()

# Консольный ввод
def read_console():
    """Читает команды из консоли"""
    import sys
    try:
        import select
        has_select = True
    except ImportError:
        has_select = False
    
    while True:
        try:
            if has_select and sys.platform != 'win32':
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    line = sys.stdin.readline()
                    if line:
                        process_command(line.strip())
            else:
                line = input()
                if line:
                    process_command(line.strip())
        except (EOFError, KeyboardInterrupt):
            break
        except:
            import time
            time.sleep(0.1)

# Запуск серверов
print("="*60)
print("КОНТРОЛЛЕР UR10e - УДАЛЕННОЕ УПРАВЛЕНИЕ")
print("="*60)
print(f"Найдено моторов: {num_motors}")
sys.stdout.flush()

# Запускаем серверы в отдельных потоках
print("\nЗапуск серверов...")
sys.stdout.flush()

tcp_thread = threading.Thread(target=tcp_server, daemon=True)
tcp_thread.start()

# Небольшая задержка для запуска TCP сервера
time.sleep(0.1)

http_thread = threading.Thread(target=http_server, daemon=True)
http_thread.start()

# Небольшая задержка для запуска HTTP сервера
time.sleep(0.1)

console_thread = threading.Thread(target=read_console, daemon=True)
console_thread.start()

print("="*60)
print("\nСпособы управления:")
print("1. Консоль Webots (введите команду здесь)")
print(f"2. TCP/IP: подключитесь к localhost:{TCP_PORT}")
print(f"3. HTTP REST API: откройте http://localhost:{HTTP_PORT}/ в браузере")
print("4. Файл commands.txt в папке контроллера")
print("\nВведите 'help' для справки по командам")
print("="*60)
sys.stdout.flush()

# Основной цикл
while robot.step(timeStep) != -1:
    # Применяем текущие скорости к моторам
    motor_index = 0
    for motor in motors:
        if motor is not None:
            if not position_mode[motor_index]:
                motor.setVelocity(current_velocities[motor_index])
            motor_index += 1

