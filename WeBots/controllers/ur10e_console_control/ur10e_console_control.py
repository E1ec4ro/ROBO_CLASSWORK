"""
Контроллер для управления роботом UR10e из консоли в WeBots
Поддерживает команды для управления суставами робота
"""

from controller import Robot
import threading
import queue

# Создаем экземпляр робота
robot = Robot()

# Получаем временной шаг симуляции
timeStep = int(robot.getBasicTimeStep())

# Очередь для команд из консоли
command_queue = queue.Queue()

# Имена суставов UR10e (6 степеней свободы)
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

# Получаем моторы для каждого сустава
motors = []
joint_names_used = []

# Пробуем найти моторы
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

# Если не нашли моторы по именам, пробуем найти все устройства типа Motor
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
print(f"\n{'='*60}")
print(f"Контроллер UR10e готов к работе!")
print(f"Найдено моторов: {num_motors}")
print(f"{'='*60}")
print("\nДоступные команды:")
print("  help              - показать справку")
print("  stop              - остановить все моторы")
print("  vel <j> <v>       - установить скорость сустава j (1-6) в рад/с")
print("  vel_all <v1> ... <v6> - установить скорости всех суставов")
print("  pos <j> <p>       - установить позицию сустава j в радианах")
print("  pos_all <p1> ... <p6> - установить позиции всех суставов")
print("  home              - вернуть робота в исходное положение")
print("  status            - показать текущее состояние суставов")
print("\nПримеры:")
print("  vel 1 0.5         - вращать сустав 1 со скоростью 0.5 рад/с")
print("  vel_all 0.5 0.3 0.4 0.2 0.3 0.25 - установить скорости всех суставов")
print("  pos 1 1.57        - установить сустав 1 в позицию 1.57 рад (90 градусов)")
print(f"{'='*60}\n")

# Текущие скорости и позиции
current_velocities = [0.0] * num_motors
target_positions = [None] * num_motors
position_mode = [False] * num_motors

def print_help():
    """Выводит справку по командам"""
    print("\nДоступные команды:")
    print("  help              - показать справку")
    print("  stop              - остановить все моторы")
    print("  vel <j> <v>       - установить скорость сустава j (1-6) в рад/с")
    print("  vel_all <v1> ... <v6> - установить скорости всех суставов")
    print("  pos <j> <p>       - установить позицию сустава j в радианах")
    print("  pos_all <p1> ... <p6> - установить позиции всех суставов")
    print("  home              - вернуть робота в исходное положение")
    print("  status            - показать текущее состояние суставов")
    print()

def process_command(cmd):
    """Обрабатывает команду из консоли"""
    global current_velocities, target_positions, position_mode
    
    parts = cmd.strip().split()
    if not parts:
        return
    
    command = parts[0].lower()
    
    try:
        if command == 'help':
            print_help()
        
        elif command == 'stop':
            current_velocities = [0.0] * num_motors
            target_positions = [None] * num_motors
            position_mode = [False] * num_motors
            motor_index = 0
            for motor in motors:
                if motor is not None:
                    motor.setVelocity(0.0)
                    motor_index += 1
            print("Все моторы остановлены")
        
        elif command == 'vel':
            if len(parts) < 3:
                print("Ошибка: Используйте: vel <номер_сустава> <скорость>")
                return
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
                            joint_name = joint_names_used[i] if i < len(joint_names_used) else f"Сустав {joint_num+1}"
                            print(f"Сустав {joint_num+1} ({joint_name}): скорость = {velocity:.3f} рад/с")
                            break
                        motor_index += 1
            else:
                print(f"Ошибка: Номер сустава должен быть от 1 до {num_motors}")
        
        elif command == 'vel_all':
            if len(parts) < num_motors + 1:
                print(f"Ошибка: Нужно указать {num_motors} скоростей")
                return
            velocities = [float(v) for v in parts[1:num_motors+1]]
            current_velocities = velocities
            target_positions = [None] * num_motors
            position_mode = [False] * num_motors
            motor_index = 0
            for motor in motors:
                if motor is not None:
                    motor.setVelocity(velocities[motor_index])
                    motor_index += 1
            print(f"Установлены скорости: {[f'{v:.3f}' for v in velocities]}")
        
        elif command == 'pos':
            if len(parts) < 3:
                print("Ошибка: Используйте: pos <номер_сустава> <позиция>")
                return
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
                            motor.setVelocity(1.0)  # Скорость движения к позиции
                            joint_name = joint_names_used[i] if i < len(joint_names_used) else f"Сустав {joint_num+1}"
                            print(f"Сустав {joint_num+1} ({joint_name}): позиция = {position:.3f} рад")
                            break
                        motor_index += 1
            else:
                print(f"Ошибка: Номер сустава должен быть от 1 до {num_motors}")
        
        elif command == 'pos_all':
            if len(parts) < num_motors + 1:
                print(f"Ошибка: Нужно указать {num_motors} позиций")
                return
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
            print(f"Установлены позиции: {[f'{p:.3f}' for p in positions]}")
        
        elif command == 'home':
            # Исходное положение (все суставы в 0)
            home_positions = [0.0] * num_motors
            target_positions = home_positions
            position_mode = [True] * num_motors
            current_velocities = [0.0] * num_motors
            motor_index = 0
            for motor in motors:
                if motor is not None:
                    motor.setPosition(home_positions[motor_index])
                    motor.setVelocity(0.5)  # Медленное возвращение
                    motor_index += 1
            print("Робот возвращается в исходное положение...")
        
        elif command == 'status':
            print("\nТекущее состояние суставов:")
            motor_index = 0
            for i, motor in enumerate(motors):
                if motor is not None:
                    joint_name = joint_names_used[i] if i < len(joint_names_used) else f"Сустав {motor_index+1}"
                    if position_mode[motor_index]:
                        pos = target_positions[motor_index] if target_positions[motor_index] is not None else 0.0
                        print(f"  Сустав {motor_index+1} ({joint_name}): позиция = {pos:.3f} рад (режим позиции)")
                    else:
                        vel = current_velocities[motor_index]
                        print(f"  Сустав {motor_index+1} ({joint_name}): скорость = {vel:.3f} рад/с")
                    motor_index += 1
            print()
        
        else:
            print(f"Неизвестная команда: {command}. Введите 'help' для справки")
    
    except ValueError as e:
        print(f"Ошибка: Неверный формат числа - {e}")
    except Exception as e:
        print(f"Ошибка при выполнении команды: {e}")

# Функция для чтения команд из консоли (в отдельном потоке)
def read_console():
    """Читает команды из консоли"""
    import sys
    
    # Импортируем select только для Linux/Mac
    try:
        import select
        has_select = True
    except ImportError:
        has_select = False
    
    while True:
        try:
            # Пробуем неблокирующее чтение для Linux/Mac
            if has_select and sys.platform != 'win32':
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    line = sys.stdin.readline()
                    if line:
                        command_queue.put(line.strip())
            else:
                # Для Windows используем блокирующий input в отдельном потоке
                line = input()
                if line:
                    command_queue.put(line.strip())
        except (EOFError, KeyboardInterrupt):
            break
        except:
            # Если не получается читать из stdin, пробуем читать из файла команд
            try:
                import os
                cmd_file = os.path.join(os.path.dirname(__file__), 'commands.txt')
                if os.path.exists(cmd_file):
                    with open(cmd_file, 'r') as f:
                        lines = f.readlines()
                        if lines:
                            for line in lines:
                                if line.strip():
                                    command_queue.put(line.strip())
                            # Очищаем файл после чтения
                            open(cmd_file, 'w').close()
            except:
                pass
            import time
            time.sleep(0.1)

# Запускаем поток для чтения консоли
console_thread = threading.Thread(target=read_console, daemon=True)
console_thread.start()

# Основной цикл управления
print("="*60)
print("КОНСОЛЬНОЕ УПРАВЛЕНИЕ UR10e")
print("="*60)
print("\nКАК ВВОДИТЬ КОМАНДЫ:")
print("1. Откройте окно 'Controller Console' в Webots")
print("   (Оно открывается автоматически или через меню)")
print("2. Введите команду и нажмите Enter")
print("3. Или создайте файл 'commands.txt' в папке контроллера")
print("\nВведите команду (или 'help' для справки):\n")

while robot.step(timeStep) != -1:
    # Обрабатываем команды из очереди
    try:
        while not command_queue.empty():
            cmd = command_queue.get_nowait()
            process_command(cmd)
    except:
        pass
    
    # Применяем текущие скорости к моторам (если не в режиме позиции)
    motor_index = 0
    for motor in motors:
        if motor is not None:
            if not position_mode[motor_index]:
                motor.setVelocity(current_velocities[motor_index])
            motor_index += 1

