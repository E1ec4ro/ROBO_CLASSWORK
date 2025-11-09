"""
Контроллер для управления двигателями робота UR10e в WeBots
Этот контроллер управляет вращением всех 6 суставов робота-манипулятора
"""

from controller import Robot
import math

# Создаем экземпляр робота
robot = Robot()

# Получаем временной шаг симуляции
timeStep = int(robot.getBasicTimeStep())

# Имена суставов UR10e (6 степеней свободы)
# Пробуем разные варианты имен в зависимости от версии WeBots
joint_name_variants = [
    # Вариант 1: стандартные имена
    [
        'shoulder_pan_joint',    # Сустав 1 - поворот основания
        'shoulder_lift_joint',   # Сустав 2 - подъем плеча
        'elbow_joint',           # Сустав 3 - локоть
        'wrist_1_joint',         # Сустав 4 - запястье 1
        'wrist_2_joint',         # Сустав 5 - запястье 2
        'wrist_3_joint'          # Сустав 6 - запястье 3
    ],
    # Вариант 2: с префиксом UR10e::
    [
        'UR10e::shoulder_pan_joint',
        'UR10e::shoulder_lift_joint',
        'UR10e::elbow_joint',
        'UR10e::wrist_1_joint',
        'UR10e::wrist_2_joint',
        'UR10e::wrist_3_joint'
    ],
    # Вариант 3: альтернативные имена
    [
        'base_joint',
        'shoulder_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]
]

# Получаем моторы для каждого сустава
motors = []
joint_names_used = []

# Пробуем найти моторы по разным вариантам имен
for variant in joint_name_variants:
    motors = []
    joint_names_used = []
    found_count = 0
    
    for joint_name in variant:
        motor = robot.getDevice(joint_name)
        if motor:
            # Устанавливаем режим управления скоростью
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            motors.append(motor)
            joint_names_used.append(joint_name)
            found_count += 1
            print(f"Мотор {joint_name} инициализирован")
        else:
            motors.append(None)
            joint_names_used.append(joint_name)
    
    if found_count > 0:
        print(f"\nНайдено моторов: {found_count} из {len(variant)}")
        break

# Если не нашли моторы по именам, пробуем найти все устройства типа Motor
if len([m for m in motors if m is not None]) == 0:
    print("\nПопытка автоматического поиска всех моторов...")
    motors = []
    joint_names_used = []
    
    # Получаем количество устройств
    num_devices = robot.getNumberOfDevices()
    for i in range(num_devices):
        device = robot.getDeviceByIndex(i)
        if device and device.getNodeType() == device.MOTOR:
            device.setPosition(float('inf'))
            device.setVelocity(0.0)
            motors.append(device)
            joint_names_used.append(device.getName())
            print(f"Найден мотор: {device.getName()}")

print(f"\nИнициализировано моторов: {len([m for m in motors if m is not None])}")
print("Контроллер готов к работе!\n")

# Параметры управления
current_time = 0.0
phase = 0.0

# Основной цикл управления
while robot.step(timeStep) != -1:
    current_time = robot.getTime()
    
    # Пример 1: Простое вращение всех суставов с разными скоростями
    # Скорости в радианах в секунду
    num_motors = len([m for m in motors if m is not None])
    
    # Генерируем скорости для всех найденных моторов
    velocities = []
    for i in range(num_motors):
        if i == 0:
            velocities.append(0.5 * math.sin(current_time))      # Сустав 1: синусоидальное движение
        elif i == 1:
            velocities.append(0.3 * math.cos(current_time * 0.7)) # Сустав 2: косинусоидальное движение
        elif i == 2:
            velocities.append(0.4 * math.sin(current_time * 1.2)) # Сустав 3
        elif i == 3:
            velocities.append(0.2 * math.cos(current_time * 0.5)) # Сустав 4
        elif i == 4:
            velocities.append(0.3 * math.sin(current_time * 0.8)) # Сустав 5
        elif i == 5:
            velocities.append(0.25 * math.cos(current_time * 1.1)) # Сустав 6
        else:
            # Для дополнительных моторов (если есть)
            velocities.append(0.2 * math.sin(current_time * (0.5 + i * 0.1)))
    
    # Пример 2: Раскомментируйте для постоянного вращения
    # velocities = [0.5] * num_motors
    
    # Пример 3: Раскомментируйте для последовательного движения
    # phase = current_time % num_motors
    # velocities = [0.0] * num_motors
    # joint_index = int(phase)
    # if joint_index < num_motors:
    #     velocities[joint_index] = 0.5
    
    # Применяем скорости к моторам
    motor_index = 0
    for motor in motors:
        if motor is not None:
            if motor_index < len(velocities):
                motor.setVelocity(velocities[motor_index])
                motor_index += 1
    
    # Вывод информации каждую секунду
    if int(current_time) % 1 == 0 and (current_time - int(current_time)) < timeStep / 1000.0:
        print(f"Время: {current_time:.2f}с")
        motor_index = 0
        for i, motor in enumerate(motors):
            if motor is not None:
                if motor_index < len(velocities):
                    joint_name = joint_names_used[i] if i < len(joint_names_used) else f"Мотор {i+1}"
                    print(f"  {joint_name}: скорость = {velocities[motor_index]:.3f} рад/с")
                    motor_index += 1
        print()

