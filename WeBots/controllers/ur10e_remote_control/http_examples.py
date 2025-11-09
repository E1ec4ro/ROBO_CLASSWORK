"""
Примеры использования HTTP API для управления UR10e
Использование: python http_examples.py
"""

import requests
import time
import json
import math

BASE_URL = 'http://localhost:10002'

def send_command(command, timeout=5):
    """Отправляет команду на робота через HTTP"""
    try:
        url = f"{BASE_URL}/command?cmd={command}"
        response = requests.get(url, timeout=timeout)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.ConnectionError:
        return {"status": "error", "message": "Не удается подключиться к серверу. Убедитесь, что Webots запущен."}
    except requests.exceptions.Timeout:
        return {"status": "error", "message": "Таймаут запроса"}
    except Exception as e:
        return {"status": "error", "message": str(e)}

def get_status():
    """Получает статус робота"""
    try:
        response = requests.get(f"{BASE_URL}/status", timeout=5)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        return {"status": "error", "message": str(e)}

def print_result(result):
    """Красиво выводит результат"""
    if result.get("status") == "ok":
        print("✅ Успешно!")
        if "message" in result:
            print(f"   {result['message']}")
        if "data" in result:
            print(f"\n   Данные:")
            print(json.dumps(result["data"], indent=4, ensure_ascii=False))
    else:
        print("❌ Ошибка!")
        if "message" in result:
            print(f"   {result['message']}")

# ============================================
# ПРИМЕР 1: Базовые команды
# ============================================
def example_basic_commands():
    """Пример базовых команд"""
    print("\n" + "="*60)
    print("ПРИМЕР 1: Базовые команды")
    print("="*60)
    
    commands = [
        "help",
        "status",
        "vel 1 0.5",
        "stop"
    ]
    
    for cmd in commands:
        print(f"\nКоманда: {cmd}")
        result = send_command(cmd)
        print_result(result)
        time.sleep(1)

# ============================================
# ПРИМЕР 2: Управление скоростями
# ============================================
def example_velocity_control():
    """Пример управления скоростями суставов"""
    print("\n" + "="*60)
    print("ПРИМЕР 2: Управление скоростями")
    print("="*60)
    
    # Установить скорость для каждого сустава
    for joint in range(1, 7):
        velocity = 0.3 + (joint - 1) * 0.1
        cmd = f"vel {joint} {velocity}"
        print(f"\nКоманда: {cmd}")
        result = send_command(cmd)
        print_result(result)
        time.sleep(0.5)
    
    time.sleep(3)
    
    # Остановить все
    print("\nОстановка всех моторов...")
    result = send_command("stop")
    print_result(result)

# ============================================
# ПРИМЕР 3: Установка всех скоростей сразу
# ============================================
def example_velocity_all():
    """Пример установки всех скоростей одновременно"""
    print("\n" + "="*60)
    print("ПРИМЕР 3: Установка всех скоростей")
    print("="*60)
    
    # Установить разные скорости для всех суставов
    velocities = [0.5, 0.3, 0.4, 0.2, 0.3, 0.25]
    cmd = "vel_all " + " ".join(map(str, velocities))
    
    print(f"\nКоманда: {cmd}")
    result = send_command(cmd)
    print_result(result)
    
    time.sleep(5)
    
    # Остановить
    print("\nОстановка...")
    result = send_command("stop")
    print_result(result)

# ============================================
# ПРИМЕР 4: Управление позициями
# ============================================
def example_position_control():
    """Пример управления позициями суставов"""
    print("\n" + "="*60)
    print("ПРИМЕР 4: Управление позициями")
    print("="*60)
    
    # Вернуть в исходное положение
    print("\nВозврат в исходное положение...")
    result = send_command("home")
    print_result(result)
    time.sleep(3)
    
    # Установить позицию сустава 1 (90 градусов = 1.57 радиан)
    print("\nУстановка позиции сустава 1 в 90 градусов...")
    result = send_command("pos 1 1.57")
    print_result(result)
    time.sleep(3)
    
    # Вернуть обратно
    print("\nВозврат в исходное положение...")
    result = send_command("home")
    print_result(result)

# ============================================
# ПРИМЕР 5: Синусоидальное движение
# ============================================
def example_sine_wave():
    """Пример синусоидального движения"""
    print("\n" + "="*60)
    print("ПРИМЕР 5: Синусоидальное движение")
    print("="*60)
    
    print("\nСустав 1 будет двигаться по синусоиде...")
    print("Нажмите Ctrl+C для остановки")
    
    try:
        for i in range(100):
            # Синусоидальная скорость от -0.5 до 0.5
            velocity = 0.5 * math.sin(i * 0.1)
            cmd = f"vel 1 {velocity:.3f}"
            send_command(cmd)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nОстановка...")
        send_command("stop")

# ============================================
# ПРИМЕР 6: Последовательность движений
# ============================================
def example_sequence():
    """Пример последовательности движений"""
    print("\n" + "="*60)
    print("ПРИМЕР 6: Последовательность движений")
    print("="*60)
    
    sequence = [
        ("home", "Возврат в исходное положение"),
        ("vel 1 0.5", "Вращение сустава 1"),
        ("vel 2 0.3", "Вращение сустава 2"),
        ("vel 3 0.4", "Вращение сустава 3"),
        ("stop", "Остановка")
    ]
    
    for cmd, description in sequence:
        print(f"\n{description}...")
        print(f"Команда: {cmd}")
        result = send_command(cmd)
        print_result(result)
        time.sleep(2)

# ============================================
# ПРИМЕР 7: Мониторинг статуса
# ============================================
def example_status_monitoring():
    """Пример мониторинга статуса"""
    print("\n" + "="*60)
    print("ПРИМЕР 7: Мониторинг статуса")
    print("="*60)
    
    # Запустить движение
    send_command("vel 1 0.5")
    
    # Мониторить статус 5 раз
    for i in range(5):
        print(f"\nПроверка статуса #{i+1}...")
        result = get_status()
        if result.get("status") == "ok" and "data" in result:
            motors = result["data"].get("motors", [])
            for motor in motors:
                print(f"  Сустав {motor['joint']}: скорость = {motor['velocity']:.3f} рад/с")
        time.sleep(1)
    
    # Остановить
    send_command("stop")

# ============================================
# ГЛАВНАЯ ФУНКЦИЯ
# ============================================
if __name__ == "__main__":
    print("="*60)
    print("ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ HTTP API ДЛЯ UR10e")
    print("="*60)
    print("\nУбедитесь, что:")
    print("1. Webots запущен")
    print("2. Симуляция активна")
    print("3. HTTP сервер работает на порту 10002")
    print("\n" + "="*60)
    
    # Проверка подключения
    print("\nПроверка подключения...")
    result = get_status()
    if result.get("status") != "ok":
        print("❌ Не удается подключиться к серверу!")
        print("   Убедитесь, что Webots запущен и симуляция активна.")
        exit(1)
    
    print("✅ Подключение установлено!")
    
    # Меню выбора примера
    examples = {
        "1": ("Базовые команды", example_basic_commands),
        "2": ("Управление скоростями", example_velocity_control),
        "3": ("Установка всех скоростей", example_velocity_all),
        "4": ("Управление позициями", example_position_control),
        "5": ("Синусоидальное движение", example_sine_wave),
        "6": ("Последовательность движений", example_sequence),
        "7": ("Мониторинг статуса", example_status_monitoring),
    }
    
    print("\nДоступные примеры:")
    for key, (name, _) in examples.items():
        print(f"  {key}. {name}")
    print("  all. Запустить все примеры")
    print("  0.  Выход")
    
    choice = input("\nВыберите пример (1-7, all, 0): ").strip()
    
    if choice == "0":
        print("Выход...")
    elif choice == "all":
        for key, (name, func) in examples.items():
            try:
                func()
            except KeyboardInterrupt:
                print("\n\nПрервано пользователем")
                send_command("stop")
                break
            except Exception as e:
                print(f"\n❌ Ошибка в примере '{name}': {e}")
    elif choice in examples:
        name, func = examples[choice]
        try:
            func()
        except KeyboardInterrupt:
            print("\n\nПрервано пользователем")
            send_command("stop")
        except Exception as e:
            print(f"\n❌ Ошибка: {e}")
    else:
        print("Неверный выбор!")

