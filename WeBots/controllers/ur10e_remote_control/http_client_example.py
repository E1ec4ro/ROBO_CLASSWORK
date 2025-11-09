"""
Пример HTTP клиента для управления UR10e
Использование: 
  python http_client_example.py                    # Интерактивный режим
  python http_client_example.py "vel 1 0.5"       # Команда из командной строки
"""

import requests
import json
import sys

HTTP_HOST = 'http://localhost:10002'

def send_command(command):
    """Отправляет команду через HTTP"""
    try:
        url = f"{HTTP_HOST}/command?cmd={command}"
        response = requests.get(url, timeout=5)
        return response.json()
    except requests.exceptions.ConnectionError:
        return {
            "status": "error",
            "message": "Не удается подключиться к серверу. Убедитесь, что Webots запущен и симуляция активна."
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

def get_status():
    """Получает статус робота"""
    try:
        response = requests.get(f"{HTTP_HOST}/status", timeout=5)
        return response.json()
    except requests.exceptions.ConnectionError:
        return {
            "status": "error",
            "message": "Не удается подключиться к серверу. Убедитесь, что Webots запущен и симуляция активна."
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

def check_connection():
    """Проверяет доступность сервера"""
    try:
        response = requests.get(f"{HTTP_HOST}/status", timeout=2)
        return response.status_code == 200
    except:
        return False

if __name__ == "__main__":
    print("="*60)
    print("HTTP клиент для управления UR10e")
    print("="*60)
    print(f"\nПроверка подключения к {HTTP_HOST}...")
    
    if not check_connection():
        print("\n❌ ОШИБКА: Не удается подключиться к серверу!")
        print("\nУбедитесь, что:")
        print("1. Webots запущен")
        print("2. Мир использует контроллер 'ur10e_remote_control'")
        print("3. Симуляция запущена (кнопка Play)")
        print("4. В консоли Webots видно сообщение 'HTTP сервер запущен на порту 10002'")
        print(f"\nПопытка подключения к: {HTTP_HOST}")
        print("\nИли откройте в браузере: http://localhost:10002/")
        sys.exit(1)
    
    print("✅ Подключение установлено!")
    print("="*60)
    
    # Режим командной строки
    if len(sys.argv) > 1:
        command = ' '.join(sys.argv[1:])
        print(f"\nОтправка команды: {command}")
        result = send_command(command)
        print(f"\nРезультат:")
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        # Интерактивный режим
        print("\nИнтерактивный режим. Введите команды или 'exit' для выхода.")
        print("Специальные команды:")
        print("  status  - получить статус робота")
        print("  exit    - выйти")
        print("\nПримеры команд:")
        print("  help")
        print("  vel 1 0.5")
        print("  vel_all 0.5 0.3 0.4 0.2 0.3 0.25")
        print("  pos 1 1.57")
        print("  home")
        print("  stop")
        print("="*60)
        
        while True:
            try:
                command = input("\nВведите команду: ").strip()
                
                if not command:
                    continue
                
                if command.lower() in ['exit', 'quit', 'q']:
                    print("Выход...")
                    break
                
                if command.lower() == 'status':
                    print("Получение статуса...")
                    result = get_status()
                else:
                    print(f"Отправка: {command}")
                    result = send_command(command)
                
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
            
            except KeyboardInterrupt:
                print("\n\nВыход...")
                break
            except Exception as e:
                print(f"❌ Неожиданная ошибка: {e}")

