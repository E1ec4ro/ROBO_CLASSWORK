"""
Пример TCP клиента для управления UR10e
Использование: python tcp_client_example.py

ВАЖНО: Перед запуском убедитесь, что:
1. Webots запущен с миром, использующим контроллер "ur10e_remote_control"
2. Симуляция запущена (кнопка Play)
3. Контроллер успешно инициализирован (проверьте консоль Webots)
"""

import socket
import json
import sys
import time

TCP_HOST = 'localhost'
TCP_PORT = 10001

def check_connection():
    """Проверяет, доступен ли сервер"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((TCP_HOST, TCP_PORT))
        sock.close()
        return result == 0
    except:
        return False

def send_command(command, timeout=5):
    """Отправляет команду через TCP"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((TCP_HOST, TCP_PORT))
        sock.sendall((command + '\n').encode('utf-8'))
        
        response = sock.recv(4096).decode('utf-8')
        result = json.loads(response)
        
        sock.close()
        return result
    except socket.timeout:
        return {"status": "error", "message": "Таймаут подключения. Сервер не отвечает."}
    except ConnectionRefusedError:
        return {"status": "error", "message": "Соединение отклонено. Убедитесь, что Webots запущен и симуляция активна."}
    except Exception as e:
        return {"status": "error", "message": f"Ошибка: {str(e)}"}

if __name__ == "__main__":
    print("="*60)
    print("TCP клиент для управления UR10e")
    print("="*60)
    print("\nПроверка подключения к серверу...")
    
    if not check_connection():
        print("\n❌ ОШИБКА: Не удается подключиться к серверу!")
        print("\nУбедитесь, что:")
        print("1. Webots запущен")
        print("2. Мир использует контроллер 'ur10e_remote_control'")
        print("3. Симуляция запущена (кнопка Play)")
        print("4. В консоли Webots видно сообщение 'TCP сервер запущен на порту 10001'")
        print(f"\nПопытка подключения к: {TCP_HOST}:{TCP_PORT}")
        sys.exit(1)
    
    print("✅ Подключение установлено!")
    print("="*60)
    
    # Интерактивный режим
    if len(sys.argv) > 1:
        # Режим командной строки
        command = ' '.join(sys.argv[1:])
        print(f"\nОтправка команды: {command}")
        result = send_command(command)
        print(f"\nРезультат:")
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        # Интерактивный режим
        print("\nИнтерактивный режим. Введите команды или 'exit' для выхода.")
        print("Примеры команд:")
        print("  help")
        print("  status")
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


