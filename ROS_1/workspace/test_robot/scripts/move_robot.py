#!/usr/bin/env python3
"""
Скрипт для управления тестовым роботом
Использование:
    python3 move_robot.py forward    # Движение вперед
    python3 move_robot.py backward   # Движение назад
    python3 move_robot.py left       # Поворот влево
    python3 move_robot.py right      # Поворот вправо
    python3 move_robot.py stop      # Остановка
"""

import rospy
from geometry_msgs.msg import Twist
import sys

def move_robot(linear_x=0.0, angular_z=0.0, duration=1.0):
    """
    Отправка команды движения роботу
    
    Args:
        linear_x: линейная скорость (м/с), положительное значение = вперед
        angular_z: угловая скорость (рад/с), положительное значение = против часовой
        duration: длительность команды (секунды)
    """
    rospy.init_node('robot_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Небольшая задержка для подключения к топику
    rospy.sleep(0.5)
    rate = rospy.Rate(10)  # 10 Hz
    
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_z
    
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        pub.publish(twist)
        if (rospy.Time.now() - start_time).to_sec() >= duration:
            break
        rate.sleep()
    
    # Остановка
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print("Использование:")
            print("  python3 move_robot.py forward   # Движение вперед")
            print("  python3 move_robot.py backward  # Движение назад")
            print("  python3 move_robot.py left      # Поворот влево")
            print("  python3 move_robot.py right     # Поворот вправо")
            print("  python3 move_robot.py stop       # Остановка")
            sys.exit(1)
        
        command = sys.argv[1].lower()
        
        if command == 'forward':
            print("Движение вперед...")
            move_robot(linear_x=0.5, duration=2.0)
        elif command == 'backward':
            print("Движение назад...")
            move_robot(linear_x=-0.5, duration=2.0)
        elif command == 'left':
            print("Поворот влево...")
            move_robot(angular_z=0.5, duration=2.0)
        elif command == 'right':
            print("Поворот вправо...")
            move_robot(angular_z=-0.5, duration=2.0)
        elif command == 'stop':
            print("Остановка...")
            move_robot(linear_x=0.0, angular_z=0.0, duration=0.1)
        else:
            print(f"Неизвестная команда: {command}")
            sys.exit(1)
            
    except rospy.ROSInterruptException:
        pass

