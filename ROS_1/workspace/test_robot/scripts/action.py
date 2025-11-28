#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# Параметры движения (можно подбирать)
SIDE_LENGTH = 0.5          # длина стороны квадрата (метры)
TURN_ANGLE = math.pi / 2.0 # поворот 90 градусов
LINEAR_SPEED = 0.25        # линейная скорость, м/с
ANGULAR_SPEED = 0.4        # угловая скорость, рад/с
DIST_TOLERANCE = 0.03      # допуск по расстоянию (м)
ANGLE_TOLERANCE = 0.05     # допуск по углу (рад) ~ 3°


class OdomTracker:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._got_odom = False
        rospy.Subscriber("/odom", Odometry, self._callback)

    def _callback(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # преобразуем кватернион в угол yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self._got_odom = True

    def wait_for_odom(self, timeout: float = 10.0) -> bool:
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self._got_odom:
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("Не дождались данных /odom")
                return False
            rospy.loginfo("Ожидаем первые данные /odom...")
            rate.sleep()
        return self._got_odom


def move_distance(pub: rospy.Publisher,
                  odom: OdomTracker,
                  distance: float,
                  speed: float) -> None:
    """
    Едет вперёд, пока робот не проедет нужную дистанцию по /odom.
    """
    start_x = odom.x
    start_y = odom.y

    twist = Twist()
    twist.linear.x = math.copysign(speed, distance)
    rate = rospy.Rate(20)
    start_time = rospy.Time.now()
    # Максимальное время движения с запасом, чтобы не «зависнуть» навсегда
    max_duration = abs(distance) / max(speed, 1e-3) * 2.0

    while not rospy.is_shutdown():
        dx = odom.x - start_x
        dy = odom.y - start_y
        traveled = math.sqrt(dx*dx + dy*dy)
        # Для отладки можно раскомментировать следующую строку:
        # rospy.loginfo(f"move_distance: traveled={traveled:.3f} / {abs(distance):.3f}")
        if traveled + DIST_TOLERANCE >= abs(distance):
            break
        if (rospy.Time.now() - start_time).to_sec() > max_duration:
            rospy.logwarn("move_distance: превышено максимальное время движения, выходим из цикла")
            break
        pub.publish(twist)
        rate.sleep()

    twist.linear.x = 0.0
    pub.publish(twist)
    rospy.sleep(0.3)


def rotate_angle(pub: rospy.Publisher,
                 odom: OdomTracker,
                 angle: float,
                 angular_speed: float) -> None:
    """
    Поворачивает робота на заданный угол (рад) по /odom.
    """
    """
    Поворачивает робот на угол angle (рад), используя изменение yaw по /odom.
    Без сложного регулятора, чтобы избежать «дрожания» на месте.
    """
    start_yaw = odom.yaw

    def norm(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    rate = rospy.Rate(30)
    twist = Twist()
    twist.angular.z = math.copysign(angular_speed, angle)

    max_duration = abs(angle) / max(angular_speed, 1e-3) * 3.0
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # сколько реально повернули от начального yaw
        turned = norm(odom.yaw - start_yaw)

        # достаточно близко к требуемому углу
        if abs(abs(turned) - abs(angle)) <= ANGLE_TOLERANCE:
            break

        # защита от зависания
        if (rospy.Time.now() - start_time).to_sec() > max_duration:
            rospy.logwarn("rotate_angle: превышено максимальное время поворота, выходим из цикла")
            break

        pub.publish(twist)
        rate.sleep()

    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.2)


def main() -> None:
    rospy.init_node('action')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    odom = OdomTracker()

    rospy.loginfo("Ожидаем данные /odom...")
    odom.wait_for_odom()

    rospy.loginfo("Начинаем движение по квадрату (по одометрии)")

    # 4 стороны квадрата
    for i in range(4):
        rospy.loginfo(f"Сторона {i + 1}")
        move_distance(pub, odom, SIDE_LENGTH, LINEAR_SPEED)

        rospy.loginfo("Поворот на 90 градусов")
        rotate_angle(pub, odom, TURN_ANGLE, ANGULAR_SPEED)

    rospy.loginfo("Поворот на 180 градусов")
    rotate_angle(pub, odom, math.pi, ANGULAR_SPEED)

    rospy.loginfo("Движение назад по прямой")
    move_distance(pub, odom, SIDE_LENGTH * 4.0, LINEAR_SPEED)

    rospy.loginfo("Готово")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
