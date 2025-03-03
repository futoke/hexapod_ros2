# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import tf_transformations
# import math

# class OdometryNode(Node):
#     def __init__(self):
#         super().__init__('odometry_node')

#         # Создаем публикацию для топика /odom
#         self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

#         # Создаем бродкастер для публикации трансформации
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Инициализируем переменные для хранения состояния робота
#         self.x = 0.0  # Позиция по X
#         self.y = 0.0  # Позиция по Y
#         self.theta = 0.0  # Угол поворота (yaw)

#         # Создаем таймер для обновления данных каждые 0.1 секунды (10 Гц)
#         self.timer = self.create_timer(0.1, self.update_odometry)

#     def update_odometry(self):
#         # Обновляем состояние робота (пример линейного и углового движения)
#         delta_x = 0.1 * math.cos(self.theta)  # Пример линейного перемещения
#         delta_y = 0.1 * math.sin(self.theta)
#         delta_theta = 0.05  # Пример углового перемещения

#         # Обновляем координаты и угол
#         self.x += delta_x
#         self.y += delta_y
#         self.theta += delta_theta

#         # Публикуем данные одометрии
#         self.publish_odometry()

#         # Публикуем трансформацию между odom и base_link
#         self.publish_tf()

#     def publish_odometry(self):
#         # Создаем сообщение типа Odometry
#         msg = Odometry()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'odom'
#         msg.child_frame_id = 'base_link'

#         # Заполняем позицию
#         msg.pose.pose.position.x = self.x
#         msg.pose.pose.position.y = self.y
#         msg.pose.pose.position.z = 0.0

#         # Заполняем ориентацию (кватернион)
#         quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
#         msg.pose.pose.orientation.x = quat[0]
#         msg.pose.pose.orientation.y = quat[1]
#         msg.pose.pose.orientation.z = quat[2]
#         msg.pose.pose.orientation.w = quat[3]

#         # Публикуем сообщение
#         self.odom_publisher.publish(msg)

#     def publish_tf(self):
#         # Создаем трансформацию между odom и base_link
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'

#         # Заполняем позицию
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0

#         # Заполняем ориентацию (кватернион)
#         quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
#         t.transform.rotation.x = quat[0]
#         t.transform.rotation.y = quat[1]
#         t.transform.rotation.z = quat[2]
#         t.transform.rotation.w = quat[3]

#         # Публикуем трансформацию
#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdometryNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# from sensor_msgs.msg import JointState
# import math
# import tf_transformations

# class SpiderOdometryNode(Node):
#     def __init__(self):
#         super().__init__('odom_node')

#         # Создаем публикатор для топика /odom
#         self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

#         # Создаем бродкастер для трансформации odom -> base_link
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Длина звеньев ноги
#         self.L_link1 = 0.077942  # Длина coxa
#         self.L_link2 = 0.07059   # Длина femur
#         self.L_link3 = 0.0794  # Длина tibia

#         # Инициализируем переменные для хранения состояния
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.prev_x_center = 0.0
#         self.prev_y_center = 0.0

#         # Подписываемся на топик /joint_states
#         self.joint_state_subscriber = self.create_subscription(
#             JointState, '/joint_states', self.joint_state_callback, 10
#         )

#         # Таймер для публикации данных каждые 0.1 секунды (10 Гц)
#         self.timer = self.create_timer(0.1, self.update_odometry)

#         # Храним текущие углы суставов
#         self.joint_angles = {}

#     def joint_state_callback(self, msg):
#         """Обработка сообщений из топика /joint_states."""
#         for name, position in zip(msg.name, msg.position):
#             self.joint_angles[name] = position

#     def calculate_leg_position(self, link1, link2, link3):
#         """Расчет положения конца ноги через прямую кинематику."""
#         x_leg = (
#             self.L_link1 * math.cos(link1) +
#             self.L_link2 * math.cos(link1 + link2) +
#             self.L_link3 * math.cos(link1 + link2 + link3)
#         )
#         y_leg = (
#             self.L_link1 * math.sin(link1) +
#             self.L_link2 * math.sin(link1 + link2) +
#             self.L_link3 * math.sin(link1 + link2 + link3)
#         )
#         return x_leg, y_leg

#     def update_odometry(self):
#         """Обновление одометрии на основе текущих углов суставов."""
#         if not self.joint_angles:
#             self.get_logger().warn("No joint angles received yet. Skipping odometry update.")
#             return

#         # Определение имен суставов для каждой ноги
#         leg_joints = [
#             ("leg1_link1_joint", "leg1_link2_joint", "leg1_link3_joint"),  # Нога 1
#             ("leg2_link1_joint", "leg2_link2_joint", "leg2_link3_joint"),  # Нога 2
#             ("leg3_link1_joint", "leg3_link2_joint", "leg3_link3_joint"),  # Нога 3
#             ("leg4_link1_joint", "leg4_link2_joint", "leg4_link3_joint"),  # Нога 4
#             ("leg5_link1_joint", "leg5_link2_joint", "leg5_link3_joint"),  # Нога 5
#             ("leg6_link1_joint", "leg6_link2_joint", "leg6_link3_joint")   # Нога 6
#         ]

#         # Рассчитываем положение центра массы
#         x_center = 0.0
#         y_center = 0.0
#         for joints in leg_joints:
#             try:
#                 link1 = self.joint_angles[joints[0]]
#                 link2 = self.joint_angles[joints[1]]
#                 link3 = self.joint_angles[joints[2]]

#                 x_leg, y_leg = self.calculate_leg_position(link1, link2, link3)
#                 x_center += x_leg
#                 y_center += y_leg
#             except KeyError:
#                 self.get_logger().warn(f"Missing joint angles for leg: {joints}. Skipping this leg.")
#                 continue

#         if len(leg_joints) > 0:
#             x_center /= len(leg_joints)
#             y_center /= len(leg_joints)

#         # Вычисляем изменение положения
#         delta_x = x_center - self.prev_x_center
#         delta_y = y_center - self.prev_y_center

#         # Обновляем предыдущее положение центра массы
#         self.prev_x_center = x_center
#         self.prev_y_center = y_center

#         # Вычисляем угол направления (yaw)
#         if delta_x != 0 or delta_y != 0:
#             self.theta += math.atan2(delta_y, delta_x)

#         # Обновляем глобальное положение
#         self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
#         self.y += delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)

#         # Публикуем данные одометрии
#         self.publish_odometry()

#     def publish_odometry(self):
#         """Публикация данных одометрии."""
#         msg = Odometry()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'odom'
#         msg.child_frame_id = 'base_link'

#         # Заполняем позицию
#         msg.pose.pose.position.x = self.x
#         msg.pose.pose.position.y = self.y
#         msg.pose.pose.position.z = 0.0

#         # Заполняем ориентацию (кватернион)
#         quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
#         msg.pose.pose.orientation.x = quat[0]
#         msg.pose.pose.orientation.y = quat[1]
#         msg.pose.pose.orientation.z = quat[2]
#         msg.pose.pose.orientation.w = quat[3]

#         # Публикуем сообщение
#         self.odom_publisher.publish(msg)

#         # Публикуем трансформацию между odom и base_link
#         self.publish_tf()

#     def publish_tf(self):
#         """Публикация трансформации между odom и base_link."""
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'

#         # Заполняем позицию
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0

#         # Заполняем ориентацию (кватернион)
#         quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
#         t.transform.rotation.x = quat[0]
#         t.transform.rotation.y = quat[1]
#         t.transform.rotation.z = quat[2]
#         t.transform.rotation.w = quat[3]

#         # Публикуем трансформацию
#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = SpiderOdometryNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class IMUOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')

        # Создаем публикатор для топика /odom
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Создаем бродкастер для трансформации odom -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Инициализируем переменные для хранения состояния
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = None
        self.prev_linear_velocity = [0.0, 0.0, 0.0]

        # Подписываемся на топик /imu
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

    def imu_callback(self, msg):
        """Обработка данных IMU."""
        if self.prev_time is None:
            self.prev_time = self.get_clock().now()
            return

        # Текущее время
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Временной интервал в секундах

        # Получаем данные из IMU
        angular_velocity = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        linear_acceleration = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

        # Интегрируем угловую скорость для получения изменения ориентации (yaw)
        self.theta += angular_velocity[2] * dt  # Берем yaw (z-ось)

        # Компенсируем влияние земного притяжения
        linear_acceleration[2] -= 9.81  # Убираем компоненту гравитации

        # Интегрируем линейное ускорение для получения скорости
        linear_velocity = [
            (self.prev_linear_velocity[i] + linear_acceleration[i]) * 0.5 * dt
            for i in range(3)
        ]

        # Интегрируем скорость для получения перемещения
        displacement = [v * dt for v in linear_velocity]

        # Обновляем координаты робота
        delta_x = displacement[0] * math.cos(self.theta) - displacement[1] * math.sin(self.theta)
        delta_y = displacement[0] * math.sin(self.theta) + displacement[1] * math.cos(self.theta)

        self.x += delta_x
        self.y += delta_y

        # Публикуем данные одометрии
        self.publish_odometry()

        # Обновляем предыдущие значения
        self.prev_time = current_time
        self.prev_linear_velocity = linear_acceleration

    def publish_odometry(self):
        """Публикация данных одометрии."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Заполняем позицию
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Заполняем ориентацию (кватернион)
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, self.theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Публикуем сообщение
        self.odom_publisher.publish(msg)

        # Публикуем трансформацию между odom и base_link
        self.publish_tf()

    def publish_tf(self):
        """Публикация трансформации между odom и base_link."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Заполняем позицию
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Заполняем ориентацию (кватернион)
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Публикуем трансформацию
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = IMUOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()