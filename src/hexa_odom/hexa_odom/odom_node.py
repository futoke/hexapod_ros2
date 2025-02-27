import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Создаем публикацию для топика /odom
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Создаем бродкастер для публикации трансформации
        self.tf_broadcaster = TransformBroadcaster(self)

        # Инициализируем переменные для хранения состояния робота
        self.x = 0.0  # Позиция по X
        self.y = 0.0  # Позиция по Y
        self.theta = 0.0  # Угол поворота (yaw)

        # Создаем таймер для обновления данных каждые 0.1 секунды (10 Гц)
        self.timer = self.create_timer(0.1, self.update_odometry)

    def update_odometry(self):
        # Обновляем состояние робота (пример линейного и углового движения)
        delta_x = 0.1 * math.cos(self.theta)  # Пример линейного перемещения
        delta_y = 0.1 * math.sin(self.theta)
        delta_theta = 0.05  # Пример углового перемещения

        # Обновляем координаты и угол
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Публикуем данные одометрии
        self.publish_odometry()

        # Публикуем трансформацию между odom и base_link
        self.publish_tf()

    def publish_odometry(self):
        # Создаем сообщение типа Odometry
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Заполняем позицию
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Заполняем ориентацию (кватернион)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Публикуем сообщение
        self.odom_publisher.publish(msg)

    def publish_tf(self):
        # Создаем трансформацию между odom и base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Заполняем позицию
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Заполняем ориентацию (кватернион)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Публикуем трансформацию
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()