import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import math
import time

class MPU6050Driver(Node):
    def __init__(self):
        super().__init__('mpu6050_driver')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Инициализация I2C
        self.bus = SMBus(1)  # Используется шина I2C-1
        self.address = 0x68  # Адрес MPU6050

        # Инициализация MPU6050
        self.init_mpu6050()

        # Переменные для хранения данных
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        # Калибровочные параметры
        self.accel_bias = [0.0, 0.0, 0.0]  # Смещение акселерометра
        self.gyro_bias = [0.0, 0.0, 0.0]   # Смещение гироскопа
        self.calibration_completed = False  # Флаг завершения калибровки

        # Запуск калибровки при инициализации
        self.get_logger().info("Starting calibration...")
        self.calibrate()

        # Параметры фильтрации
        self.alpha = 0.98  # Коэффициент для complementary filter
        self.filtered_ax = 0.0
        self.filtered_ay = 0.0
        self.filtered_az = 0.0
        self.filtered_gx = 0.0
        self.filtered_gy = 0.0
        self.filtered_gz = 0.0

    def init_mpu6050(self):
        """Инициализация MPU6050"""
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Включаем датчик
        self.bus.write_byte_data(self.address, 0x1C, 0x10)  # Настройка акселерометра ±2g
        self.bus.write_byte_data(self.address, 0x1B, 0x08)  # Настройка гироскопа ±500°/s

    def read_word_2c(self, register):
        """Чтение 16-битного значения со знаком"""
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def read_acceleration(self):
        """Чтение данных с акселерометра"""
        self.ax = self.read_word_2c(0x3B) / 16384.0
        self.ay = self.read_word_2c(0x3D) / 16384.0
        self.az = self.read_word_2c(0x3F) / 16384.0

    def read_gyroscope(self):
        """Чтение данных с гироскопа"""
        self.gx = self.read_word_2c(0x43) / 131.0
        self.gy = self.read_word_2c(0x45) / 131.0
        self.gz = self.read_word_2c(0x47) / 131.0

    def calibrate(self):
        """Калибровка MPU6050"""
        self.get_logger().info("Calibrating MPU6050... Keep the sensor still.")
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        samples = 100  # Количество измерений для калибровки

        for _ in range(samples):
            self.read_acceleration()
            self.read_gyroscope()

            # Собираем данные
            accel_sum[0] += self.ax
            accel_sum[1] += self.ay
            accel_sum[2] += self.az
            gyro_sum[0] += self.gx
            gyro_sum[1] += self.gy
            gyro_sum[2] += self.gz

            time.sleep(0.01)  # Задержка между измерениями

        # Вычисляем средние значения
        self.accel_bias = [x / samples for x in accel_sum]
        self.gyro_bias = [x / samples for x in gyro_sum]

        # Компенсируем влияние земного притяжения на Z-ось акселерометра
        self.accel_bias[2] -= 1.0  # Учитываем g = 1.0 (нормализованное значение)

        self.get_logger().info(f"Calibration complete: Accel Bias={self.accel_bias}, Gyro Bias={self.gyro_bias}")
        self.calibration_completed = True

    def apply_complementary_filter(self, raw_value, filtered_value, bias):
        """Применение complementary filter"""
        return self.alpha * (filtered_value + (raw_value - bias) * 0.01) + (1 - self.alpha) * raw_value

    def timer_callback(self):
        """Callback для публикации данных"""
        try:
            if not self.calibration_completed:
                return  # Ждем завершения калибровки

            # Чтение данных
            self.read_acceleration()
            self.read_gyroscope()

            # Применяем калибровку
            self.ax -= self.accel_bias[0]
            self.ay -= self.accel_bias[1]
            self.az -= self.accel_bias[2]

            self.gx -= self.gyro_bias[0]
            self.gy -= self.gyro_bias[1]
            self.gz -= self.gyro_bias[2]

            # Применяем complementary filter
            self.filtered_ax = self.apply_complementary_filter(self.ax, self.filtered_ax, self.accel_bias[0])
            self.filtered_ay = self.apply_complementary_filter(self.ay, self.filtered_ay, self.accel_bias[1])
            self.filtered_az = self.apply_complementary_filter(self.az, self.filtered_az, self.accel_bias[2])

            self.filtered_gx = self.apply_complementary_filter(self.gx, self.filtered_gx, self.gyro_bias[0])
            self.filtered_gy = self.apply_complementary_filter(self.gy, self.filtered_gy, self.gyro_bias[1])
            self.filtered_gz = self.apply_complementary_filter(self.gz, self.filtered_gz, self.gyro_bias[2])

        except OSError:
            self.get_logger().error("Failed to read data from MPU6050")
            return

        # Создаем сообщение Imu
        msg = Imu()
        msg.header.frame_id = "imu_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Заполняем линейное ускорение (с учетом фильтрации)
        msg.linear_acceleration.x = self.filtered_ax
        msg.linear_acceleration.y = self.filtered_ay
        msg.linear_acceleration.z = self.filtered_az

        # Заполняем угловую скорость (с учетом фильтрации)
        msg.angular_velocity.x = self.filtered_gx
        msg.angular_velocity.y = self.filtered_gy
        msg.angular_velocity.z = self.filtered_gz

        # Ориентация (пустая, так как MPU6050 её не предоставляет)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # Публикуем сообщение
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()