#include "main_gait.hpp"
#include "tripod_gait.hpp"
#include "hexa_ik.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include "sensor_msgs/msg/joy.hpp"

class TripodGaitNode : public rclcpp::Node, public std::enable_shared_from_this<TripodGaitNode> {
public:
    TripodGaitNode()
        : Node("main_gait_node"),
          tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "MAIN Gait Node started");

        // Публикация углов суставов
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);


          // Подписка на топик /joy
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
         "/joy", 10, 
         std::bind(&TripodGaitNode::joy_callback, this, std::placeholders::_1));
  

        
        // Подписка на cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TripodGaitNode::cmd_vel_callback, this, std::placeholders::_1));


        // Таймер для выполнения обработки
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TripodGaitNode::update_tripod_gait, this));

        // Инициализация времени для синусоиды
        start_time_ = this->get_clock()->now();
    }

private:
    // TF2 компоненты
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Публикация состояний суставов
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // Подписка на /cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Хранилище текущего состояния суставов
    JointState joint_state_;

    // Глобальная переменная для геометрии гексапода
    const HexapodGeometry HEXAPOD_GEOMETRY;


 
    // Время старта для синусоиды
    rclcpp::Time start_time_;

    // Массивы для хранения линейных и угловых скоростей
    std::array<double, 3> linear_vel = {0.0, 0.0, 0.0};
    std::array<double, 3> angular_vel = {0.0, 0.0, 0.0};
    // double linear_vel[3];  // [x, y, z] линейные скорости
    // double angular_vel[3]; // [x, y, z] угловые скорости

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // Вывод значений осей

        float axis_value = msg->axes[4];
        RCLCPP_INFO(this->get_logger(), "Axis value: %f", static_cast<double>(axis_value));

        linear_vel[0] = msg->axes[3]*0.1; // линейная скорость по оси X
        linear_vel[1] = msg->axes[2]*0.1; // линейная скорость по оси Y
        linear_vel[2] = 0.0; // линейная скорость по оси Z

        angular_vel[0] = 0.0; // угловая скорость по оси X
        angular_vel[1] = 0.0; // угловая скорость по оси Y
        angular_vel[2] =msg->axes[0]*0.1; // угловая скорость по оси Z

        // RCLCPP_INFO(this->get_logger(), "Axes: ");
        // for (size_t i = 0; i < msg->axes.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "  Axis %zu: %f", i, msg->axes[i]);
        // }

        // // Вывод значений кнопок
        // RCLCPP_INFO(this->get_logger(), "Buttons: ");
        // for (size_t i = 0; i < msg->buttons.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "  Button %zu: %d", i, msg->buttons[i]);
        // }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;


    
    // Callback для обработки команд скорости
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Обновление линейных и угловых скоростей из полученного сообщения
        // linear_vel[0] = msg->linear.x; // линейная скорость по оси X
        // linear_vel[1] = msg->linear.y; // линейная скорость по оси Y
        // linear_vel[2] = msg->linear.z; // линейная скорость по оси Z

        // angular_vel[0] = msg->angular.x; // угловая скорость по оси X
        // angular_vel[1] = msg->angular.y; // угловая скорость по оси Y
        // angular_vel[2] = msg->angular.z; // угловая скорость по оси Z

        // Логирование полученных значений
        // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear[x=%.2f, y=%.2f, z=%.2f], angular[x=%.2f, y=%.2f, z=%.2f]",
        //             linear_vel[0], linear_vel[1], linear_vel[2],
        //             angular_vel[0], angular_vel[1], angular_vel[2]);
    }


    // Основной цикл обработки данных
    void update_tripod_gait() {
        // Текущее время
        rclcpp::Time current_time = this->get_clock()->now();
        double elapsed_time = (current_time - start_time_).seconds();

        double cycle_time = 1.0;   // Время полного цикла (быстрее или медленнее шаг)
        double step_height = 0.07; // Высота шага


        // Вызов тактирующей функции
        std::vector<double> timing = generate_cubic_timing(elapsed_time, cycle_time);

        // Распределяем фазы между ногами
        std::vector<double> leg_phases = gait_phases_tripod(timing[0], {0, 2, 4}, {1, 3, 5});



        // Генерация траекторий для каждой ноги
        for (size_t i = 0; i < 6; ++i) {

            std::array<double, 4> step = generate_arc_and_line(leg_phases[i], step_height, linear_vel, angular_vel[2]);

            std::array<double, 4> ik_pos = tripod_transform_line(i, step, LEG_ANGLES, X_OFFSET, Y_OFFSET, linear_vel);
           
            std::array<double, 4> ik_rot = tripod_transform_rot(i, step, LEG_ANGLES, X_OFFSET, Y_OFFSET, angular_vel);

            // Вычисление IK
            const auto& leg_params = HEXAPOD_GEOMETRY.legs[i];
            // IKResult ik_result = calculate_leg_ik(ik_pos[0]+ik_rot[0], ik_pos[1]+ ik_rot[1], ik_pos[2], leg_params);
            IKResult ik_result = calculate_leg_ik(ik_pos[0], ik_pos[1], ik_pos[2], leg_params);

            if (ik_result.success) {
                joint_state_.positions[i * 3] = ik_result.joint1_angle;
                joint_state_.positions[i * 3 + 1] = ik_result.joint2_angle;
                joint_state_.positions[i * 3 + 2] = ik_result.joint3_angle;
            } else {
                RCLCPP_WARN(this->get_logger(), "IK failed for leg %zu", i + 1);
            }
        }

        // Вывод тайминга для отладки
        // RCLCPP_INFO(this->get_logger(), "Gait phase timing: %f", timing[0]);

        // Публикация состояния суставов
        publish_joint_states();
    }

    // Генерация тактирующего сигнала на основе полинома 3-й степени (cubic easing) с замедлением к 0.5 и ускорением с 0.5
    std::vector<double> generate_cubic_timing(
        double elapsed_time, 
        double cycle_time) {

        std::vector<double> timing;

        // Нормализованное время в пределах цикла [0, 1)
        double normalized_time = std::fmod(elapsed_time, cycle_time) / cycle_time;

        // Ускорение и замедление: разные фазы
        double gait_phase;
        if (normalized_time < 0.5) {
            double t = normalized_time * 2.0; // Приводим к диапазону 0–1 для первой половины
            gait_phase = 0.5 * (3 * t * t - 2 * t * t * t); // S-образное замедление к 0.5
        } else {
            double t = (normalized_time - 0.5) * 2.0; // Приводим к диапазону 0–1 для второй половины
            gait_phase = 0.5 + (0.5 * (3 * t * t - 2 * t * t * t)); // S-образное ускорение с 0.5
        }

        timing.push_back(gait_phase);

        // Вывод значений тайминга в порт (консоль)
        // std::cout << "Elapsed time: " << elapsed_time 
        //         << ", Normalized time: " << normalized_time 
        //         << ", Gait phase: " << gait_phase << std::endl;

        return timing;
    }


    // Генерация траектории дуги и прямой для одной ноги
    std::array<double, 4> generate_arc_and_line(
        double gait_phase,
        double step_height,
        const std::array<double, 3>& direction,
        double rotation
        ) {

        std::array<double, 4> result = {0.0, 0.0, 0.0, 0.0};


        if (std::abs(direction[0])+ std::abs(direction[1]) +  std::abs(rotation) > 0.01) {
            if (gait_phase < 0.5) {
            double t = gait_phase * 2.0;
            result[0] = t * direction[0];       // Прогресс по X
            result[1] = t * direction[1];       // Прогресс по Y
            result[3] = t * rotation;       // Прогресс по Y

            result[2] = step_height * t * (1 - t); // Парабола для подъема
            } else {
                double t = (gait_phase - 0.5) * 2.0;
                result[0] = (1.0 - t) * direction[0]; // Линейное возвращение по X
                result[1] = (1.0 - t) * direction[1]; // Линейное возвращение по Y
                result[3] = (1.0 - t) * rotation; // Линейное возвращение по Y
                
                result[2] = 0.0; // Опускаем ногу на землю
            }
        }

        return result;
    }

    // Функция публикации состояний суставов
    void publish_joint_states() {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.name = JOINT_NAMES;
        joint_state_msg.position = joint_state_.positions;

        joint_state_pub_->publish(joint_state_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TripodGaitNode>());
    rclcpp::shutdown();
    return 0;
}
