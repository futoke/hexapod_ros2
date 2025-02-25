#ifndef TRIPOD_GAIT_HPP
#define TRIPOD_GAIT_HPP

#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <rclcpp/rclcpp.hpp>


constexpr std::array<double, 6> LEG_ANGLES = {
    0.523598, -1.570796, 2.617994,
    -2.617994, 1.570796, -0.523599
};

constexpr std::array<double, 6> X_OFFSET = {
    0.18,
    0.18,
    0.18,
    0.18,
    0.18,
    0.18,
};
constexpr std::array<double, 6> Y_OFFSET = {
    -1.0,
    -2.0,
    1.0,
    -1.0,
    2.0,
    1.0,
};

// // Генерация тактирующего сигнала на основе полинома 3-й степени (cubic easing) с замедлением к 0.5 и ускорением с 0.5
double generate_cubic_timing(double elapsed_time, double cycle_time);

// Генерация траектории дуги и прямой для одной ноги
std::array<double, 6> generate_arc_and_line(
    double gait_phase,
    double step_height,
    const std::array<double, 3>& direction,
    double rotation
);

// Функция для генерации фаз для ног в триногом ходе
std::array<double, 6> gait_phases_tripod(double global_phase);

// Функция для трансформации позиции ноги с учетом углов и смещений
std::array<double, 6> tripod_transform_line(
    int leg_number, 
    const std::array<double, 6>& pos, 
    const std::array<double, 6>& LEG_ANGLES, 
    const std::array<double, 6>& X_OFFSET, 
    const std::array<double, 3>& direction,
    const double rotatation
    );
    // Функция для трансформации позиции ноги с учетом углов и смещений
std::array<double, 3> tripod_transform_rot(
    int leg_number, 
    const std::array<double, 3>& pos, 
    const double rotation
    );

#endif // TRIPOD_GAIT_HPP


