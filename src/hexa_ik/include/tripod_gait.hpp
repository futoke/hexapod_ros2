#ifndef TRIPOD_GAIT_HPP
#define TRIPOD_GAIT_HPP

#include <string>
#include <vector>
#include <array>


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

// Функция для генерации фаз для ног в триногом ходе
std::vector<double> gait_phases_tripod(
    double global_phase, 
    const std::vector<size_t>& tripod1, 
    const std::vector<size_t>& tripod2);

// Функция для трансформации позиции ноги с учетом углов и смещений
std::array<double, 4> tripod_transform_line(
    int i, 
    const std::array<double, 4>& pos, 
    const std::array<double, 6>& LEG_ANGLES, 
    const std::array<double, 6>& X_OFFSET, 
    const std::array<double, 6>& Y_OFFSET, 
    const std::array<double, 3>& direction);
    // Функция для трансформации позиции ноги с учетом углов и смещений
std::array<double, 4> tripod_transform_rot(
    int i, 
    const std::array<double, 4>& pos, 
    const std::array<double, 6>& LEG_ANGLES, 
    const std::array<double, 6>& X_OFFSET, 
    const std::array<double, 6>& Y_OFFSET, 
    const std::array<double, 3>& direction);

#endif // TRIPOD_GAIT_HPP


