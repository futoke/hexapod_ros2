#include "tripod_gait.hpp"
#include <cmath>
std::vector<double> gait_phases_tripod(
    double global_phase, 
    const std::vector<size_t>& tripod1, 
    const std::vector<size_t>& tripod2) {

    std::vector<double> leg_phases(6, 0.0); // Инициализация фаз для всех ног

    // Присваиваем фазы для первой группы
    for (const auto& leg : tripod1) {
        leg_phases[leg] = global_phase; // Глобальная фаза для первой группы
    }

    // Присваиваем фазы для второй группы
    for (const auto& leg : tripod2) {
        leg_phases[leg] = std::fmod(global_phase + 0.5, 1.0); // Смещение на 0.5 для второй группы
    }

    return leg_phases;
}

std::array<double, 4> tripod_transform_line(
    int i, 
    const std::array<double, 4>& pos, 
    const std::array<double, 6>& LEG_ANGLES, 
    const std::array<double, 6>& X_OFFSET, 
    const std::array<double, 6>& Y_OFFSET, 
    const std::array<double, 3>& direction) {
    
    std::array<double, 4> result = pos; // Копируем исходное положение
    result[0]= result[0]-(direction[0]/2);
    result[1]= result[1]-(direction[1]/2);

    // Инверсия направления движения для 1-й и 4-й ноги
    if (i == 1 || i == 4) {
        result[0] = -result[0]; // Разворачиваем направление движения
        result[1] = -result[1]; // Разворачиваем направление движения
    }

    // Применяем поворот вектора траектории относительно локальной оси ноги
    double angle = LEG_ANGLES[i]; // Угол текущей ноги
    double rotated_x = result[0] * std::cos(angle) - result[1] * std::sin(angle);
    double rotated_y = result[0] * std::sin(angle) + result[1] * std::cos(angle);

    // Коррекция с учетом смещений
    rotated_x += X_OFFSET[i];
    rotated_y += 0.0; //(direction[0]) / Y_OFFSET[i];
    result[2] -= 0.07; // Опускаем ногу на землю

    // Записываем в результат
    result[0] = rotated_x;
    result[1] = rotated_y;

    return result;
}

std::array<double, 4> tripod_transform_rot(
    int i, 
    const std::array<double, 4>& pos, 
    const std::array<double, 6>& LEG_ANGLES, 
    const std::array<double, 6>& X_OFFSET, 
    const std::array<double, 6>& Y_OFFSET, 
    const std::array<double, 3>& direction) {
    
    std::array<double, 4> result = pos; // Копируем исходное положение
    // result[0]= result[0]-(direction[0]/2);
    // result[1]= result[1]-(direction[1]/2);

    // Коррекция с учетом смещений
    // double rotated_x = 0;
    double rotated_y = (pos[3]);
    // result[2] -= 0.05; // Опускаем ногу на землю

    // Записываем в результат
    // result[0] = rotated_x;
    result[1] = rotated_y;

    return result;
}