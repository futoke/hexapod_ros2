#include "tripod_gait.hpp"



// Генерация тактирующего сигнала на основе полинома 3-й степени (cubic easing) с замедлением к 0.5 и ускорением с 0.5

double generate_cubic_timing(
    double elapsed_time, 
    double cycle_time) {

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

    return gait_phase;
}

// // Функция генерации тактирования с использованием времени от старта
// double generate_cubic_timing(double elapsed_time, double cycle_time) {

//     // Нормализованное время в пределах цикла [0, 1)
//     double normalized_time = elapsed_time / cycle_time;

//     // Ограничиваем нормализованное время в пределах от 0 до 1 (в случае переполнения)
//     normalized_time = normalized_time - std::floor(normalized_time);

//     // С-образное замедление и ускорение: разные фазы
//     double gait_phase;
//     if (normalized_time < 0.5) {
//         double t = normalized_time * 2.0; // Приводим к диапазону 0–1 для первой половины
//         gait_phase = 0.5 * (3 * t * t - 2 * t * t * t); // S-образное замедление к 0.5
//     } else {
//         double t = (normalized_time - 0.5) * 2.0; // Приводим к диапазону 0–1 для второй половины
//         gait_phase = 0.5 + (0.5 * (3 * t * t - 2 * t * t * t)); // S-образное ускорение с 0.5
//     }

//     return gait_phase;
// }


// Генерация траектории дуги и прямой для одной ноги
std::array<double, 6> generate_arc_and_line(
    double gait_phase,
    double step_height,
    const std::array<double, 3>& direction,
    double rotation
    ) {

    std::array<double, 6> result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    if (std::abs(direction[0])+ std::abs(direction[1]+rotation) > 0.001) {
        if (gait_phase < 0.5) {
        double t = gait_phase * 2.0;
        result[0] = t * direction[0];       // Прогресс по X
        result[1] = t * direction[1];       // Прогресс по Y
        result[2] = step_height * t * (1 - t); // Парабола для подъема

        result[3] = t * rotation; 
        } else {
            double t = (gait_phase - 0.5) * 2.0;
            result[0] = (1.0 - t) * direction[0]; // Линейное возвращение по X
            result[1] = (1.0 - t) * direction[1]; // Линейное возвращение по Y
            result[2] = 0.0; // Опускаем ногу на землю

            result[3] = (1.0 - t) * rotation; 
        }
    }

    return result;
}
std::array<double, 6> gait_phases_tripod(double global_phase) {

    std::array<size_t, 3> tripod1 = {0, 2, 4};
    std::array<size_t, 3> tripod2 = {1, 3, 5};
    std::array<double, 6> leg_phases = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    // Присваиваем фазы для первой группы
    for (size_t leg : tripod1) {
        leg_phases[leg] = global_phase; // Глобальная фаза для первой группы
    }

    // Присваиваем фазы для второй группы
    for (size_t leg : tripod2) {
        leg_phases[leg] = std::fmod(global_phase + 0.5, 1.0); // Смещение на 0.5 для второй группы
    }

    return leg_phases;
}

// leg number для какой ноги
// arc дуга сгенерированная для шага
// LEG_ANGLES разворот ног относительно тела
// X_OFFSET как далеко от центра тела генерируется дуга 
// direction вектор напрвления 
std::array<double, 6> tripod_transform_line(
    int leg_number, 
    const std::array<double, 6>& arc, 
    const std::array<double, 6>& LEG_ANGLES, 
    const std::array<double, 6>& X_OFFSET, 
    const std::array<double, 3>& direction,
    const double rotatation
    ){
    std::array<double, 6> result = arc; // Копируем исходное положение
    result[0]= result[0]-(direction[0]/2);
    result[1]= result[1]-(direction[1]/2);

    // Инверсия направления движения для 1-й и 4-й ноги
    if (leg_number == 1 || leg_number == 4) {
        result[0] = -result[0]; // Разворачиваем направление движения
        result[1] = -result[1]; // Разворачиваем направление движения
    }

    // Применяем поворот вектора траектории относительно локальной оси ноги
    double angle = LEG_ANGLES[leg_number]; // Угол текущей ноги
    double rotated_x = result[0] * std::cos(angle) - result[1] * std::sin(angle);
    double rotated_y = result[0] * std::sin(angle) + result[1] * std::cos(angle);

    // Коррекция с учетом смещений
    rotated_x += X_OFFSET[leg_number];
    rotated_y += result[3]; //
    result[2] -= 0.07; // Опускаем ногу на землю

    // Записываем в результат
    result[0] = rotated_x;
    result[1] = rotated_y;

    return result;
}





std::array<double, 3> tripod_transform_rot(
    int leg_number, 
    const std::array<double, 3>& pos, 
    const double rotation
    ){
    std::array<double, 3> result = pos; // Копируем исходное положение
    double rotated_y = (pos[3]);
    result[1] = rotated_y;

    return result;
}