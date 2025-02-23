#ifndef MAIN_GAIT_HPP
#define MAIN_GAIT_HPP

#include <string>
#include <vector>

// Константы: Имена суставов робота
const std::vector<std::string> JOINT_NAMES = {
            "leg1_link1_joint", "leg1_link2_joint", "leg1_link3_joint", 
            "leg2_link1_joint", "leg2_link2_joint", "leg2_link3_joint", 
            "leg3_link1_joint", "leg3_link2_joint", "leg3_link3_joint", 
            "leg4_link1_joint", "leg4_link2_joint", "leg4_link3_joint", 
            "leg5_link1_joint", "leg5_link2_joint", "leg5_link3_joint", 
            "leg6_link1_joint", "leg6_link2_joint", "leg6_link3_joint",
        };

// // Константы: Имена эффекторов ног
// const std::vector<std::string> GROUP_A = {
//     "leg1_link4", "leg3_link4","leg5_link4", 
// };
// const std::vector<std::string> GROUP_B = {
//    "leg2_link4","leg4_link4", "leg6_link4"
// };
// const std::vector<std::string> GROUP_B = {
//     "leg1_link4", "leg2_link4", "leg3_link4",
//     "leg4_link4", "leg5_link4", "leg6_link4"
// };

// Структура для представления состояния суставов
struct JointState {
    std::vector<double> positions; // Углы суставов
    std::vector<double> velocities; // Скорости суставов (опционально)
    std::vector<double> efforts;   // Усилия в суставах (опционально)

    JointState() {
        positions.resize(JOINT_NAMES.size(), 0.0);
        velocities.resize(JOINT_NAMES.size(), 0.0);
        efforts.resize(JOINT_NAMES.size(), 0.0);
    }
};

// Структура для хранения позиции эффектора относительно тела
struct EffectorPosition {
    double x;
    double y;
    double z;
};

#endif // MAIN_GAIT_HPP


