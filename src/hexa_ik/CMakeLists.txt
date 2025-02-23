cmake_minimum_required(VERSION 3.5)
project(hexa_ik)

# Найти необходимые пакеты
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)  # Основной пакет для работы с URDF
find_package(urdfdom_headers REQUIRED)  # Заголовочные файлы URDF

# Указать директории для заголовочных файлов
include_directories(include)

# Собрать исполняемый файл узла для походки
add_executable(main_gait
    src/main_gait.cpp
    src/hexa_ik.cpp
    src/tripod_gait.cpp
)
ament_target_dependencies(main_gait 
    rclcpp 
    tf2 
    tf2_geometry_msgs 
    geometry_msgs 
    std_msgs 
    sensor_msgs 
    urdf
)

# Установка исполняемых файлов
install(TARGETS
  main_gait
  DESTINATION lib/${PROJECT_NAME}
)

# Установка launch-файлов
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
