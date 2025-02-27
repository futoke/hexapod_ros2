
# вот мой текущий лаунч, поправь его 
    
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Путь к пакету и URDF
    pkgPath = launch_ros.substitutions.FindPackageShare(package='hexa_description').find('hexa_description')
    urdfModelPath = os.path.join(pkgPath, 'urdf/hexa.urdf')

    # Чтение содержимого URDF в параметр
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    # Параметры для robot_state_publisher
    params = {'robot_description': robot_desc,
              'use_sim_time': False}

    # Узел для публикации состояний робота
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    # Узел для публикации состояний суставов (обычный)
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    # Узел для публикации состояний суставов (с GUI)
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # # Узел для запуска RViz без файла конфигурации
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',

    # )

    # Узел для запуска RViz с указанием файла конфигурации
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            launch_ros.substitutions.FindPackageShare('hexa_utils').find('hexa_utils'),
            'rviz',
            'hexa_config.rviz'
        )]
    )


    # Узел походки
    tripod_gait_node = launch_ros.actions.Node(
        package='hexa_ik',
        executable='main_gait',
        output='screen'
    )

        # Узел для джойстика (PS4)
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        # parameters=[{'device': '/dev/input/js0'}]  # Убедись, что указываешь правильное устройство
    )

    # Возвращаем описание запуска
    return launch.LaunchDescription([
        # Аргумент для включения/выключения GUI
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='This is a flag for joint_state_publisher_gui'
        ),
        # Аргумент для указания пути к модели URDF
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=urdfModelPath,
            description='Path to the urdf model file'
        ),
        # Узлы
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        
        
        rviz_node,
        tripod_gait_node, 
        joy_node,
    ])