#  Copyright (c) 2020 robomaster-oss, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the MIT License, See the MIT License for more details.
#
#  You should have received a copy of the MIT License along with this program.
#  If not, see <https://opensource.org/licenses/MIT/>.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xmacro.xmacro4sdf import XMLMacro4sdf

def generate_launch_description():
    ld = LaunchDescription()
    pkg_rmua19_ignition_simulator = get_package_share_directory('rmua19_ignition_simulator')
    pkg_rmuc21_ignition_simulator = get_package_share_directory('rmuc21_ignition_simulator')
    robot_xmacro_path = os.path.join(pkg_rmua19_ignition_simulator, 'resource', 'models', 'rmua19_standard_robot_b','model.sdf.xmacro')
    world_sdf_path = os.path.join(pkg_rmuc21_ignition_simulator, 'resource', 'worlds', 'rmuc21_empty_world.sdf')
    ign_config_path = os.path.join(pkg_rmuc21_ignition_simulator, 'ign', 'gui.config')
    # launch Gazebo World (contain 2 standard robot)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -v 4 --gui-config ' + ign_config_path,
        }.items()
    )
    ld.add_action(gazebo)
    # robot names
    robot_names=["standard_robot_red1","standard_robot_blue1"]
    # Spawn robot
    robot_macro = XMLMacro4sdf()
    robot_macro.set_xml_file(robot_xmacro_path)
    robot_macro.generate({"global_initial_color":"red"})
    robot_xml = robot_macro.to_string()
    spawn1 = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', robot_names[0] ,'-x', '-11','-y', '0','-z', '0.20', '-string', robot_xml],
        output='screen')
    ld.add_action(spawn1)
    robot_macro.generate({"global_initial_color":"blue"})
    robot_xml = robot_macro.to_string()
    spawn2 = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', robot_names[1] ,'-x', '11','-y', '0','-z', '0.20', '-Y', '3.14159', '-string', robot_xml],
        output='screen')
    ld.add_action(spawn2)
    # robot base for each robot
    for robot_name in robot_names:
        robot_base =Node(package='rmua19_ignition_simulator', executable='rmua19_robot_base',
            namespace= robot_name+"/robot_base",
            parameters=[
                {"world_name": "default"},
                {"robot_name": robot_name},
            ],
            output='screen') 
        robot_ign_bridge = Node(package='ros_ign_bridge',executable='parameter_bridge',
            namespace= robot_name,
            arguments=["/world/default/model/%s/link/front_industrial_camera/sensor/front_industrial_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image"%(robot_name),
                    "/world/default/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"%(robot_name),
                    "/%s/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"%(robot_name)
            ],
            remappings=[
                ("/world/default/model/%s/link/front_industrial_camera/sensor/front_industrial_camera/image"%(robot_name),"front_camera/image"),
                ("/world/default/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan"%(robot_name),"rplidar_a2/scan"),
                ("/%s/odometry"%(robot_name),"robot_base/odom"),
            ],
            output='screen'
        )
        ld.add_action(robot_base)
        ld.add_action(robot_ign_bridge)
    #referee system
    referee_ign_bridge = Node(package='ros_ign_bridge',executable='parameter_bridge',
        namespace="referee_system",
        arguments=["/referee_system/attack_info@std_msgs/msg/String[ignition.msgs.StringMsg",
                   "/referee_system/shoot_info@std_msgs/msg/String[ignition.msgs.StringMsg",
                ],
        output='screen'
    )
    ld.add_action(referee_ign_bridge)
    return ld
