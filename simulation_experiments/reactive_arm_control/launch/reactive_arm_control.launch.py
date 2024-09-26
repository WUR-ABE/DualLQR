import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    rviz_description_package = LaunchConfiguration("rviz_description_package")
    launch_servo = LaunchConfiguration("launch_servo")

    # Set default arguments
    ur_type_launch_arg = DeclareLaunchArgument(
        "ur_type", 
        default_value="ur5e", 
        description="Type of ur robot", 
        choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
    )
    robot_ip_launch_arg = DeclareLaunchArgument("robot_ip", default_value="www.xxx.yyy.zzz", description="IP address of robot")  # Sim robot ip
    launch_rviz_launch_arg = DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    description_package_launch_arg = DeclareLaunchArgument(
        "description_package", default_value="ur_description", description="ROS package containg robot description"
    )
    description_file_launch_arg = DeclareLaunchArgument(
        "description_file", default_value="ur.urdf.xacro", description="URDF file containting robot description"
    )
    moveit_config_package_launch_arg = DeclareLaunchArgument(
        "moveit_config_package", default_value="ur_moveit_config", description="ROS package containing MoveIt configuration"
    )
    moveit_config_file_launch_arg = DeclareLaunchArgument(
        "moveit_config_file", default_value="ur.srdf.xacro", description="SRDF file containting MoveIt configuration"
    )
    prefix_launch_arg = DeclareLaunchArgument("prefix", default_value="", description="Prefix for robot name")
    activate_joint_controller_launch_arg = DeclareLaunchArgument(
        "activate_joint_controller",
        default_value="true",
        description="Whether to activate the initially loaded joint controller.",
    )
    initial_joint_controller_launch_arg = DeclareLaunchArgument(
        "initial_joint_controller",
        default_value="forward_position_controller",
        description="Name of the initially loaded joint controller.",
    )
    use_fake_hardware_arg = DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use fake hardware?")
    rviz_description_package_launch_arg = DeclareLaunchArgument(
        "rviz_description_package", default_value="reactive_arm_control", description="ROS package containing robot description for RViz")
    launch_servo_launch_arg = DeclareLaunchArgument(
        "launch_servo",
        default_value="false",
        description="Launch MoveIt Servo?")
    

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )
        
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("reactive_arm_control", "config/ur5e_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    #### UR5e launching
    ur_robot_driver_dir = get_package_share_directory("reactive_arm_control")
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ur_robot_driver_dir, "/launch/ur_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": launch_rviz,
            "activate_joint_controller": activate_joint_controller,
            "initial_joint_controller": initial_joint_controller,
            "use_fake_hardware": use_fake_hardware,
            "rviz_description_package": rviz_description_package,
        }.items(),
    )

    # Moveit servo
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            ur_type_launch_arg,
            robot_ip_launch_arg,
            launch_rviz_launch_arg,
            description_package_launch_arg,
            description_file_launch_arg,
            moveit_config_package_launch_arg,
            moveit_config_file_launch_arg,
            prefix_launch_arg,
            activate_joint_controller_launch_arg,
            initial_joint_controller_launch_arg,
            rviz_description_package_launch_arg,
            launch_servo_launch_arg,
            use_fake_hardware_arg,
            ur_control_launch,
            servo_node,
        ]
    )
