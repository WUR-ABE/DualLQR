from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    publish_rate = LaunchConfiguration("publish_rate")
    tf_object_topic = LaunchConfiguration("tf_object_topic")
    tf_manipulator_topic = LaunchConfiguration("tf_manipulator_topic")
    tf_origin_topic = LaunchConfiguration("tf_origin_topic")
    controller_type = LaunchConfiguration("controller_type")
    data_folder = LaunchConfiguration("data_folder")

    # Nodes
    traj_executor_reactive = Node(
        package="lfd_executor",
        executable="traj_executor_reactive",
        name="traj_executor_reactive",
        parameters=[
            {
                "publish_rate": publish_rate,
                "tf_object_topic": tf_object_topic,
                "tf_manipulator_topic": tf_manipulator_topic,
                "tf_origin_topic": tf_origin_topic,
                "controller_type": controller_type,
                "data_folder": data_folder,
            }
        ],
    )
    controlled_goal_pose_broadcaster = Node(
        package="lfd_executor",
        executable="controlled_goal_pose_broadcaster",
        name="controlled_goal_pose_broadcaster",
    )
    dynamic_base_frame_broadcaster = Node(
        package="lfd_executor",
        executable="dynamic_base_frame_broadcaster",
        name="dynamic_base_frame_broadcaster",
    )

    nodes = [
        traj_executor_reactive,
        controlled_goal_pose_broadcaster,
        dynamic_base_frame_broadcaster,
    ]

    return nodes


def generate_launch_description():
    # Set default arguments
    publish_rate_launch_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="125.0",
        description="Publish rate of the node",
    )
    tf_object_topic_launch_arg = DeclareLaunchArgument(
        "tf_object_topic",
        default_value="cube_ik",
        description="Topic of the object tf",
    )
    tf_manipulator_topic_launch_arg = DeclareLaunchArgument(
        "tf_manipulator_topic",
        default_value="tool_ik",
        description="Topic of the manipulator tf",
    )
    tf_origin_topic_launch_arg = DeclareLaunchArgument(
        "tf_origin_topic",
        default_value="relaxed_ik",
        description="Topic of the origin tf",
    )
    controller_type_launch_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="SingleLQR",
        description="Type of controller to use",
    )
    data_folder_launch_arg = DeclareLaunchArgument(
        "data_folder",
        default_value="/media/data/OneDrive/PhD/03 Study 2/03 Software/trajectories/",
        description="Folder to save data",
    )

    declared_arguments = [
        publish_rate_launch_arg,
        tf_object_topic_launch_arg,
        tf_manipulator_topic_launch_arg,
        tf_origin_topic_launch_arg,
        controller_type_launch_arg,
        data_folder_launch_arg,
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
