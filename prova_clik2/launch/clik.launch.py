from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()
   
    #cartesian_trajectory = Node(
       # package="prova_clik2",
        #executable="cartesian_trajectory",
        #output="screen",
       # parameters=[
           # moveit_config.robot_description,
           # moveit_config.robot_description_semantic,
           # moveit_config.robot_description_kinematics,
        #],
   # )
    clik_node = Node(
        package="prova_clik2",
        executable="clik",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )


    return LaunchDescription([
         clik_node#, cartesian_trajectory
        ])