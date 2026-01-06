# launch/ros_module.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the ROS module.

    Returns:
        LaunchDescription: The launch description.
    """
    # Create the launch description
    ld = LaunchDescription()

    # Add the UI input node
    ld.add_action(
        Node(
            package='ros_module',
            executable='ui_input_node',
            name='ui_input_node',
            output='screen'
        )
    )

    # Add the LLM receiver node
    ld.add_action(
        Node(
            package='ros_module',
            executable='llm_receiver_node',
            name='llm_receiver_node',
            output='screen'
        )
    )

    # Add the simulator node
    ld.add_action(
        Node(
            package='ros_module',
            executable='simulator_node',
            name='simulator_node',
            output='screen'
        )
    )

    return ld
