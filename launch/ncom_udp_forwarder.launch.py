from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the NCOM UDP forwarder node.
    Allows configuration of the NCOM topic, UDP address, and port via launch arguments.
    """
    # Declare the launch arguments that can be passed from the command line
    declare_ncom_topic_arg = DeclareLaunchArgument(
        'ncom_topic',
        default_value='/oxts/ncom',
        description='The topic that publishes oxts_msgs/msg/Ncom.'
    )
    
    declare_udp_address_arg = DeclareLaunchArgument(
        'udp_address',
        default_value='127.0.0.1',
        description='The destination IP address for the UDP packets.'
    )

    declare_udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='3000',
        description='The destination UDP port for the UDP packets.'
    )

    # Configure the node to launch
    ncom_udp_forwarder_node = Node(
        package='ncom_udp_forwarder',
        executable='ncom_forwarder_node',
        name='ncom_udp_forwarder',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # Pass the launch arguments to the node's parameters
            'ncom_topic': LaunchConfiguration('ncom_topic'),
            'udp_address': LaunchConfiguration('udp_address'),
            'udp_port': LaunchConfiguration('udp_port')
        }]
    )

    # Create the launch description and add the actions
    return LaunchDescription([
        declare_ncom_topic_arg,
        declare_udp_address_arg,
        declare_udp_port_arg,
        ncom_udp_forwarder_node
    ])


