import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        
        TimerAction(
            period=1.0,
            actions=[
        Node(
            package='sensors',
            executable='unix_timestamp',
            output='screen'
        ),
            ]
        ),
        # UWB node (starts 2 seconds after launch)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='sensors',
                    executable='uwb_node',
                    name='uwb_node',
                    output='screen',
                    parameters=[{
                        'udp_ip': '192.168.1.100',     # UWB device IP
                        'udp_port': 9998,               # UWB device port
                        'listen_port': 9999,            # local port for incoming CIR data
                        'cirinterval': 4000,            # pulse repetition interval in us (1ms..1s)
                        'cirtaps': 64,                  # number of CIR taps (do not change)
                        'frequency': 6500000,           # Tx frequency in KHz (6.4GHz-8GHz)
                        'ciroffset': 0,                 # CIR offset in taps (0..1015)
                        'mode': 1,                      # Bit0=1: correlated data mode
                        'dccompfiltercoef': 1,          # DC comp filter coef (0x01=k=0.01 .. 0x63=k=0.99)
                    }]
                )
            ]
        ),
        # (Optional) UWB test sender (starts 3 seconds after launch)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='sensors',
                    executable='uwb_test_sender',
                    output='screen'
                )
            ]
        ),
    ])