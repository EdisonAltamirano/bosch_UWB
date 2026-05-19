from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sensors",
                executable="uwb_live_rdm_viewer",
                name="uwb_live_rdm_viewer",
                output="screen",
                parameters=[
                    {
                        "topic_name": "/uwb/frame_raw",
                        "refresh_period_s": 0.5,
                        "doppler_window_s": 4.0,
                        "display_mode": "auto",
                        "matplotlib_backend": "auto",
                    }
                ],
                additional_env={
                    "MPLCONFIGDIR": "/tmp/matplotlib",
                },
            ),
        ]
    )
