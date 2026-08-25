from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    two nodes here:
    - leader (w/ sine wave)
    - follower (w/ velocity cmd)
    """
    return LaunchDescription([
        Node(
            package='px4_sitl_sim',
            executable='x500_leader',
            output='screen',
            parameters=[{
                # --- ROS / MAVROS basics ---
                'ns': '/leader/mavros',
                'rate_hz': 20.0,
                'takeoff_alt': 5.0,
                'xy_speed': 1.5,          # planar speed along the curve (m/s)
                'warmup_sec': 2.0,
                'lock_yaw': True,         # keep nose fixed
                'yaw_offset_deg': 0.0,    # set 45.0 if you want “forward” visually NE

                # --- Sine shape (BODY frame) ---
                'amplitude_m': 8.0,       # ±8 m left/right
                'length_m': 9.0,          # ≤ 9 m forward
                'num_waves': 2,           # how many S-bends inside 9 m (tight if big)
                'phase_deg': 0.0,
                'lateral_bias_m': 0.0,    # shift right (+) or left (-) if needed

                # --- Curvature limiter (set to 0.0 to allow tight, “compressed” waves) ---
                # Set to some number to flatten the wave and give gentler turns
                # (so will get fewer bends over 9 m).
                'min_turn_radius_m': 0.0,
            }],
        ),
        Node(
            package='scout',
            executable='x500_follower',
            output='screen',
            parameters=[{
                # --- ROS / MAVROS basics ---
                'ns': '/follower/mavros',
                'rate_hz': 20.0,
                'takeoff_alt': 3.0,
                'xy_speed': 1.5,          # planar speed along the curve (m/s)
                'warmup_sec': 2.0,
                'lock_yaw': True,         # keep nose fixed
                'yaw_offset_deg': 0.0,    # set 45.0 if you want “forward” visually NE
                'is_gazebo': True,         # make sure this is off when doing experiment in the field
                'sim_offset': [4.0, 0.0, 0.0] # in follower {B} frame
            }],
        )
    ])
