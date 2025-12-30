from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package= 'imu_sensor_cpp',
            name= 'imu_node',
            executable= 'imu_node',
            output= 'screen',
            emulate_tty= True,
            parameters=[
                {
                    'alpha': 0.98,
                    'magnitude_low_threshold': 0.85,
                    'magnitude_high_threshold': 1.15,
                    'gimbal_lock_threshold': 0.97,
                    'beta': 0.1,
                    'R': [6.0, 6.0, 6.0],
                    'Q': [0.008, 0.008, 0.008, 0.008],
                    'mode': 'ekf',
                    'frame_id': 'imu_link',
                    'start_calibration': False,
                    'delete_calibration_data': False,
                    'gyro_deadzone': 0.02,
                }
            ]

        )
    ])