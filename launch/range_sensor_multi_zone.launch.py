# launch file for the range sensor publisher

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                # x y z qx qy qz qw parent child
                                arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link']),
                            Node(package='range_sensor_multi_zone',
                               executable='range_sensor_multi_zone',
                               name='range_sensor_multi_zone',
                               output='screen',
                               #prefix=['gdbserver localhost:1234'],
                               parameters=[{'i2c_adapter_nr': 7,
                                           'num_sensors': 2,
                                           'timer_period': 500,
                                           'resolution': 8,
                                           'ranging_frequency_hz': 10,
                                           'frame_ids': [
                                               'base_link',
                                               'base_link']}])
                               ])
