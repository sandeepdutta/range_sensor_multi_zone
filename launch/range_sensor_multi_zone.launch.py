# launch file for the range sensor publisher

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='range_sensor_multi_zone', 
                        executable='range_sensor_multi_zone', 
                        name='range_sensor_multi_zone',
                        output='screen',
                        #prefix = ['xterm -e gdb -ex run --args'],
                        parameters=[{'i2c_adapter_nr': 7,
                                     'num_sensors': 8,
                                     'resolution': 8, # 8x8, 4 = 4x4
                                     'ranging_frequency_hz': 10,
                                     'diag_verbose': False,
                                     'max_height': 0.075,
                                     'min_height': -0.2,
                                     'min_distance': 50,
                                     'max_distance': 1000,
                                     'radius_outlier_radius': 0.1,
                                     'radius_outlier_min_neighbors': 12,
                                     'sensor_mask': 0x00, # 0x00 = all sensors enabled, 0xaa corner sensors enabled
                                     'horizontal_fov': 15.0, # degrees
                                     'vertical_fov': 45.0, # degrees
                                     'diag_verbose': False,
                                     'frame_ids': ['RF_45_TOF',
                                                   'RF_90_TOF',
                                                   'LF_45_TOF',
                                                   'LF_90_TOF',
                                                   'LB_45_TOF',
                                                   'LB_90_TOF',
                                                   'RB_45_TOF',
                                                   'RB_90_TOF']}]
                                )])