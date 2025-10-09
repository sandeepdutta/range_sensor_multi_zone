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
                                     'ranging_frequency_hz': 15,
                                     'max_height': 0.15,
                                     'min_height': 0.0,
                                     'min_distance': 250,
                                     'max_distance': 1000,
                                     'range_sigma_threshold': 100,
                                     'radius_outlier_enabled': True,
                                     'radius_outlier_radius': 0.1,
                                     'radius_outlier_min_neighbors': 15,
                                     'temporal_filter_enabled': True,
                                     'temporal_filter_size': 2,
                                     'sensor_mask': 0x00, # 0x00 = all sensors enabled, 0xaa corner sensors enabled
                                     'horizontal_fov': 11.25, #45.0, # degrees
                                     'vertical_fov': 45.0, # degrees
                                     'sharpener_percent': 25, # 
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