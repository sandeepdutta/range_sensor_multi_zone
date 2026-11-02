# launch file for the range sensor publisher

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='range_sensor_multi_zone', 
                        executable='range_sensor_multi_zone', 
                        name='range_sensor_multi_zone',
                        output='screen',
                        #prefix = ['xterm -e gdb -ex run --args'],
                        remappings=[('odom', 'odometry/filtered_local')],
                        parameters=[{'i2c_adapter_nr': 7,
                                     'num_sensors': 8,
                                     'resolution': 8, # 8x8, 4 = 4x4
                                     'ranging_frequency_hz': 15,
                                     'max_height': 0.25, # 150mm = 0.15m
                                     'min_height': -0.05, # 0mm = 0m
                                     'min_distance': 2, # 2mm = 0.002m
                                     'max_distance': 1000, # 1000mm = 1.0m
                                     'range_sigma_percent_threshold': 1.1, # reject if sigma > 1.1% of distance
                                     'spad_history_enabled': False,
                                     'spad_history_size' : 5,   # total size
                                     'spad_history_min_valid_count' : 3, # 
                                     'spad_history_tolerance_mm': 5.0,
                                     'radius_outlier_enabled': True,
                                     'radius_outlier_radius': 0.15,
                                     'radius_outlier_min_neighbors': 1,
                                     'temporal_filter_enabled': False,
                                     'temporal_filter_alpha': 0.9, # new reading 90%
                                     'temporal_filter_size': 2,
                                     'sensor_mask': 0x00, # 0x00 = all sensors enabled, 0xaa corner sensors enabled
                                     'horizontal_fov':45.0,  # degrees
                                     'vertical_fov': 45.0, # degrees
                                     'sharpener_percent': 25, #
                                     'diag_verbose': False,
                                     'frame_ids': ['TOPV_LF_link', #0 bottom left front
                                                   'TOPV_FS_link', #1 bottom fron straight
                                                   'TOPV_RF_link', #2 bottom right front
                                                   'TOPV_RS_link', #3 bottom right straight
                                                   'TOPV_RB_link', #4 bottom right back
                                                   'TOPV_BS_link', #5 bottom back straight
                                                   'TOPV_LB_link', #6 bottom left back
                                                   'TOPV_LS_link']}] #7 bottom left straight
                                )])