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
                        parameters=[{'i2c_adapter_nr': 1,
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
                                     'frame_ids': ['BOT_LF_TOF', #0 bottom left front
                                                   'BOT_FS_TOF', #1 bottom fron straight
                                                   'BOT_RF_TOF', #2 bottom right front
                                                   'BOT_RS_TOF', #3 bottom right straight
                                                   'BOT_RB_TOF', #4 bottom right back
                                                   'BOT_BS_TOF', #5 bottom back straight
                                                   'BOT_LB_TOF', #6 bottom left back
                                                   'BOT_LS_TOF']}] #7 bottom left straight
                                )])