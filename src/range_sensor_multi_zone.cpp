#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cmath>
#include <cerrno>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "range_sensor_multi_zone/range_sensor_multi_zone.h"
#include "range_sensor_multi_zone/vl53l5cx_api.h"

namespace range_sensor_multi_zone
{
    RangeSensorMultiZone::RangeSensorMultiZone()
    : Node("range_sensor_multi_zone")
    {
        i2c_adapter_nr_ = this->declare_parameter("i2c_adapter_nr", 7);
        num_sensors_    = this->declare_parameter("num_sensors", 8);
        diag_verbose_ = this->declare_parameter("diag_verbose", false);
        resolution_ = this->declare_parameter("resolution", 4);
        max_height_ = this->declare_parameter("max_height", 0.5);
        min_height_ = this->declare_parameter("min_height", 0.0);
        min_distance_ = this->declare_parameter("min_distance", 10);
        max_distance_ = this->declare_parameter("max_distance", 500);
        radius_outlier_enabled_ = this->declare_parameter("radius_outlier_enabled", true);
        radius_outlier_radius_ = this->declare_parameter("radius_outlier_radius", 0.1);
        radius_outlier_min_neighbors_ = this->declare_parameter("radius_outlier_min_neighbors", 3);
        temporal_filter_enabled_ = this->declare_parameter("temporal_filter_enabled", false);
        temporal_filter_size_ = this->declare_parameter("temporal_filter_size", 3);
        temporal_filter_alpha_ = this->declare_parameter("temporal_filter_alpha", 0.3);
        sensor_mask_ = this->declare_parameter("sensor_mask", 0); // 0 = all sensors enabled
        horizontal_fov_ = this->declare_parameter("horizontal_fov", 53.0); // degrees
        vertical_fov_ = this->declare_parameter("vertical_fov", 45.0); // degrees
        sharpener_percent_ = this->declare_parameter("sharpener_percent", 14); // default value from sensor
        range_sigma_threshold_ = this->declare_parameter("range_sigma_threshold", 20); // mm
        ranging_frequency_hz_ = this->declare_parameter("ranging_frequency_hz", 10);
        timer_period_ = 1000 / ranging_frequency_hz_; // in milliseconds
        // add 10% margin
        timer_period_ = (int) ((float)timer_period_ * 0.9);
        RCLCPP_INFO(this->get_logger(), "Timer period: %d ms", timer_period_);
        RCLCPP_INFO(this->get_logger(), "Max height: %f m", max_height_);
        RCLCPP_INFO(this->get_logger(), "Min height: %f m", min_height_);
        RCLCPP_INFO(this->get_logger(), "Min distance: %d mm", min_distance_);
        RCLCPP_INFO(this->get_logger(), "Radius outlier filter enabled: %s", radius_outlier_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Radius outlier radius: %f m", radius_outlier_radius_);
        RCLCPP_INFO(this->get_logger(), "Radius outlier min neighbors: %d", radius_outlier_min_neighbors_);
        RCLCPP_INFO(this->get_logger(), "Temporal filter enabled: %s", temporal_filter_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Temporal filter size: %d frames", temporal_filter_size_);
        RCLCPP_INFO(this->get_logger(), "Temporal filter alpha: %f", temporal_filter_alpha_);
        RCLCPP_INFO(this->get_logger(), "Sensor mask: 0x%02X", sensor_mask_);
        RCLCPP_INFO(this->get_logger(), "Horizontal FOV: %f degrees", horizontal_fov_);
        RCLCPP_INFO(this->get_logger(), "Vertical FOV: %f degrees", vertical_fov_);
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", resolution_, resolution_);
        RCLCPP_INFO(this->get_logger(), "Sharpener percent: %d%%", sharpener_percent_);
        RCLCPP_INFO(this->get_logger(), "Range sigma threshold: %d mm", range_sigma_threshold_);
        frame_ids_topic_names_ = this->declare_parameter("frame_ids", std::vector<std::string>{"Sensor_0", 
                                                                                               "Sensor_1", 
                                                                                               "Sensor_2", 
                                                                                               "Sensor_3", 
                                                                                               "Sensor_4", 
                                                                                               "Sensor_5", 
                                                                                               "Sensor_6", 
                                                                                               "Sensor_7"});
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_),
                                std::bind(&RangeSensorMultiZone::timer_callback, this));

        // Initialize combined point cloud publisher
        combined_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("range_pointcloud", 10);

        // Initialize laser scan publisher
        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("range_scan", 10);

        // Initialize individual point cloud publishers
        pointcloud_publishers_.resize(num_sensors_);
        for (int i = 0; i < num_sensors_; i++) {
            pointcloud_publishers_[i] = this->create_publisher<sensor_msgs::msg::PointCloud2>(frame_ids_topic_names_[i], 10);
        }

        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to odometry topic to get timestamps
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RangeSensorMultiZone::odom_callback, this, std::placeholders::_1));

        // Initialize ROS2 point cloud for each sensor
        sensor_pointclouds_.resize(num_sensors_);

        // Initialize sensor read tracking for diagnostics
        sensor_read_counts_.resize(num_sensors_, 0);
        last_sensor_read_counts_.resize(num_sensors_, 0);
        last_sensor_read_change_times_.resize(num_sensors_, this->now());
        last_diagnostic_check_time_ = this->now();

        // Initialize diagnostic updater
        diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
        diagnostic_updater_->setHardwareID("Range Sensor Multi Zone");
        diagnostic_updater_->add("Sensor Status", this, &RangeSensorMultiZone::diagnostic_callback);

        // Create timer to periodically update diagnostics (1 Hz) if not verbose
        if (!diag_verbose_) {
            diagnostic_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                [this]() {
                    if (diagnostic_updater_) {
                        diagnostic_updater_->force_update();
                    }
                });
        }

        init_sensor();

        RCLCPP_INFO(this->get_logger(), "RangeSensorMultiZone constructor done");
    }

    RangeSensorMultiZone::~RangeSensorMultiZone()
    {
        // Stop ranging for all sensors
        if (configuration_.platform.i2c_hdl >= 0) {
            for (int i = 0; i < num_sensors_; i++) {
                // Skip disabled sensors
                if (!is_sensor_enabled(i)) {
                    continue;
                }

                // Select the sensor
                if (VL53L5CX_Sensor_Select(&configuration_.platform, i, VL53L5CX_DEFAULT_I2C_ADDRESS) == 0) {
                    // Stop ranging for this sensor
                    if (vl53l5cx_stop_ranging(&configuration_) == 0) {
                        RCLCPP_INFO(this->get_logger(), "Stopped ranging for sensor %d", i);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to stop ranging for sensor %d", i);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to select sensor %d for cleanup", i);
                }
            }

            // Close I2C handle
            close(configuration_.platform.i2c_hdl);
            RCLCPP_INFO(this->get_logger(), "Closed I2C adapter %d", i2c_adapter_nr_);
        }
    }

    void RangeSensorMultiZone::init_sensor()
    {
        /* Open the I2C on Linux */
        char i2c_hdlname[20];
        snprintf(i2c_hdlname, 19, "/dev/i2c-%d", i2c_adapter_nr_);
        configuration_.platform.i2c_hdl = open(i2c_hdlname, O_RDWR);
        if (configuration_.platform.i2c_hdl < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open i2c adapter %d", i2c_adapter_nr_);
            rclcpp::shutdown();
            return;
        }
        for (int i = 0; i < num_sensors_; i++) {
            if (VL53L5CX_Sensor_Select(&configuration_.platform, i, VL53L5CX_DEFAULT_I2C_ADDRESS) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to select sensor %d", i);
                rclcpp::shutdown();
                return;
            }

	        // Check if there is a VL53L5CX sensor connected (retry up to 3 times)
            uint8_t isAlive = 0;
            bool sensor_alive = false;
            for (int retry = 0; retry < 3; retry++) {
                if (vl53l5cx_is_alive(&configuration_, &isAlive) != 0) {
                    RCLCPP_WARN(this->get_logger(), "Failed check if sensor %d is alive (attempt %d/3): %s",
                                i, retry + 1, strerror(errno));
                    if (retry < 2) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue;
                    }
                    RCLCPP_ERROR(this->get_logger(), "Failed check if sensor %d is alive after 3 attempts", i);
                    rclcpp::shutdown();
                    return;
                }
                if (!isAlive) {
                    RCLCPP_WARN(this->get_logger(), "Sensor %d is not alive (attempt %d/3)", i, retry + 1);
                    if (retry < 2) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue;
                    }
                    RCLCPP_ERROR(this->get_logger(), "Sensor %d is not alive after 3 attempts", i);
                    rclcpp::shutdown();
                    return;
                }
                sensor_alive = true;
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Sensor %d is alive .. loading firmware", i);
            uint8_t status = vl53l5cx_init(&configuration_);
            if (status != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load firmware for sensor %d, error: 0x%x, perror %s", i,  status, strerror(errno));
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Sensor %d firmware loaded", i);
            if (vl53l5cx_set_resolution(&configuration_,
                                   resolution_ == 4 ? VL53L5CX_RESOLUTION_4X4 : VL53L5CX_RESOLUTION_8X8) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set resolution for sensor %d", i);
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Sensor %d resolution set to %dx%d", i, resolution_, resolution_);
            if (vl53l5cx_set_ranging_frequency_hz(&configuration_, ranging_frequency_hz_) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set ranging frequency for sensor %d", i);
                rclcpp::shutdown();
                return;
            }
            if (vl53l5cx_set_target_order(&configuration_, VL53L5CX_TARGET_ORDER_CLOSEST) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set target order for sensor %d", i);
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Sensor %d target order set to %d", i, VL53L5CX_TARGET_ORDER_CLOSEST);
            if (vl53l5cx_set_sharpener_percent(&configuration_, sharpener_percent_) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set sharpener percent for sensor %d", i);
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Sensor %d sharpener percent set to %d%%", i, sharpener_percent_);
            RCLCPP_INFO(this->get_logger(), "Sensor %d sharpener percent set to %d", i, 0);
            RCLCPP_INFO(this->get_logger(), "Sensor %d ranging frequency set to %d Hz", i, ranging_frequency_hz_);
            if (vl53l5cx_start_ranging(&configuration_) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to start ranging for sensor %d", i);
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Sensor %d ranging started", i);
        }
        RCLCPP_INFO(this->get_logger(), "RangeSensorMultiZone init_sensor done");
    }


    void RangeSensorMultiZone::timer_callback()
    {
            // Loop over the sensors
            for (int i = 0; i < num_sensors_; i++) {
                // Select the Sensor
                if (VL53L5CX_Sensor_Select(&configuration_.platform, i, VL53L5CX_DEFAULT_I2C_ADDRESS) != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to select sensor %d", i);
                    rclcpp::shutdown();
                    return;
                }
                // Check if the sensor is ready
                uint8_t isReady = 0;
                if (vl53l5cx_check_data_ready(&configuration_, &isReady) != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to check if sensor %d is ready", i);
                    continue;
                }
                if (!isReady) continue;

                // Get the data
                VL53L5CX_ResultsData results;
                if (vl53l5cx_get_ranging_data(&configuration_, &results) != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get data for sensor %d", i);
                    continue;
                }

                // Increment sensor read count for diagnostics
                sensor_read_counts_[i]++;

                // Only convert to point cloud if sensor is enabled
                if (is_sensor_enabled(i)) {
                    convert_to_pointcloud(i, results);
                } else {
                    if (diag_verbose_) {
                        RCLCPP_DEBUG(this->get_logger(), "Sensor %d data read but skipped point cloud conversion (masked out)", i);
                    }
                }
            }

            // Combine all sensor point clouds and transform to base_link
            combine_and_transform_pointclouds();

            // Update diagnostics if verbose mode is enabled
            if (diag_verbose_ && diagnostic_updater_) {
                diagnostic_updater_->force_update();
            }
    }


    void RangeSensorMultiZone::convert_to_pointcloud(int sensor_id, const VL53L5CX_ResultsData& results)
    {
        // Create ROS2 point cloud for this sensor
        sensor_pointclouds_[sensor_id].data.clear();
        sensor_pointclouds_[sensor_id].header.stamp = this->get_clock()->now();
        sensor_pointclouds_[sensor_id].header.frame_id = frame_ids_topic_names_[sensor_id];
        sensor_pointclouds_[sensor_id].height = 1;
        sensor_pointclouds_[sensor_id].is_dense = true;

        // Set point cloud fields
        sensor_pointclouds_[sensor_id].fields.resize(4);
        sensor_pointclouds_[sensor_id].fields[0].name = "x";
        sensor_pointclouds_[sensor_id].fields[0].offset = 0;
        sensor_pointclouds_[sensor_id].fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        sensor_pointclouds_[sensor_id].fields[0].count = 1;
        
        sensor_pointclouds_[sensor_id].fields[1].name = "y";
        sensor_pointclouds_[sensor_id].fields[1].offset = 4;
        sensor_pointclouds_[sensor_id].fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        sensor_pointclouds_[sensor_id].fields[1].count = 1;
        
        sensor_pointclouds_[sensor_id].fields[2].name = "z";
        sensor_pointclouds_[sensor_id].fields[2].offset = 8;
        sensor_pointclouds_[sensor_id].fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        sensor_pointclouds_[sensor_id].fields[2].count = 1;
        
        sensor_pointclouds_[sensor_id].fields[3].name = "intensity";
        sensor_pointclouds_[sensor_id].fields[3].offset = 12;
        sensor_pointclouds_[sensor_id].fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        sensor_pointclouds_[sensor_id].fields[3].count = 1;
        
        sensor_pointclouds_[sensor_id].point_step = 16; // 4 fields * 4 bytes each
        sensor_pointclouds_[sensor_id].row_step = 0;

        // VL53L5CX field of view and zone layout
        // Use configurable field of view parameters
        const float horizontal_fov_rad = horizontal_fov_ * M_PI / 180.0f; // Convert degrees to radians
        const float vertical_fov_rad = vertical_fov_ * M_PI / 180.0f;   // Convert degrees to radians

        int zones = resolution_ * resolution_;
        float max_z = 0.0f;
        float min_z = 0.0f;
        int min_distance_mm = 10000;

        for (int zone_id = 0; zone_id < zones; zone_id++) {
            uint8_t nb_targets = results.nb_target_detected[zone_id];

            if (nb_targets == 0) continue;

            // Calculate zone position (row, col) from zone_id
            int row = zone_id / resolution_;
            int col = zone_id % resolution_;

            // Convert zone coordinates to angular offsets
            // Match Python implementation: per_px = deg2rad(fov) / res
            float per_px_h = horizontal_fov_rad / resolution_;
            float per_px_v = vertical_fov_rad / resolution_;
            
            // Python: w*per_px - deg2rad(fov)/2 - deg2rad(90)
            float zone_h_angle = col * per_px_h - horizontal_fov_rad / 2.0f - M_PI / 2.0f;
            // Python: h*per_px - deg2rad(fov)/2
            float zone_v_angle = row * per_px_v - vertical_fov_rad / 2.0f;

            // Process all targets in this zone
            for (int target = 0; target < nb_targets && target < (int)VL53L5CX_NB_TARGET_PER_ZONE; target++) {
                int target_idx = zone_id * VL53L5CX_NB_TARGET_PER_ZONE + target;

                uint16_t distance_mm = results.distance_mm[target_idx];
                uint8_t target_status = results.target_status[target_idx];
                uint16_t range_sigma_mm = results.range_sigma_mm[target_idx];
                // Skip invalid measurements
                if (distance_mm <= min_distance_ || 
                    distance_mm > max_distance_ ||
                    range_sigma_mm > range_sigma_threshold_ ||
                    target_status != 5) continue;
                
                if (distance_mm < min_distance_mm) min_distance_mm = distance_mm;
                
                // Convert distance from mm to meters
                float distance_m = distance_mm / 1000.0f;

                // Match Python implementation:
                // x = e*cos(w*per_px - deg2rad(fov)/2 - deg2rad(90))
                // y = e*sin(h*per_px - deg2rad(fov)/2)
                // z = e
                float sensor_x = distance_m * std::cos(zone_h_angle);
                float sensor_y = distance_m * std::sin(zone_v_angle);
                float sensor_z = distance_m;
                
                // Convert to ROS coordinate system: X forward, Y left, Z up
                float x = sensor_z;   // Forward
                float y = -sensor_x;   // Left
                float z = -sensor_y;  // Up (negative because sensor Y is down)
                
                if (z > max_z) max_z = z;
                if (z < min_z) min_z = z;
                
                // Apply height filtering - clip points outside min_height and max_height
                if (z < min_height_ || z > max_height_) {
                    continue; // Skip this point
                }

                // Add point to ROS2 point cloud
                sensor_pointclouds_[sensor_id].data.resize(sensor_pointclouds_[sensor_id].data.size() + 16);
                uint8_t* data_ptr = &sensor_pointclouds_[sensor_id].data[sensor_pointclouds_[sensor_id].data.size() - 16];
                
                memcpy(data_ptr, &x, 4);
                memcpy(data_ptr + 4, &y, 4);
                memcpy(data_ptr + 8, &z, 4);
                
                // Use range sigma as intensity
                float intensity = static_cast<float>(range_sigma_mm);
                memcpy(data_ptr + 12, &intensity, 4);
            }
        }

        // Update point cloud properties
        sensor_pointclouds_[sensor_id].width = sensor_pointclouds_[sensor_id].data.size() / 16; // 16 bytes per point

        // Publish individual point cloud only if there are subscribers
        if (pointcloud_publishers_[sensor_id]->get_subscription_count() > 0) {
            pointcloud_publishers_[sensor_id]->publish(sensor_pointclouds_[sensor_id]);
        }

        if (diag_verbose_) {
            RCLCPP_INFO(this->get_logger(), "Sensor %d: Created point cloud with %zu points (max_z: %f, min_z: %f, min_distance_mm: %d)",
                       sensor_id, (size_t)sensor_pointclouds_[sensor_id].width, max_z, min_z, min_distance_mm);
        }
    }

    void RangeSensorMultiZone::publish_data()
    {
        RCLCPP_INFO(this->get_logger(), "RangeSensorMultiZone publish_data");
    }

    void RangeSensorMultiZone::combine_and_transform_pointclouds()
    {
        // Create combined point cloud
        sensor_msgs::msg::PointCloud2 combined_cloud;

        // Use odometry timestamp if available, otherwise use current time
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (latest_odom_stamp_.sec != 0 || latest_odom_stamp_.nanosec != 0) {
                combined_cloud.header.stamp = latest_odom_stamp_;
            } else {
                combined_cloud.header.stamp = this->get_clock()->now();
            }
        }

        combined_cloud.header.frame_id = "base_link";
        combined_cloud.height = 1;
        combined_cloud.is_dense = true;

        // Set point cloud fields
        combined_cloud.fields.resize(4);
        combined_cloud.fields[0].name = "x";
        combined_cloud.fields[0].offset = 0;
        combined_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        combined_cloud.fields[0].count = 1;
        
        combined_cloud.fields[1].name = "y";
        combined_cloud.fields[1].offset = 4;
        combined_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        combined_cloud.fields[1].count = 1;
        
        combined_cloud.fields[2].name = "z";
        combined_cloud.fields[2].offset = 8;
        combined_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        combined_cloud.fields[2].count = 1;
        
        combined_cloud.fields[3].name = "intensity";
        combined_cloud.fields[3].offset = 12;
        combined_cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        combined_cloud.fields[3].count = 1;
        
        combined_cloud.point_step = 16; // 4 fields * 4 bytes each
        combined_cloud.row_step = 0;

        // Combine all sensor point clouds
        for (int i = 0; i < num_sensors_; i++) {
            // Skip sensor if masked out or no data
            if (!is_sensor_enabled(i) || sensor_pointclouds_[i].data.empty()) continue;

            try {
                // Transform the point cloud from sensor frame to base_link
                sensor_msgs::msg::PointCloud2 transformed_cloud;
                tf2::doTransform(sensor_pointclouds_[i], transformed_cloud, 
                               tf_buffer_->lookupTransform("base_link", frame_ids_topic_names_[i], tf2::TimePointZero));

                // Concatenate point clouds
                combined_cloud.data.insert(combined_cloud.data.end(), 
                                         transformed_cloud.data.begin(), 
                                         transformed_cloud.data.end());

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform point cloud from %s to base_link: %s",
                           frame_ids_topic_names_[i].c_str(), ex.what());
                continue;
            }
        }

        // Update combined point cloud properties
        combined_cloud.width = combined_cloud.data.size() / 16; // 16 bytes per point

        // Convert to PCL once for all filtering operations
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (combined_cloud.width > 0 && (radius_outlier_enabled_ || temporal_filter_enabled_)) {
            pcl::fromROSMsg(combined_cloud, *pcl_cloud);

            // Apply temporal moving average filter first (if enabled)
            if (temporal_filter_enabled_) {
                pcl_cloud = apply_temporal_filter(pcl_cloud);
            }

            // Apply radius outlier filter after temporal (if enabled)
            if (radius_outlier_enabled_) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_outlier_removal;
                
                radius_outlier_removal.setInputCloud(pcl_cloud);
                radius_outlier_removal.setRadiusSearch(radius_outlier_radius_);
                radius_outlier_removal.setMinNeighborsInRadius(radius_outlier_min_neighbors_);
                radius_outlier_removal.filter(*filtered_cloud);
                
                pcl_cloud = filtered_cloud;
            }

            // Convert back to ROS2 message once
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl::toROSMsg(*pcl_cloud, output_cloud);
            output_cloud.header = combined_cloud.header;

            // Publish filtered combined point cloud
            combined_pointcloud_publisher_->publish(output_cloud);

            // Convert to laser scan and publish only if there are subscribers
            if (laserscan_publisher_->get_subscription_count() > 0) {
                auto laser_scan = pointcloud_to_laserscan(pcl_cloud, output_cloud.header);
                laserscan_publisher_->publish(laser_scan);
            }

            if (diag_verbose_) {
                RCLCPP_DEBUG(this->get_logger(), "Published combined point cloud with %zu points and laser scan",
                           pcl_cloud->points.size());
            }
        } else {
            // No filtering, publish as-is
            combined_pointcloud_publisher_->publish(combined_cloud);

            // Convert to PCL for laser scan generation only if there are subscribers
            if (laserscan_publisher_->get_subscription_count() > 0 && combined_cloud.width > 0) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(combined_cloud, *temp_cloud);
                auto laser_scan = pointcloud_to_laserscan(temp_cloud, combined_cloud.header);
                laserscan_publisher_->publish(laser_scan);
            }

            if (diag_verbose_) {
                RCLCPP_DEBUG(this->get_logger(), "Published combined point cloud with %zu points and laser scan",
                           (size_t)combined_cloud.width);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr RangeSensorMultiZone::apply_temporal_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
    {
        // Add current cloud to buffer
        pointcloud_buffer_.push_back(input_cloud);

        // Keep only the last N frames
        if (pointcloud_buffer_.size() > static_cast<size_t>(temporal_filter_size_)) {
            pointcloud_buffer_.erase(pointcloud_buffer_.begin());
        }

        // If buffer is not full yet, just return the current cloud
        if (pointcloud_buffer_.size() < static_cast<size_t>(temporal_filter_size_)) {
            return input_cloud;
        }

        // Create exponentially averaged point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr averaged_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // Use exponential moving average with alpha parameter
        // For each voxel, apply: new_value = alpha * current_value + (1-alpha) * previous_value
        std::map<std::tuple<int, int, int>, pcl::PointXYZI> voxel_map;

        // Voxelize points from all frames (10mm voxels for grouping)
        float voxel_size = 0.01f; // 10mm
        
        // Process frames in chronological order (oldest first)
        for (size_t frame_idx = 0; frame_idx < pointcloud_buffer_.size(); frame_idx++) {
            const auto& cloud = pointcloud_buffer_[frame_idx];
            float frame_weight = (frame_idx == pointcloud_buffer_.size() - 1) ? temporal_filter_alpha_ : (1.0f - temporal_filter_alpha_);
            
            for (const auto& point : cloud->points) {
                int vx = static_cast<int>(point.x / voxel_size);
                int vy = static_cast<int>(point.y / voxel_size);
                int vz = static_cast<int>(point.z / voxel_size);
                auto voxel_key = std::make_tuple(vx, vy, vz);
                
                if (voxel_map.find(voxel_key) == voxel_map.end()) {
                    // First occurrence of this voxel
                    voxel_map[voxel_key] = point;
                } else {
                    // Apply exponential moving average
                    auto& existing_point = voxel_map[voxel_key];
                    if (frame_idx == pointcloud_buffer_.size() - 1) {
                        // Current frame: exponential moving average
                        existing_point.x = temporal_filter_alpha_ * point.x + (1.0f - temporal_filter_alpha_) * existing_point.x;
                        existing_point.y = temporal_filter_alpha_ * point.y + (1.0f - temporal_filter_alpha_) * existing_point.y;
                        existing_point.z = temporal_filter_alpha_ * point.z + (1.0f - temporal_filter_alpha_) * existing_point.z;
                        existing_point.intensity = temporal_filter_alpha_ * point.intensity + (1.0f - temporal_filter_alpha_) * existing_point.intensity;
                    } else {
                        // Previous frames: weighted average
                        existing_point.x = frame_weight * point.x + (1.0f - frame_weight) * existing_point.x;
                        existing_point.y = frame_weight * point.y + (1.0f - frame_weight) * existing_point.y;
                        existing_point.z = frame_weight * point.z + (1.0f - frame_weight) * existing_point.z;
                        existing_point.intensity = frame_weight * point.intensity + (1.0f - frame_weight) * existing_point.intensity;
                    }
                }
            }
        }

        // Convert map to point cloud
        for (const auto& voxel : voxel_map) {
            averaged_cloud->points.push_back(voxel.second);
        }

        averaged_cloud->width = averaged_cloud->points.size();
        averaged_cloud->height = 1;
        averaged_cloud->is_dense = true;

        if (diag_verbose_) {
            RCLCPP_DEBUG(this->get_logger(), "Temporal filter (alpha=%.2f): %zu frames, %zu -> %zu points",
                       temporal_filter_alpha_, pointcloud_buffer_.size(), input_cloud->points.size(), averaged_cloud->points.size());
        }

        return averaged_cloud;
    }

    sensor_msgs::msg::LaserScan RangeSensorMultiZone::pointcloud_to_laserscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std_msgs::msg::Header& header)
    {
        sensor_msgs::msg::LaserScan scan;
        scan.header = header;

        // LaserScan parameters - full 360 degree coverage
        scan.angle_min = -M_PI;  // -180 degrees
        scan.angle_max = M_PI;   // +180 degrees
        scan.angle_increment = 0.5 * M_PI / 180.0;  // 0.5 degree resolution
        scan.time_increment = 0.0;
        scan.scan_time = 0.0;
        scan.range_min = min_distance_ / 1000.0;  // Convert mm to meters
        scan.range_max = max_distance_ / 1000.0;  // Convert mm to meters

        // Calculate number of beams
        int num_beams = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
        scan.ranges.resize(num_beams, std::numeric_limits<float>::infinity());
        scan.intensities.resize(num_beams, 0.0);

        // For 3D scan support, we'll project all points within the height range onto a 2D plane
        // and keep the closest point for each angle
        for (const auto& point : cloud->points) {
            // Check if point is within valid height range (this allows vertical 3D coverage)
            if (point.z < min_height_ || point.z > max_height_) {
                continue;
            }

            // Calculate angle from x,y coordinates (horizontal angle)
            float angle = atan2(point.y, point.x);

            // Calculate horizontal range (2D distance in x-y plane)
            float range = sqrt(point.x * point.x + point.y * point.y);

            // Check if range is valid
            if (range < scan.range_min || range > scan.range_max) {
                continue;
            }

            // Calculate beam index
            int beam_index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);

            // Check if beam index is valid
            if (beam_index < 0 || beam_index >= num_beams) {
                continue;
            }

            // Keep closest point for each beam (this projects 3D data to 2D scan)
            if (range < scan.ranges[beam_index]) {
                scan.ranges[beam_index] = range;
                scan.intensities[beam_index] = point.intensity;
            }
        }

        return scan;
    }

    bool RangeSensorMultiZone::is_sensor_enabled(int sensor_id) const
    {
        // Check if sensor_id is within valid range
        if (sensor_id < 0 || sensor_id >= 8) {
            return false;
        }
        
        // Check if the corresponding bit is set in the sensor mask
        // Bit 0 = sensor 0, bit 1 = sensor 1, etc.
        // If bit is set (1), sensor is DISABLED
        // If bit is clear (0), sensor is ENABLED
        return !(sensor_mask_ & (1 << sensor_id));
    }

    void RangeSensorMultiZone::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_stamp_ = msg->header.stamp;
    }

    void RangeSensorMultiZone::diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        // Get current time
        rclcpp::Time current_time = this->now();
        double time_elapsed = (current_time - last_diagnostic_check_time_).seconds();

        bool sensor_warn = false;
        bool sensor_error = false;
        std::string message;
        std::vector<std::string> stalled_sensors_warn;
        std::vector<std::string> stalled_sensors_error;

        // Check for sensor stalls if more than 5 seconds have passed
        if (time_elapsed >= 5.0) {
            // Update sensor read change times if counts have changed
            for (int i = 0; i < num_sensors_; i++) {
                if (sensor_read_counts_[i] != last_sensor_read_counts_[i]) {
                    last_sensor_read_change_times_[i] = current_time;
                    last_sensor_read_counts_[i] = sensor_read_counts_[i];
                }
            }

            // Check each sensor for stalls (only enabled sensors)
            for (int i = 0; i < num_sensors_; i++) {
                if (!is_sensor_enabled(i)) {
                    continue; // Skip disabled sensors
                }

                double sensor_stall_time = (current_time - last_sensor_read_change_times_[i]).seconds();

                if (sensor_stall_time >= 30.0) {
                    sensor_error = true;
                    stalled_sensors_error.push_back("Sensor " + std::to_string(i) + " (" +
                                                   std::to_string(static_cast<int>(sensor_stall_time)) + "s)");
                } else if (sensor_stall_time >= 5.0) {
                    sensor_warn = true;
                    stalled_sensors_warn.push_back("Sensor " + std::to_string(i) + " (" +
                                                   std::to_string(static_cast<int>(sensor_stall_time)) + "s)");
                }
            }

            last_diagnostic_check_time_ = current_time;
        }

        // Build diagnostic message
        if (sensor_error) {
            message = "ERROR: Sensors stalled > 30s: ";
            for (size_t i = 0; i < stalled_sensors_error.size(); i++) {
                message += stalled_sensors_error[i];
                if (i < stalled_sensors_error.size() - 1) message += ", ";
            }
            if (!stalled_sensors_warn.empty()) {
                message += ". WARN: Sensors stalled > 5s: ";
                for (size_t i = 0; i < stalled_sensors_warn.size(); i++) {
                    message += stalled_sensors_warn[i];
                    if (i < stalled_sensors_warn.size() - 1) message += ", ";
                }
            }
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
        } else if (sensor_warn) {
            message = "WARNING: Sensors stalled > 5s: ";
            for (size_t i = 0; i < stalled_sensors_warn.size(); i++) {
                message += stalled_sensors_warn[i];
                if (i < stalled_sensors_warn.size() - 1) message += ", ";
            }
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All sensors operational");
        }

        // Add sensor read counts
        for (int i = 0; i < num_sensors_; i++) {
            std::string sensor_key = "Sensor " + std::to_string(i) + " reads";
            stat.add(sensor_key, static_cast<int>(sensor_read_counts_[i]));
        }

        // Add sensor configuration info
        stat.add("Number of Sensors", num_sensors_);
        stat.add("Sensor Mask", static_cast<int>(sensor_mask_));
        stat.add("Resolution", std::to_string(resolution_) + "x" + std::to_string(resolution_));
        stat.add("Ranging Frequency", std::to_string(ranging_frequency_hz_) + " Hz");
        stat.add("Min Distance", std::to_string(min_distance_) + " mm");
        stat.add("Max Distance", std::to_string(max_distance_) + " mm");
        stat.add("Min Height", std::to_string(min_height_) + " m");
        stat.add("Max Height", std::to_string(max_height_) + " m");
        stat.add("Radius Outlier Filter", radius_outlier_enabled_ ? "Enabled" : "Disabled");
        stat.add("Temporal Filter", temporal_filter_enabled_ ? "Enabled" : "Disabled");
    }

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<range_sensor_multi_zone::RangeSensorMultiZone>());
    rclcpp::shutdown();
    return 0;
}
