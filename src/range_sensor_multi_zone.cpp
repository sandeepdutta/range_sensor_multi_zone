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

        // Initialize individual point cloud publishers
        pointcloud_publishers_.resize(num_sensors_);
        for (int i = 0; i < num_sensors_; i++) {
            pointcloud_publishers_[i] = this->create_publisher<sensor_msgs::msg::PointCloud2>(frame_ids_topic_names_[i], 10);
        }

        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize ROS2 point cloud for each sensor
        sensor_pointclouds_.resize(num_sensors_);

        init_sensor();

        RCLCPP_INFO(this->get_logger(), "RangeSensorMultiZone constructor done");
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

	        // Check if there is a VL53L5CX sensor connected
            uint8_t isAlive = 0;
	        if (vl53l5cx_is_alive(&configuration_, &isAlive) != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed check if sensor %d is alive %s", i, strerror(errno));
                    rclcpp::shutdown();
                    return;
            }
            if (!isAlive) {
                RCLCPP_ERROR(this->get_logger(), "Sensor %d is not alive", i);
                rclcpp::shutdown();
                return;
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
                float y = sensor_x;   // Left
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
                
                // Use signal strength as intensity if available
                #ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
                float intensity = static_cast<float>(results.signal_per_spad[target_idx]);
                #else
                float intensity = 255.0f; // Default intensity
                #endif
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
        combined_cloud.header.stamp = this->get_clock()->now();
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

            if (diag_verbose_) {
                RCLCPP_DEBUG(this->get_logger(), "Published combined point cloud with %zu points",
                           pcl_cloud->points.size());
            }
        } else {
            // No filtering, publish as-is
            combined_pointcloud_publisher_->publish(combined_cloud);

            if (diag_verbose_) {
                RCLCPP_DEBUG(this->get_logger(), "Published combined point cloud with %zu points",
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

        // Create averaged point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr averaged_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // For simplicity, we'll average all points from all frames
        // A more sophisticated approach would use nearest neighbor matching
        std::map<std::tuple<int, int, int>, std::vector<pcl::PointXYZI>> voxel_map;

        // Voxelize points from all frames (10mm voxels for grouping)
        float voxel_size = 0.01f; // 10mm
        for (const auto& cloud : pointcloud_buffer_) {
            for (const auto& point : cloud->points) {
                int vx = static_cast<int>(point.x / voxel_size);
                int vy = static_cast<int>(point.y / voxel_size);
                int vz = static_cast<int>(point.z / voxel_size);
                voxel_map[std::make_tuple(vx, vy, vz)].push_back(point);
            }
        }

        // Average points in each voxel
        for (const auto& voxel : voxel_map) {
            const auto& points = voxel.second;
            pcl::PointXYZI avg_point;
            avg_point.x = 0;
            avg_point.y = 0;
            avg_point.z = 0;
            avg_point.intensity = 0;

            for (const auto& p : points) {
                avg_point.x += p.x;
                avg_point.y += p.y;
                avg_point.z += p.z;
                avg_point.intensity += p.intensity;
            }

            float count = static_cast<float>(points.size());
            avg_point.x /= count;
            avg_point.y /= count;
            avg_point.z /= count;
            avg_point.intensity /= count;

            averaged_cloud->points.push_back(avg_point);
        }

        averaged_cloud->width = averaged_cloud->points.size();
        averaged_cloud->height = 1;
        averaged_cloud->is_dense = true;

        if (diag_verbose_) {
            RCLCPP_DEBUG(this->get_logger(), "Temporal filter: %zu frames, %zu -> %zu points",
                       pointcloud_buffer_.size(), input_cloud->points.size(), averaged_cloud->points.size());
        }

        return averaged_cloud;
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


}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<range_sensor_multi_zone::RangeSensorMultiZone>());
    rclcpp::shutdown();
    return 0;
}
