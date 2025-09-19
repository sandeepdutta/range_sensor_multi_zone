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
#include <range_sensor_multi_zone/range_sensor_multi_zone.h>
#include <range_sensor_multi_zone/vl53l5cx_api.h>

namespace range_sensor_multi_zone
{
    RangeSensorMultiZone::RangeSensorMultiZone()
    : Node("range_sensor_multi_zone")
    {
        i2c_adapter_nr_ = this->declare_parameter("i2c_adapter_nr", 7);
        num_sensors_    = this->declare_parameter("num_sensors", 2);
        diag_verbose_ = this->declare_parameter("diag_verbose", false);
        resolution_ = this->declare_parameter("resolution", 4);
        ranging_frequency_hz_ = this->declare_parameter("ranging_frequency_hz", 10);
        timer_period_ = 1000 / ranging_frequency_hz_; // in milliseconds
        // add 10% margin
        timer_period_ = (int) ((float)timer_period_ * 0.9);
        RCLCPP_INFO(this->get_logger(), "Timer period: %d ms", timer_period_);
        frame_ids_topic_names_ = this->declare_parameter("frame_ids", std::vector<std::string>{"range_sensor_front_left",
                                                                                               "range_sensor_front_right",
                                                                                               "range_sensor_back_left",
                                                                                               "range_sensor_back_right"});
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_),
                                std::bind(&RangeSensorMultiZone::timer_callback, this));

        // Initialize point cloud publishers for each sensor
        for (int i = 0; i < num_sensors_; i++) {
            std::string topic_name = "sensor_" + std::to_string(i) + "/pointcloud";
            auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
            pointcloud_publishers_.push_back(publisher);
        }

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
                    rclcpp::shutdown();
                    return;
                }
                if (!isReady) continue;

                // Get the data
                VL53L5CX_ResultsData results;
                if (vl53l5cx_get_ranging_data(&configuration_, &results) != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get data for sensor %d", i);
                    rclcpp::shutdown();
                    return;
                }

                // Convert to point cloud and publish
                convert_to_pointcloud(i, results);
            }
    }


    void RangeSensorMultiZone::convert_to_pointcloud(int sensor_id, const VL53L5CX_ResultsData& results)
    {
        // Set frame id based on sensor
        std::string frame_id = "sensor_" + std::to_string(sensor_id) + "_frame";
        if (sensor_id < (int)frame_ids_topic_names_.size()) {
            frame_id = frame_ids_topic_names_[sensor_id];
        }

        // VL53L5CX field of view and zone layout
        // The sensor has a 63° x 45° field of view
        // For 4x4: zones are numbered 0-15, for 8x8: zones are numbered 0-63
        const float horizontal_fov = 63.0f * M_PI / 180.0f; // radians
        const float vertical_fov = 45.0f * M_PI / 180.0f;   // radians

        int zones = resolution_ * resolution_;

        // Count valid points first
        size_t point_count = 0;
        for (int zone_id = 0; zone_id < zones; zone_id++) {
            uint8_t nb_targets = results.nb_target_detected[zone_id];
            for (int target = 0; target < nb_targets && target < (int)VL53L5CX_NB_TARGET_PER_ZONE; target++) {
                int target_idx = zone_id * VL53L5CX_NB_TARGET_PER_ZONE + target;
                uint16_t distance_mm = results.distance_mm[target_idx];
                uint8_t target_status = results.target_status[target_idx];
                if (distance_mm > 0 && target_status == 5) { // Status 5 = valid measurement
                    point_count++;
                }
            }
        }

        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.height = 1;
        cloud_msg.width = point_count;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        // Define point cloud fields (x, y, z, intensity)
        cloud_msg.fields.resize(4);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;

        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;

        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;

        cloud_msg.fields[3].name = "intensity";
        cloud_msg.fields[3].offset = 12;
        cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[3].count = 1;

        cloud_msg.point_step = 16; // 4 fields * 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.data.resize(cloud_msg.row_step);

        // Use iterators to fill the point cloud data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

        for (int zone_id = 0; zone_id < zones; zone_id++) {
            uint8_t nb_targets = results.nb_target_detected[zone_id];

            if (nb_targets == 0) continue;

            // Calculate zone position (row, col) from zone_id
            int row = zone_id / resolution_;
            int col = zone_id % resolution_;

            // Convert zone coordinates to angular offsets
            // Center the coordinates: (0,0) is center of sensor
            float zone_h_angle = ((col - (resolution_ - 1) / 2.0f) / (resolution_ - 1)) * horizontal_fov;
            float zone_v_angle = ((row - (resolution_ - 1) / 2.0f) / (resolution_ - 1)) * vertical_fov;

            // Process all targets in this zone
            for (int target = 0; target < nb_targets && target < (int)VL53L5CX_NB_TARGET_PER_ZONE; target++) {
                int target_idx = zone_id * VL53L5CX_NB_TARGET_PER_ZONE + target;

                uint16_t distance_mm = results.distance_mm[target_idx];
                uint8_t target_status = results.target_status[target_idx];

                // Skip invalid measurements
                if (distance_mm == 0 || target_status != 5) continue; // Status 5 = valid measurement

                // Convert distance from mm to meters
                float distance_m = distance_mm / 1000.0f;

                // Convert spherical coordinates to Cartesian
                // Using standard robotics convention: X forward, Y left, Z up
                *iter_x = distance_m * std::cos(zone_v_angle) * std::cos(zone_h_angle);  // Forward
                *iter_y = distance_m * std::cos(zone_v_angle) * std::sin(zone_h_angle);  // Left
                *iter_z = distance_m * std::sin(zone_v_angle);                           // Up

                // Use signal strength as intensity if available
                #ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
                *iter_intensity = static_cast<float>(results.signal_per_spad[target_idx]);
                #else
                *iter_intensity = 255.0f; // Default intensity
                #endif

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_intensity;
            }
        }

        pointcloud_publishers_[sensor_id]->publish(cloud_msg);

        if (diag_verbose_) {
            RCLCPP_INFO(this->get_logger(), "Sensor %d: Published point cloud with %zu points",
                       sensor_id, point_count);
        }
    }

    void RangeSensorMultiZone::publish_data()
    {
        RCLCPP_INFO(this->get_logger(), "RangeSensorMultiZone publish_data");
    }


}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<range_sensor_multi_zone::RangeSensorMultiZone>());
    rclcpp::shutdown();
    return 0;
}
