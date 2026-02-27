#ifndef RANGE_SENSOR_MULTI_ZONE__RANGE_SENSOR_MULTI_ZONE_HPP_
#define RANGE_SENSOR_MULTI_ZONE__RANGE_SENSOR_MULTI_ZONE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <range_sensor_multi_zone/vl53l5cx_api.h>
#include <range_sensor_multi_zone/platform.h>
#include <range_sensor_multi_zone/vl53l5cx_plugin_detection_thresholds.h>
#include <range_sensor_multi_zone/vl53l5cx_plugin_motion_indicator.h>
#include <range_sensor_multi_zone/vl53l5cx_plugin_xtalk.h>
#include <mutex>

namespace range_sensor_multi_zone
{
    class RangeSensorMultiZone : public rclcpp::Node
    {
        public:
            explicit RangeSensorMultiZone();
            ~RangeSensorMultiZone();

        private:
            int i2c_adapter_nr_;
            int num_sensors_;
            int timer_period_;
            int resolution_;
            int ranging_frequency_hz_;
            bool diag_verbose_;
            double max_height_;
            double min_height_;
            int min_distance_;
            int max_distance_;
            bool radius_outlier_enabled_;
            double radius_outlier_radius_;
            int radius_outlier_min_neighbors_;
            bool temporal_filter_enabled_;
            int temporal_filter_size_;
            double temporal_filter_alpha_;
            uint8_t sensor_mask_;
            double horizontal_fov_;
            double vertical_fov_;
            int sharpener_percent_;
            int range_sigma_threshold_;
            uint8_t target_status_filter_;
            std::vector<std::string> frame_ids_;
            std::vector<std::string> topic_names_;
            std::vector<std::string> frame_ids_topic_names_;
            VL53L5CX_Configuration configuration_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pointcloud_publisher_;
            rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;
            std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_publishers_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::vector<sensor_msgs::msg::PointCloud2> sensor_pointclouds_;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointcloud_buffer_;
            std::mutex odom_mutex_;
            builtin_interfaces::msg::Time latest_odom_stamp_;

            // Diagnostic updater
            std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
            rclcpp::TimerBase::SharedPtr diagnostic_timer_;

            // Sensor read tracking for diagnostics
            std::vector<uint64_t> sensor_read_counts_;
            std::vector<uint64_t> last_sensor_read_counts_;
            // Per sensor x,y,z min Max
            std::vector<float> sensor_min_x_, sensor_max_x_, sensor_min_y_,
                               sensor_max_y_, sensor_min_z_, sensor_max_z_;
            // Per sensor range sigma min/max
            std::vector<uint16_t> sensor_min_sigma_mm_, sensor_max_sigma_mm_;
            std::vector<rclcpp::Time> last_sensor_read_change_times_;
            rclcpp::Time last_diagnostic_check_time_;

            // Timing tracking for diagnostics
            std::vector<int64_t> sensor_read_times_ms_;     // Per-sensor I2C read time
            std::vector<int64_t> sensor_convert_times_ms_;  // Per-sensor conversion time
            std::vector<int64_t> sensor_tf_times_ms_;       // Per-sensor TF transform time
            int64_t combine_transform_time_ms_;             // Total combine and transform time
            int64_t timer_callback_time_ms_;                // Total timer callback time
            int64_t temporal_filter_time_ms_;               // Temporal filter time
            int64_t radius_filter_time_ms_;                 // Radius outlier filter time

            /**
             * @brief Initialize VL53L5CX sensors via I2C
             *
             * Opens I2C adapter, detects sensors, loads firmware, and starts ranging.
             * Configures each sensor with resolution, frequency, and processing parameters.
             */
            void init_sensor();

            /**
             * @brief Main timer callback for sensor reading and processing
             *
             * Called at fixed intervals (configured by ranging_frequency_hz).
             * Reads data from all enabled sensors, converts to point clouds,
             * applies filtering, transforms to base_link, and publishes results.
             * Measures timing for performance diagnostics.
             */
            void timer_callback();

            /**
             * @brief Placeholder for future data publishing functionality
             */
            void publish_data();

            /**
             * @brief Convert raw VL53L5CX sensor data to ROS2 point cloud
             *
             * Processes raw range, sigma, and status data from sensor.
             * Applies distance and sigma filtering, transforms to ROS coordinates.
             * Applies per-sensor filtering (radius outlier, temporal).
             * Transforms point cloud to base_link frame and publishes immediately.
             *
             * @param sensor_id Sensor index (0 to num_sensors-1)
             * @param results Raw VL53L5CX ranging results
             * @param timestamp Header timestamp for point cloud (from odometry or clock)
             */
            void convert_to_pointcloud(int sensor_id, const VL53L5CX_ResultsData& results, const rclcpp::Time& timestamp);

            /**
             * @brief Combine individual sensor point clouds and generate laser scan
             *
             * Concatenates already-transformed and filtered individual point clouds.
             * Generates 2D laser scan projection from combined 3D point cloud.
             * Publishes combined point cloud and laser scan to topics.
             * Uses latest odometry timestamp if available.
             *
             * @param timestamp Callback timestamp to use as fallback if no odometry available
             */
            void combine_and_transform_pointclouds(const rclcpp::Time& timestamp);

            /**
             * @brief Apply temporal exponential moving average filter
             *
             * Maintains buffer of N recent frames and applies weighted averaging
             * across frames using alpha parameter. Uses voxel-based grouping.
             *
             * @param input_cloud Current point cloud to add to buffer
             * @return Filtered point cloud with temporal smoothing applied
             */
            pcl::PointCloud<pcl::PointXYZI>::Ptr apply_temporal_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud);

            /**
             * @brief Convert 3D point cloud to 2D laser scan message
             *
             * Projects 3D points onto 2D horizontal plane, keeping closest point per angle.
             * Respects height filtering (min_height_, max_height_).
             * Generates 360-degree scan with configurable angular resolution.
             *
             * @param cloud Input point cloud in base_link frame
             * @param header Header information for scan message (frame_id, timestamp)
             * @return LaserScan message ready for publishing
             */
            sensor_msgs::msg::LaserScan pointcloud_to_laserscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std_msgs::msg::Header& header);

            /**
             * @brief Check if sensor is enabled based on sensor_mask
             *
             * Bit 0 = sensor 0, bit 1 = sensor 1, etc.
             * If bit is set (1), sensor is DISABLED. If clear (0), sensor is ENABLED.
             *
             * @param sensor_id Sensor index to check
             * @return true if sensor is enabled, false if masked out
             */
            bool is_sensor_enabled(int sensor_id) const;

            /**
             * @brief Callback for odometry subscription to synchronize with robot state
             *
             * Stores latest odometry timestamp for use in point cloud headers.
             * Allows point clouds to be timestamped with state estimate rather than sensor clock.
             *
             * @param msg Odometry message containing timestamp and pose
             */
            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

            /**
             * @brief Publish sensor diagnostics and performance metrics
             *
             * Called by diagnostic_updater_ at configured rate (1 Hz if not verbose).
             * Reports: sensor read counts, min/max spatial ranges, sigma ranges,
             * target status filter, timing statistics (I2C read, conversion, transform).
             * Detects sensor stalls (no data for >5s warns, >30s errors).
             *
             * @param stat Diagnostic status wrapper to add key-value pairs
             */
            void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);
    };
}
#endif
