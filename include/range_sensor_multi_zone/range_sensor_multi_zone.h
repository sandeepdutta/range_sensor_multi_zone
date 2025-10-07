#ifndef RANGE_SENSOR_MULTI_ZONE__RANGE_SENSOR_MULTI_ZONE_HPP_
#define RANGE_SENSOR_MULTI_ZONE__RANGE_SENSOR_MULTI_ZONE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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

namespace range_sensor_multi_zone
{
    class RangeSensorMultiZone : public rclcpp::Node
    {
        public:
            explicit RangeSensorMultiZone();
            ~RangeSensorMultiZone() = default;

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
            double radius_outlier_radius_;
            int radius_outlier_min_neighbors_;
            uint8_t sensor_mask_;
            double horizontal_fov_;
            double vertical_fov_;
            std::vector<std::string> frame_ids_;
            std::vector<std::string> topic_names_;
            std::vector<std::string> frame_ids_topic_names_;
            VL53L5CX_Configuration configuration_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pointcloud_publisher_;
            std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_publishers_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::vector<sensor_msgs::msg::PointCloud2> sensor_pointclouds_;

            void init_sensor();
            void timer_callback();
            void publish_data();
            void convert_to_pointcloud(int sensor_id, const VL53L5CX_ResultsData& results);
            void combine_and_transform_pointclouds();
            bool is_sensor_enabled(int sensor_id) const;
    };
}
#endif
