#ifndef RANGE_SENSOR_MULTI_ZONE__RANGE_SENSOR_MULTI_ZONE_HPP_
#define RANGE_SENSOR_MULTI_ZONE__RANGE_SENSOR_MULTI_ZONE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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
            std::vector<std::string> frame_ids_;
            std::vector<std::string> topic_names_;
            std::vector<std::string> frame_ids_topic_names_;
            VL53L5CX_Configuration configuration_;
            rclcpp::TimerBase::SharedPtr timer_;
            std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_publishers_;

            void init_sensor();
            void timer_callback();
            void publish_data();
            void convert_to_pointcloud(int sensor_id, const VL53L5CX_ResultsData& results);
    };
}
#endif
