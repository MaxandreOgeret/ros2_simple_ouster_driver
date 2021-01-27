#include "driver.h"

namespace ros2_simple_ouster_driver {
    Driver::Driver(const rclcpp::NodeOptions & options) : Node("driver", options) {

      this->declare_parameter("ip_sensor", "192.168.3.200");
      this->declare_parameter("ip_dest", "192.168.3.100");
      this->declare_parameter("lidar_port", "7502");
      this->declare_parameter("imu_port", "7503");
      this->declare_parameter("lidar_mode", "512x10");
      this->declare_parameter("timestamp_mode", "TIME_FROM_INTERNAL_OSC");

      wrapper = new Wrapper(this->get_parameter("ip_sensor").as_string(),
                            this->get_parameter("ip_dest").as_string(),
                            std::stoi(this->get_parameter("lidar_port").as_string()),
                            std::stoi(this->get_parameter("imu_port").as_string()),
                            this->get_parameter("lidar_mode").as_string(),
                            this->get_parameter("timestamp_mode").as_string()
                            );

      wrapper->configure();
      wrapper->get();
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_simple_ouster_driver::Driver)