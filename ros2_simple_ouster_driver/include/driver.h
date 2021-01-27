#ifndef ROS2_SIMPLE_OUSTER_DRIVER_DRIVER_H
#define ROS2_SIMPLE_OUSTER_DRIVER_DRIVER_H

#include "rclcpp/rclcpp.hpp"
#include "wrapper.h"
#include "ros2_simple_ouster_driver_msgs/msg/packet.hpp"

namespace ros2_simple_ouster_driver {
    class Driver : public rclcpp::Node {
    public:
        Wrapper* wrapper;
        ros2_simple_ouster_driver_msgs::msg::Packet lidar_packet, imu_packet;

        explicit Driver(const rclcpp::NodeOptions & options);
    };
}

#endif //ROS2_SIMPLE_OUSTER_DRIVER_DRIVER_H
