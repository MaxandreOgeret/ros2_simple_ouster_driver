#include "rclcpp/rclcpp.hpp"
#include "driver.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<ros2_simple_ouster_driver::Driver>(options);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}