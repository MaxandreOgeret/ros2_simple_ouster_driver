#ifndef ROS2_SIMPLE_OUSTER_DRIVER_EXCEPTION_H
#define ROS2_SIMPLE_OUSTER_DRIVER_EXCEPTION_H

namespace ros2_simple_ouster_driver
{

/**
 * @class OusterDriverException
 * @brief Thrown when Ouster lidar driver encounters a fatal error
 */
    class OusterDriverException : public std::runtime_error
    {
    public:
        /**
         * @brief A constructor for ros2_ouster::OusterDriverException
         * @param description string to display the exception message
         */
        explicit OusterDriverException(const std::string description)
                : std::runtime_error(description) {}
    };

}  // namespace ros2_ouster

#endif //ROS2_SIMPLE_OUSTER_DRIVER_EXCEPTION_H
