cmake_minimum_required(VERSION 3.5)
project(ros2_simple_ouster_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_simple_ouster_driver_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(
        include
)

set(library_name ouster_client)
set(executable_name simple_ouster_driver)

set(dependencies
        rclcpp
        rclcpp_lifecycle
        rclcpp_components
        sensor_msgs
        std_msgs
        geometry_msgs
        ros2_simple_ouster_driver_msgs
        builtin_interfaces
        tf2_ros
        tf2_geometry_msgs
        )

add_library(${library_name} SHARED
        src/ouster/netcompat.cpp
        src/ouster/client.cpp
        src/ouster/lidar_scan.cpp
        src/ouster/types.cpp
        src/driver.cpp
        src/wrapper.cpp
        )

ament_target_dependencies(${library_name}
        ${dependencies}
        )

target_link_libraries(${library_name}
        jsoncpp_lib
        )

add_executable(${executable_name}
        src/main.cpp
        )

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(ouster_client "${PROJECT_NAME}::Driver")
set(node_plugins "${node_plugins}${PROJECT_NAME}::Driver;$<TARGET_FILE:simple_ouster_driver>\n")

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Packet.msg"
        )

install(TARGETS ${executable_name} ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/
        DESTINATION include/
        )

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()