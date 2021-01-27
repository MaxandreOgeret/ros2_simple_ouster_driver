#ifndef ROS2_SIMPLE_OUSTER_DRIVER_SENSOR_H
#define ROS2_SIMPLE_OUSTER_DRIVER_SENSOR_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "exception.h"

using namespace std;
using namespace ouster;

const int N_SCANS = 5;
const size_t UDP_BUF_SIZE = 65536;

namespace ros2_simple_ouster_driver {

    class Wrapper {
    public:
        string ip_sensor;
        string ip_dest;
        int lidar_port;
        int imu_port;
        sensor::lidar_mode lidar_mode;
        sensor::timestamp_mode timestamp_mode;
        std::shared_ptr<sensor::client> handle;
        sensor::sensor_info info;
        std::vector<LidarScan> scans;
        const sensor::packet_format* pf;
        ScanBatcher* batch_to_scan;
        std::unique_ptr<uint8_t[]> packet_buf;
        sensor::client_state st;
//        PacketMsg lidar_packet, imu_packet;

        Wrapper(string ip_sensor, string ip_dest, int lidar_port, int imu_port, string lidar_mode, string timestamp_mode);
        void configure();
        void get();
    };
}


#endif //ROS2_SIMPLE_OUSTER_DRIVER_SENSOR_H