#include "wrapper.h"
#include <iostream>

namespace ros2_simple_ouster_driver {
    Wrapper::Wrapper(string ip_sensor, string ip_dest, int lidar_port, int imu_port, string lidar_mode,
                     string timestamp_mode)
            : ip_sensor(ip_sensor), ip_dest(ip_dest), lidar_port(lidar_port), imu_port(imu_port) {
      this->lidar_mode = sensor::lidar_mode_of_string(lidar_mode);
      if (!this->lidar_mode) {
        throw new std::invalid_argument("Invalid lidar_mode");
      }

      this->timestamp_mode = sensor::timestamp_mode_of_string(timestamp_mode);
      if (!this->lidar_mode) {
        throw new std::invalid_argument("Invalid timestamp_mode");
      }
    }

    void Wrapper::configure() {
      // Connecting
      handle = sensor::init_client(
              this->ip_sensor,
              this->ip_dest,
              this->lidar_mode,
              this->timestamp_mode,
              this->lidar_port,
              this->imu_port
      );

      if (!this->handle) {
        throw new OusterDriverException("Unable to connect to sensor");
      }

      // Gathering metadata
      info = sensor::parse_metadata(get_metadata(*handle));

      size_t w = info.format.columns_per_frame;
      size_t h = info.format.pixels_per_column;

      // Holds lidar data for an entire rotation of the device
      scans = {N_SCANS, LidarScan{w, h}};
      pf = &sensor::get_format(info);
      batch_to_scan = new ScanBatcher(info.format.columns_per_frame, *pf);

      // buffer to store raw packet data
      packet_buf = std::unique_ptr<uint8_t[]>(new uint8_t[UDP_BUF_SIZE]);

      // --------------------

      std::cerr << "  Firmware version:  " << info.fw_rev
                << "\n  Serial number:     " << info.sn
                << "\n  Product line:      " << info.prod_line
                << "\n  Scan dimensions:   " << w << " x " << h << std::endl;

      // --------------------
    }

    void Wrapper::get() {


      for (int i = 0; i < N_SCANS;) {
        // wait until sensor data is available
        sensor::client_state st = sensor::poll_client(*handle);

        // check for error status
        if (st & sensor::CLIENT_ERROR) {
          std::cerr << "Sensor client returned error state" << std::endl;
          std::exit(EXIT_FAILURE);
        }

        // check for lidar data, read a packet and add it to the current batch
        if (st & sensor::LIDAR_DATA) {
          if (!sensor::read_lidar_packet(*handle, packet_buf.get(), *pf)) {
            std::cerr << "Failed to read a packet of the expected size" << std::endl;
            std::exit(EXIT_FAILURE);
          }

          // batcher will return "true" when the current scan is complete
          if ((*batch_to_scan)(packet_buf.get(), scans[i])) {
            // LidarScan provides access to azimuth block data and headers
            auto n_invalid = std::count_if(
                    scans[i].headers.begin(), scans[i].headers.end(),
                    [](const LidarScan::BlockHeader &h) {
                        return h.status != 0xffffffff;
                    });
            // retry until we receive a full scan
            if (n_invalid == 0) i++;
          }
        }

        // check if IMU data is available (but don't do anything with it)
        if (st & sensor::IMU_DATA) {
          sensor::read_imu_packet(*handle, packet_buf.get(), *pf);
        }
      }
    }
}