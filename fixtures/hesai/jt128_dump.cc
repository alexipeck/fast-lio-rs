// Decodes a JT128 pcap with Hesai SDK 2.0 and writes the points of a range of
// frames plus every IMU sample it emits, as CSV. Reference data for the
// jt128 crate's decoder test.
//
// Usage: jt128_dump <pcap> <angle.csv> <firetimes.csv> <points.csv> <imu.csv> <first> <last>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "hesai_lidar_sdk.hpp"

using namespace hesai::lidar;

int main(int argc, char **argv) {
  if (argc != 8) {
    std::fprintf(stderr, "usage: %s <pcap> <angle.csv> <firetimes.csv> <points.csv> <imu.csv> <first> <last>\n", argv[0]);
    return 2;
  }
  DriverParam param;
  param.input_param.source_type = DATA_FROM_PCAP;
  param.input_param.pcap_path = argv[1];
  param.input_param.correction_file_path = argv[2];
  param.input_param.firetimes_path = argv[3];
  param.decoder_param.pcap_play_synchronization = false;
  std::FILE *points = std::fopen(argv[4], "w");
  std::FILE *imu = std::fopen(argv[5], "w");
  const int first = std::atoi(argv[6]), last = std::atoi(argv[7]);
  std::fprintf(points, "frame,index,x,y,z,intensity,ring,time_s,time_ns\n");
  std::fprintf(imu, "t,ax,ay,az,gx,gy,gz\n");

  HesaiLidarSdk<LidarPointXYZICRTT> sdk;
  std::atomic<int> frame_no{0};
  sdk.RegRecvCallback([&](const LidarDecodedFrame<LidarPointXYZICRTT> &f) {
    int n = frame_no++;
    if (n < first || n > last) return;
    for (uint32_t i = 0; i < f.points_num; ++i) {
      const auto &p = f.points[i];
      std::fprintf(points, "%d,%u,%.9g,%.9g,%.9g,%u,%u,%llu,%u\n", n, i, p.x, p.y, p.z, p.intensity, p.ring,
                   static_cast<unsigned long long>(p.timeSecond), p.timeNanosecond);
    }
  });
  sdk.RegRecvCallback([&](const LidarImuData &d) {
    std::fprintf(imu, "%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", d.timestamp, d.imu_accel_x, d.imu_accel_y, d.imu_accel_z,
                 d.imu_ang_vel_x, d.imu_ang_vel_y, d.imu_ang_vel_z);
  });
  if (!sdk.Init(param)) {
    std::fprintf(stderr, "SDK init failed\n");
    return 1;
  }
  sdk.Start();
  while (!sdk.lidar_ptr_->IsPlayEnded()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::this_thread::sleep_for(std::chrono::seconds(2));
  sdk.Stop();
  std::fclose(points);
  std::fclose(imu);
  std::fprintf(stderr, "frames seen: %d\n", frame_no.load());
  return 0;
}
