// Runs FAST-LIO's own mapping code (src/laserMapping.cpp, compiled unchanged
// against ros_shim/) on an input recorded by `lio --dump-input`, and writes
// one CSV row per synchronised frame in the same format as `lio --frames-csv`.
//
// The recorded frames are already preprocessed, so the harness feeds them to
// FAST-LIO's buffers the way its callbacks would, then runs the body of
// FAST-LIO's main loop after each event.
//
// Usage: run_fastlio <input.bin> <frames.csv> [key=value ...]
//   keys: max_iteration filter_size_surf filter_size_map cube_side_length
//         det_range gyr_cov acc_cov b_gyr_cov b_acc_cov extrinsic_est_en
//         extrinsic_R (9 comma-separated) extrinsic_T (3) time_offset

#define main fast_lio_main_unused
#include "laserMapping.cpp"
#undef main

#include <cstring>
#include <map>
#include <sstream>

// The input is already preprocessed; FAST-LIO's preprocessing is never called.
Preprocess::Preprocess() {}
Preprocess::~Preprocess() {}
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &, PointCloudXYZI::Ptr &) {}
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &, PointCloudXYZI::Ptr &) {}

namespace {

std::vector<double> numbers(const std::string &s) {
  std::vector<double> v;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) v.push_back(std::stod(item));
  return v;
}

template <class T>
bool read_val(std::FILE *f, T &v) {
  return std::fread(&v, sizeof(T), 1, f) == 1;
}

void write_row(std::FILE *out, int index, const char *kind, int down, int eff) {
  state_ikfom s = kf.get_x();
  V3D p = s.pos + s.rot * s.offset_T_L_I;
  Eigen::Quaterniond q = s.rot * s.offset_R_L_I;
  std::fprintf(out, "%d,%s,%.6f,%.6f,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,0\n",
               index, kind, Measures.lidar_beg_time, Measures.lidar_end_time, down, eff,
               ikdtree.Root_Node ? ikdtree.validnum() : 0, p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z(),
               s.vel[0], s.vel[1], s.vel[2], s.bg[0], s.bg[1], s.bg[2], s.ba[0], s.ba[1], s.ba[2]);
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <input.bin> <frames.csv> [key=value ...]\n", argv[0]);
    return 2;
  }
  std::map<std::string, std::string> kv;
  for (int i = 3; i < argc; ++i) {
    std::string a = argv[i];
    auto eq = a.find('=');
    kv[a.substr(0, eq)] = a.substr(eq + 1);
  }
  auto get = [&](const char *k, double d) { return kv.count(k) ? std::stod(kv[k]) : d; };

  // ImuProcess leaves last_lidar_end_time_ uninitialised; FAST-LIO's global
  // instance gets fresh zeroed memory at startup. Pin that down.
  alignas(ImuProcess) static unsigned char imu_storage[sizeof(ImuProcess)] = {};
  p_imu = shared_ptr<ImuProcess>(new (imu_storage) ImuProcess(), [](ImuProcess *) {});

  // The parameter block of FAST-LIO's main, with the Mid-360 defaults lio uses.
  NUM_MAX_ITERATIONS = int(get("max_iteration", 3));
  filter_size_surf_min = get("filter_size_surf", 0.5);
  filter_size_map_min = get("filter_size_map", 0.5);
  cube_len = get("cube_side_length", 1000);
  DET_RANGE = float(get("det_range", 100));
  gyr_cov = get("gyr_cov", 0.1);
  acc_cov = get("acc_cov", 0.1);
  b_gyr_cov = get("b_gyr_cov", 0.0001);
  b_acc_cov = get("b_acc_cov", 0.0001);
  extrinsic_est_en = get("extrinsic_est_en", 0) != 0;
  time_diff_lidar_to_imu = get("time_offset", 0.0);
  extrinT = kv.count("extrinsic_T") ? numbers(kv["extrinsic_T"]) : std::vector<double>{0, 0, 0};
  extrinR = kv.count("extrinsic_R") ? numbers(kv["extrinsic_R"]) : std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1};
  lidar_type = VELO16;
  p_pre->lidar_type = lidar_type;

  // The setup that follows it in main.
  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
  p_imu->lidar_type = lidar_type;
  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

  std::FILE *in = std::fopen(argv[1], "rb");
  std::FILE *out = std::fopen(argv[2], "w");
  if (!in || !out) {
    std::fprintf(stderr, "cannot open input or output\n");
    return 1;
  }
  std::fprintf(out, "frame,kind,beg,end,downsampled,effective,map_points,x,y,z,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,proc_ms\n");

  int index = 0;
  uint8_t tag;
  while (read_val(in, tag)) {
    if (tag == 1) {
      double v[7];
      if (std::fread(v, sizeof(double), 7, in) != 7) break;
      sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu());
      msg->header.stamp.fromSec(v[0]);
      msg->linear_acceleration.x = v[1]; msg->linear_acceleration.y = v[2]; msg->linear_acceleration.z = v[3];
      msg->angular_velocity.x = v[4]; msg->angular_velocity.y = v[5]; msg->angular_velocity.z = v[6];
      imu_cbk(msg);
    } else if (tag == 2) {
      double beg;
      uint32_t n;
      if (!read_val(in, beg) || !read_val(in, n)) break;
      PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
      cloud->points.resize(n);
      for (uint32_t i = 0; i < n; ++i) {
        float f[5];
        if (std::fread(f, sizeof(float), 5, in) != 5) return 1;
        PointType &p = cloud->points[i];
        p.x = f[0]; p.y = f[1]; p.z = f[2]; p.intensity = f[3]; p.curvature = f[4];
        p.normal_x = p.normal_y = p.normal_z = 0;
      }
      cloud->width = n;
      cloud->height = 1;
      // standard_pcl_cbk, after p_pre->process.
      mtx_buffer.lock();
      if (beg < last_timestamp_lidar) lidar_buffer.clear();
      lidar_buffer.push_back(cloud);
      time_buffer.push_back(beg);
      last_timestamp_lidar = beg;
      mtx_buffer.unlock();
    } else {
      std::fprintf(stderr, "bad tag %u\n", tag);
      return 1;
    }

    // The body of FAST-LIO's main loop.
    while (sync_packages(Measures)) {
      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        write_row(out, index++, "FirstScan", 0, 0);
        continue;
      }
      p_imu->Process(Measures, kf, feats_undistort);
      state_point = kf.get_x();
      pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
      if (feats_undistort->empty() || (feats_undistort == NULL)) {
        write_row(out, index++, "ImuInit", 0, 0);
        continue;
      }
      flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
      lasermap_fov_segment();
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      feats_down_size = feats_down_body->points.size();
      if (ikdtree.Root_Node == nullptr) {
        if (feats_down_size > 5) {
          ikdtree.set_downsample_param(filter_size_map_min);
          feats_down_world->resize(feats_down_size);
          for (int i = 0; i < feats_down_size; i++)
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
          ikdtree.Build(feats_down_world->points);
        }
        write_row(out, index++, "MapInit", feats_down_size, 0);
        continue;
      }
      if (feats_down_size < 5) {
        write_row(out, index++, "Skipped", feats_down_size, 0);
        continue;
      }
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);
      pointSearchInd_surf.resize(feats_down_size);
      Nearest_Points.resize(feats_down_size);
      double solve_H_time = 0;
      kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
      state_point = kf.get_x();
      map_incremental();
      write_row(out, index++, "Updated", feats_down_size, effct_feat_num);
    }
  }
  std::fclose(out);
  std::fprintf(stderr, "frames: %d\n", index);
  return 0;
}
