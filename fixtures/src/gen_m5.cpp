// Golden data for milestone 5, from FAST-LIO's own code: motion compensation
// through ImuProcess::Process (UndistortPcl) on synthetic moving frames, and
// the iterated update (update_iterated_dyn_share_modified) driven by fixed
// point-to-plane correspondences with FAST-LIO's Jacobian layout.

#include <omp.h>  // esekfom.hpp calls omp_get_wtime() without including it

#include <algorithm>
#include <new>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "IMU_Processing.hpp"
#include "json_out.hpp"

namespace {

std::mt19937_64 rng(20261029);
json::Writer out(stdout);

double uni(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
V3D rand_v3(double s) {
  double x = uni(-s, s), y = uni(-s, s), z = uni(-s, s);
  return V3D(x, y, z);
}

typedef esekfom::esekf<state_ikfom, 12, input_ikfom> Filter;

void write_quat(const Eigen::Quaterniond &q) {
  double v[4] = {q.w(), q.x(), q.y(), q.z()};
  out.values(v, 4);
}

void write_state(const state_ikfom &s) {
  out.begin_object();
  out.key("pos"); out.matrix(s.pos);
  out.key("rot"); write_quat(s.rot);
  out.key("off_r"); write_quat(s.offset_R_L_I);
  out.key("off_t"); out.matrix(s.offset_T_L_I);
  out.key("vel"); out.matrix(s.vel);
  out.key("bg"); out.matrix(s.bg);
  out.key("ba"); out.matrix(s.ba);
  out.key("grav"); out.matrix(s.grav.vec);
  out.end_object();
}

void write_cloud(const PointCloudXYZI &c) {
  out.begin_array();
  for (const auto &p : c.points) {
    double v[5] = {p.x, p.y, p.z, p.intensity, p.curvature};
    out.values(v, 5);
  }
  out.end_array();
}

void h_unused(state_ikfom &, esekfom::dyn_share_datastruct<double> &) {}

// Initialisation frames, then frames with the sensor turning and accelerating.
void undistort_case() {
  // ImuProcess leaves last_lidar_end_time_ uninitialised. FAST-LIO allocates it
  // at program start, in fresh zeroed memory; do the same so it reads 0.
  alignas(ImuProcess) static unsigned char storage[sizeof(ImuProcess)] = {};
  ImuProcess &imu = *new (storage) ImuProcess();
  Eigen::Quaterniond r_ext(Eigen::AngleAxisd(0.3, V3D(0.2, -0.5, 1).normalized()));
  V3D t_ext(0.04, -0.02, 0.03);
  imu.set_extrinsic(t_ext, r_ext.toRotationMatrix());
  imu.set_gyr_cov(V3D(0.1, 0.1, 0.1));
  imu.set_acc_cov(V3D(0.1, 0.1, 0.1));
  imu.set_gyr_bias_cov(V3D(1e-4, 1e-4, 1e-4));
  imu.set_acc_bias_cov(V3D(1e-4, 1e-4, 1e-4));
  Filter kf;
  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw, h_unused, 3, epsi);

  out.key("undistort"); out.begin_object();
  out.key("ext_r"); out.matrix(r_ext.toRotationMatrix());
  out.key("ext_t"); out.matrix(t_ext);
  out.key("frames"); out.begin_array();
  double t = 50.0;
  for (int frame = 0; frame < 6; ++frame) {
    bool moving = frame >= 3;
    MeasureGroup meas;
    meas.lidar_beg_time = t;
    meas.lidar_end_time = t + 0.1;
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    for (int i = 0; i < 400; ++i) {
      PointType p;
      V3D v = rand_v3(20);
      p.x = float(v.x()); p.y = float(v.y()); p.z = float(v.z());
      p.intensity = float(uni(0, 255));
      // Distinct times so the C++ sort order is unique.
      p.curvature = float(0.25 * i + uni(0.0, 0.2));
      p.normal_x = p.normal_y = p.normal_z = 0;
      cloud->push_back(p);
    }
    std::shuffle(cloud->points.begin(), cloud->points.end(), rng);
    meas.lidar = cloud;
    out.begin_object();
    out.key("lidar_beg_time"); out.num(meas.lidar_beg_time);
    out.key("lidar_end_time"); out.num(meas.lidar_end_time);
    out.key("cloud"); write_cloud(*cloud);
    out.key("imu"); out.begin_array();
    int n = frame < 3 ? 4 : 20;
    for (int k = 0; k < n; ++k) {
      double ts = t + 0.005 * (k + 1);
      V3D acc = V3D(0.02, -0.01, 1.0) + rand_v3(0.01);
      V3D gyr = V3D(0.002, -0.001, 0.003) + rand_v3(0.001);
      if (moving) {
        acc += V3D(0.3 * std::sin(10 * ts), 0.2, -0.1);
        gyr += V3D(0.8 * std::cos(7 * ts), -0.5, 1.2);
      }
      sensor_msgs::Imu::Ptr m(new sensor_msgs::Imu());
      m->header.stamp.fromSec(ts);
      m->linear_acceleration.x = acc.x(); m->linear_acceleration.y = acc.y(); m->linear_acceleration.z = acc.z();
      m->angular_velocity.x = gyr.x(); m->angular_velocity.y = gyr.y(); m->angular_velocity.z = gyr.z();
      meas.imu.push_back(m);
      out.begin_object();
      out.key("t"); out.num(ts);
      out.key("acc"); out.matrix(acc);
      out.key("gyro"); out.matrix(gyr);
      out.end_object();
    }
    out.end_array();
    PointCloudXYZI::Ptr undistorted(new PointCloudXYZI());
    imu.Process(meas, kf, undistorted);
    out.key("undistorted"); write_cloud(*undistorted);
    out.key("x"); write_state(kf.get_x());
    out.key("p"); out.matrix(kf.get_P());
    out.end_object();
    t += 0.1;
  }
  out.end_array();
  out.end_object();
}

// Fixed point-to-plane correspondences, evaluated at the current state with
// the Jacobian rows h_share_model builds.
struct Planes {
  std::vector<V3D> body, normal;
  std::vector<double> d;
};
Planes g_planes;

void h_planes(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  size_t m = g_planes.body.size();
  ekfom_data.h_x = Eigen::MatrixXd::Zero(m, 12);
  ekfom_data.h.resize(m);
  for (size_t i = 0; i < m; ++i) {
    V3D be = g_planes.body[i];
    M3D be_cross;
    be_cross << SKEW_SYM_MATRX(be);
    V3D point_this = s.offset_R_L_I * be + s.offset_T_L_I;
    M3D cross;
    cross << SKEW_SYM_MATRX(point_this);
    V3D n = g_planes.normal[i];
    V3D world = s.rot * point_this + s.pos;
    V3D C(s.rot.conjugate() * n);
    V3D A(cross * C);
    V3D B(be_cross * s.offset_R_L_I.conjugate() * C);
    ekfom_data.h_x.block<1, 12>(i, 0) << n.x(), n.y(), n.z(), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    ekfom_data.h(i) = -(n.dot(world) + g_planes.d[i]);
  }
}

void update_case(const char *name, int m) {
  Filter kf;
  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw, h_planes, 3, epsi);
  state_ikfom truth;
  truth.pos = rand_v3(2);
  truth.rot = SO3(Eigen::Quaterniond(Eigen::AngleAxisd(uni(-1, 1), rand_v3(1).normalized())));
  truth.offset_R_L_I = SO3(Eigen::Quaterniond(Eigen::AngleAxisd(0.05, rand_v3(1).normalized())));
  truth.offset_T_L_I = rand_v3(0.05);
  truth.vel = rand_v3(1);
  truth.grav = S2(vect3(V3D(0.05, -0.02, -1)));
  g_planes = Planes();
  for (int i = 0; i < m; ++i) {
    V3D n = rand_v3(1).normalized();
    V3D be = rand_v3(15);
    V3D world = truth.rot * (truth.offset_R_L_I * be + truth.offset_T_L_I) + truth.pos;
    g_planes.body.push_back(be);
    g_planes.normal.push_back(n);
    g_planes.d.push_back(-n.dot(world));
  }
  // Start a little off the truth, with a dense covariance.
  Eigen::Matrix<double, 23, 1> err;
  for (int k = 0; k < 23; ++k) err(k) = uni(-0.03, 0.03);
  state_ikfom x0 = truth;
  x0.boxplus(err);
  Eigen::Matrix<double, 23, 23> a;
  for (int r = 0; r < 23; ++r)
    for (int c = 0; c < 23; ++c) a(r, c) = uni(-0.05, 0.05);
  Filter::cov p0 = a * a.transpose() + 0.001 * Filter::cov::Identity();
  kf.change_x(x0);
  kf.change_P(p0);
  double solve_time = 0;
  kf.update_iterated_dyn_share_modified(0.001, solve_time);

  out.key(name); out.begin_object();
  out.key("x0"); write_state(x0);
  out.key("p0"); out.matrix(p0);
  out.key("body"); out.begin_array(); for (auto &v : g_planes.body) out.matrix(v); out.end_array();
  out.key("normal"); out.begin_array(); for (auto &v : g_planes.normal) out.matrix(v); out.end_array();
  out.key("d"); out.begin_array(); for (double v : g_planes.d) out.num(v); out.end_array();
  out.key("x"); write_state(kf.get_x());
  out.key("p"); out.matrix(kf.get_P());
  out.end_object();
}

}  // namespace

int main() {
  std::FILE *json_stream = json::take_stdout();
  out.reset(json_stream);
  out.begin_object();
  out.key("source"); out.str("hku-mars/FAST_LIO@7cc4175");
  undistort_case();
  update_case("update_many", 200);
  update_case("update_few", 10);
  out.end_object();
  std::fputc('\n', json_stream);
  std::fclose(json_stream);
  return 0;
}
