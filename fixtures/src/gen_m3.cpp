// Golden data for milestone 3, produced by FAST-LIO's own code: MTK manifold
// operations, the state's boxplus/boxminus/oplus, the process model, the
// filter's predict step, and IMU initialisation through ImuProcess::Process.
// Inputs come from a fixed-seed generator and are written alongside the outputs.

#include <omp.h>  // esekfom.hpp calls omp_get_wtime() without including it

#include <algorithm>
#include <cstdio>
#include <deque>
#include <random>
#include <vector>

#include "IMU_Processing.hpp"
#include "json_out.hpp"

namespace {

std::mt19937_64 rng(20261028);
json::Writer out(stdout);

double uni(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
V3D rand_v3(double s) { return V3D(uni(-s, s), uni(-s, s), uni(-s, s)); }
Eigen::Quaterniond rand_quat() {
  Eigen::Quaterniond q(uni(-1, 1), uni(-1, 1), uni(-1, 1), uni(-1, 1));
  q.normalize();
  return q;
}

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

state_ikfom rand_state() {
  state_ikfom s;
  s.pos = rand_v3(10);
  s.rot = SO3(rand_quat());
  s.offset_R_L_I = SO3(rand_quat());
  s.offset_T_L_I = rand_v3(0.5);
  s.vel = rand_v3(3);
  s.bg = rand_v3(0.05);
  s.ba = rand_v3(0.3);
  s.grav = S2(vect3(rand_v3(1)));
  return s;
}

void h_unused(state_ikfom &, esekfom::dyn_share_datastruct<double> &) {}

typedef esekfom::esekf<state_ikfom, 12, input_ikfom> Filter;

void init_filter(Filter &kf) {
  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw, h_unused, 4, epsi);
}

// Vectors spanning MTK's small-angle branches as well as ordinary rotations.
std::vector<V3D> sample_vectors() {
  std::vector<V3D> v = {V3D::Zero(), V3D(1e-13, -2e-13, 5e-14), V3D(3e-7, 1e-7, -2e-7),
                        V3D(4e-3, -1e-3, 2e-3), V3D(0.5, -0.2, 0.1), V3D(0.0, 0.0, 3.0)};
  for (int i = 0; i < 6; ++i) v.push_back(rand_v3(2));
  return v;
}

void cos_sinc_cases() {
  out.key("cos_sinc"); out.begin_array();
  for (double x2 : {0.0, 1e-20, 1e-12, 1e-9, 1e-6, 1e-5, 1.2e-4, 1.220703125e-4, 1.3e-4, 1e-2, 0.5, 2.0, 9.0}) {
    auto cs = MTK::cos_sinc_sqrt<double>(x2);
    out.begin_object();
    out.key("x2"); out.num(x2);
    out.key("cos"); out.num(cs.first);
    out.key("sinc"); out.num(cs.second);
    out.end_object();
  }
  out.end_array();
}

void so3_cases() {
  out.key("so3_exp"); out.begin_array();
  for (const V3D &v : sample_vectors())
    for (double scale : {1.0, 0.5, 0.01, 0.0}) {
      SO3 q = SO3::exp(v, scale);
      out.begin_object();
      out.key("v"); out.matrix(v);
      out.key("scale"); out.num(scale);
      out.key("q"); write_quat(q);
      out.end_object();
    }
  out.end_array();

  out.key("so3_log"); out.begin_array();
  std::vector<Eigen::Quaterniond> qs = {Eigen::Quaterniond::Identity(),
                                        Eigen::Quaterniond(-1, 0, 0, 0),
                                        Eigen::Quaterniond(SO3::exp(V3D(1e-12, 0, 0)))};
  for (const V3D &v : sample_vectors()) qs.push_back(SO3::exp(v));
  for (int i = 0; i < 6; ++i) qs.push_back(rand_quat());
  for (const auto &q : qs)
    for (double sign : {1.0, -1.0}) {
      Eigen::Quaterniond qq(sign * q.w(), sign * q.x(), sign * q.y(), sign * q.z());
      V3D v = SO3::log(SO3(qq));
      out.begin_object();
      out.key("q"); write_quat(qq);
      out.key("v"); out.matrix(v);
      out.end_object();
    }
  out.end_array();

  out.key("a_matrix"); out.begin_array();
  for (const V3D &v : sample_vectors()) {
    out.begin_object();
    out.key("v"); out.matrix(v);
    out.key("m"); out.matrix(MTK::A_matrix(vect3(v)));
    out.end_object();
  }
  out.end_array();
}

void s2_cases() {
  std::vector<V3D> dirs = {V3D(1, 0, 0), V3D(-1, 0, 0), V3D(-1, 1e-9, 0), V3D(0, 0, -1), V3D(0.3, -0.2, -0.9)};
  for (int i = 0; i < 6; ++i) dirs.push_back(rand_v3(1));
  out.key("s2"); out.begin_array();
  for (const V3D &d : dirs) {
    S2 g{vect3(d)};
    vect2 delta;
    delta[0] = uni(-0.2, 0.2);
    delta[1] = uni(-0.2, 0.2);
    vect2 zero = vect2::Zero();
    Eigen::Matrix<double, 3, 2> bx, mx, mx0;
    Eigen::Matrix<double, 2, 3> nx;
    Eigen::Matrix3d hat;
    g.S2_Bx(bx);
    g.S2_Mx(mx, delta);
    g.S2_Mx(mx0, zero);
    g.S2_Nx_yy(nx);
    g.S2_hat(hat);
    S2 plus = g;
    plus.boxplus(delta);
    S2 others[3] = {S2(vect3(rand_v3(1))), g, S2(vect3(-g.vec))};
    out.begin_object();
    out.key("vec"); out.matrix(g.vec);
    out.key("delta"); out.matrix(delta);
    out.key("bx"); out.matrix(bx);
    out.key("mx"); out.matrix(mx);
    out.key("mx_zero"); out.matrix(mx0);
    out.key("nx_yy"); out.matrix(nx);
    out.key("hat"); out.matrix(hat);
    out.key("boxplus"); out.matrix(plus.vec);
    out.key("boxminus"); out.begin_array();
    for (const S2 &o : others) {
      vect2 res;
      g.boxminus(res, o);
      out.begin_object();
      out.key("other"); out.matrix(o.vec);
      out.key("res"); out.matrix(res);
      out.end_object();
    }
    out.end_array();
    out.end_object();
  }
  out.end_array();
}

void state_cases() {
  out.key("state_ops"); out.begin_array();
  for (int i = 0; i < 10; ++i) {
    state_ikfom x = rand_state(), y = rand_state();
    Eigen::Matrix<double, 23, 1> dx;
    for (int k = 0; k < 23; ++k) dx(k) = uni(-0.3, 0.3);
    Eigen::Matrix<double, 24, 1> f;
    for (int k = 0; k < 24; ++k) f(k) = uni(-2, 2);
    double dt = uni(0.002, 0.01);

    state_ikfom plus = x;
    plus.boxplus(dx);
    Eigen::Matrix<double, 23, 1> minus;
    x.boxminus(minus, y);
    state_ikfom op = x;
    op.oplus(f, dt);

    out.begin_object();
    out.key("x"); write_state(x);
    out.key("dx"); out.matrix(dx);
    out.key("boxplus"); write_state(plus);
    out.key("y"); write_state(y);
    out.key("boxminus"); out.matrix(minus);
    out.key("f"); out.matrix(f);
    out.key("dt"); out.num(dt);
    out.key("oplus"); write_state(op);
    out.end_object();
  }
  out.end_array();
}

void process_cases() {
  out.key("process"); out.begin_array();
  for (int i = 0; i < 10; ++i) {
    state_ikfom x = rand_state();
    input_ikfom in;
    in.acc = vect3(V3D(0, 0, 9.81) + rand_v3(2));
    in.gyro = vect3(rand_v3(1));
    out.begin_object();
    out.key("x"); write_state(x);
    out.key("acc"); out.matrix(in.acc);
    out.key("gyro"); out.matrix(in.gyro);
    out.key("f"); out.matrix(get_f(x, in));
    out.key("df_dx"); out.matrix(df_dx(x, in));
    out.key("df_dw"); out.matrix(df_dw(x, in));
    out.end_object();
  }
  out.end_array();
}

void predict_case() {
  Filter kf;
  init_filter(kf);
  state_ikfom x0 = rand_state();
  Eigen::Matrix<double, 23, 23> a;
  for (int r = 0; r < 23; ++r)
    for (int c = 0; c < 23; ++c) a(r, c) = uni(-0.1, 0.1);
  Filter::cov p0 = a * a.transpose() + 0.01 * Filter::cov::Identity();
  kf.change_x(x0);
  kf.change_P(p0);
  auto q = process_noise_cov();
  q.block<3, 3>(0, 0).diagonal() = V3D::Constant(uni(0.01, 0.2));
  q.block<3, 3>(3, 3).diagonal() = V3D::Constant(uni(0.01, 0.2));
  q.block<3, 3>(6, 6).diagonal() = V3D::Constant(uni(1e-5, 1e-4));
  q.block<3, 3>(9, 9).diagonal() = V3D::Constant(uni(1e-5, 1e-4));

  out.key("predict"); out.begin_object();
  out.key("x0"); write_state(x0);
  out.key("p0"); out.matrix(p0);
  out.key("q"); out.matrix(q);
  out.key("steps"); out.begin_array();
  for (int i = 0; i < 40; ++i) {
    double dt = uni(0.003, 0.007);
    input_ikfom in;
    in.acc = vect3(V3D(0, 0, 9.81) + rand_v3(1));
    in.gyro = vect3(rand_v3(0.8));
    kf.predict(dt, q, in);
    out.begin_object();
    out.key("dt"); out.num(dt);
    out.key("acc"); out.matrix(in.acc);
    out.key("gyro"); out.matrix(in.gyro);
    out.key("x"); write_state(kf.get_x());
    out.key("p"); out.matrix(kf.get_P());
    out.end_object();
  }
  out.end_array();
  out.end_object();
}

// Drives ImuProcess::Process through initialisation the way laserMapping.cpp
// configures it: 3 frames of 4 samples, so the sample count passes 10 on frame 3.
void imu_init_case() {
  ImuProcess imu;
  Eigen::Quaterniond r_ext = rand_quat();
  V3D t_ext = rand_v3(0.1);
  imu.set_extrinsic(t_ext, r_ext.toRotationMatrix());
  imu.set_gyr_cov(V3D(0.1, 0.1, 0.1));
  imu.set_acc_cov(V3D(0.1, 0.1, 0.1));
  imu.set_gyr_bias_cov(V3D(1e-4, 1e-4, 1e-4));
  imu.set_acc_bias_cov(V3D(1e-4, 1e-4, 1e-4));
  Filter kf;
  init_filter(kf);

  out.key("imu_init"); out.begin_object();
  out.key("ext_r"); out.matrix(r_ext.toRotationMatrix());
  out.key("ext_t"); out.matrix(t_ext);
  out.key("frames"); out.begin_array();
  double t = 100.0;
  for (int frame = 0; frame < 3; ++frame) {
    MeasureGroup meas;
    meas.lidar_beg_time = t;
    out.begin_object();
    out.key("lidar_beg_time"); out.num(t);
    out.key("imu"); out.begin_array();
    for (int k = 0; k < 4; ++k) {
      sensor_msgs::Imu::Ptr m(new sensor_msgs::Imu());
      m->header.stamp.fromSec(t + 0.005 * k);
      V3D acc = V3D(0.02, -0.01, 1.0) + rand_v3(0.01);
      V3D gyr = V3D(0.002, -0.001, 0.003) + rand_v3(0.001);
      m->linear_acceleration.x = acc.x(); m->linear_acceleration.y = acc.y(); m->linear_acceleration.z = acc.z();
      m->angular_velocity.x = gyr.x(); m->angular_velocity.y = gyr.y(); m->angular_velocity.z = gyr.z();
      meas.imu.push_back(m);
      out.begin_object();
      out.key("t"); out.num(m->header.stamp.toSec());
      out.key("acc"); out.matrix(acc);
      out.key("gyro"); out.matrix(gyr);
      out.end_object();
    }
    out.end_array();
    meas.lidar_end_time = t + 0.1;
    PointCloudXYZI::Ptr undistorted(new PointCloudXYZI());
    int infos_before = ros::info_count();
    imu.Process(meas, kf, undistorted);
    out.key("done"); out.boolean(ros::info_count() > infos_before);
    out.key("x"); write_state(kf.get_x());
    out.key("p"); out.matrix(kf.get_P());
    out.end_object();
    t += 0.1;
  }
  out.end_array();
  out.end_object();
}

}  // namespace

int main() {
  out.begin_object();
  out.key("source"); out.str("hku-mars/FAST_LIO@7cc4175");
  cos_sinc_cases();
  so3_cases();
  s2_cases();
  state_cases();
  process_cases();
  predict_case();
  imu_init_case();
  out.end_object();
  std::printf("\n");
  return 0;
}
