// Golden data for milestone 4, produced by the code FAST-LIO runs: the plane
// fit (common_lib.h esti_plane, float), PCL's VoxelGrid downsample as
// configured in laserMapping.cpp, and ikd-Tree operation sequences.

#include <omp.h>  // esekfom.hpp calls omp_get_wtime() without including it

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <pcl/filters/voxel_grid.h>

#include "IMU_Processing.hpp"
#include "json_out.hpp"

namespace {

std::mt19937_64 rng(20261029);
json::Writer out(stdout);

double uni(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
double gauss(double sigma) { return std::normal_distribution<double>(0.0, sigma)(rng); }
V3D rand_v3(double s) {
  double x = uni(-s, s), y = uni(-s, s), z = uni(-s, s);
  return V3D(x, y, z);
}

PointType make_point(const V3D &p, float intensity = 0.f, float curvature = 0.f) {
  PointType pt;
  pt.x = static_cast<float>(p.x());
  pt.y = static_cast<float>(p.y());
  pt.z = static_cast<float>(p.z());
  pt.intensity = intensity;
  pt.curvature = curvature;
  pt.normal_x = pt.normal_y = pt.normal_z = 0.f;
  return pt;
}

// Each point as [x, y, z, intensity, curvature].
void write_points(const PointVector &pts) {
  out.begin_array();
  for (const auto &p : pts) {
    double v[5] = {p.x, p.y, p.z, p.intensity, p.curvature};
    out.values(v, 5);
  }
  out.end_array();
}

void plane_case(const PointVector &pts) {
  Eigen::Matrix<float, 4, 1> abcd;
  bool ok = esti_plane(abcd, pts, 0.1f);
  // Skip sets whose worst residual sits within float noise of the threshold:
  // the accept decision there depends on rounding, not on the algorithm.
  double worst = 0;
  for (const auto &p : pts)
    worst = std::max(worst, std::fabs(double(abcd(0)) * p.x + double(abcd(1)) * p.y + double(abcd(2)) * p.z + abcd(3)));
  if (std::fabs(worst - 0.1) < 1e-4) return;
  out.begin_object();
  out.key("points"); write_points(pts);
  out.key("ok"); out.boolean(ok);
  out.key("abcd"); out.matrix(abcd);
  out.end_object();
}

void plane_cases() {
  out.key("plane"); out.begin_array();
  for (double sigma : {0.0, 0.005, 0.02, 0.05, 0.2}) {
    for (int i = 0; i < 12; ++i) {
      V3D n = rand_v3(1).normalized();
      V3D center = rand_v3(40);
      V3D u = n.cross(V3D(0.3, 0.5, 0.8)).normalized();
      V3D w = n.cross(u);
      PointVector pts;
      for (int k = 0; k < NUM_MATCH_POINTS; ++k)
        pts.push_back(make_point(center + uni(-1, 1) * u + uni(-1, 1) * w + gauss(sigma) * n));
      plane_case(pts);
    }
  }
  // A plane through the origin, where FAST-LIO's A·n = -1 form breaks down.
  {
    PointVector pts;
    for (int k = 0; k < NUM_MATCH_POINTS; ++k) pts.push_back(make_point(V3D(uni(-2, 2), uni(-2, 2), 0.0)));
    plane_case(pts);
  }
  // Collinear points: rank-deficient QR.
  {
    PointVector pts;
    for (int k = 0; k < NUM_MATCH_POINTS; ++k) pts.push_back(make_point(V3D(1, 2, 3) + k * 0.3 * V3D(1, -1, 0.5)));
    plane_case(pts);
  }
  out.end_array();
}

void voxel_case(const PointCloudXYZI::Ptr &cloud, float leaf) {
  pcl::VoxelGrid<PointType> filter;
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(cloud);
  PointCloudXYZI result;
  filter.filter(result);
  out.begin_object();
  out.key("leaf"); out.num(leaf);
  out.key("input"); write_points(cloud->points);
  out.key("output"); write_points(result.points);
  out.end_object();
}

void voxel_cases() {
  out.key("voxel"); out.begin_array();
  auto cloud_in_box = [](int n, double half, double ox) {
    PointCloudXYZI::Ptr c(new PointCloudXYZI());
    for (int i = 0; i < n; ++i) {
      double x = uni(-half, half) + ox, y = uni(-half, half), z = uni(-half, half);
      float in = float(uni(0, 255)), t = float(uni(0, 100));
      c->push_back(make_point(V3D(x, y, z), in, t));
    }
    return c;
  };
  voxel_case(cloud_in_box(3000, 20, 0), 0.5f);
  voxel_case(cloud_in_box(3000, 3, 10), 0.5f);  // dense: many points per voxel
  voxel_case(cloud_in_box(2000, 2, -5), 0.1f);
  voxel_case(cloud_in_box(500, 30, 0), 1e-4f);  // too fine: PCL returns the input unchanged
  out.end_array();
}

}  // namespace

int main() {
  out.begin_object();
  out.key("source"); out.str("hku-mars/FAST_LIO@7cc4175 + PCL VoxelGrid");
  plane_cases();
  voxel_cases();
  out.end_object();
  std::printf("\n");
  return 0;
}
