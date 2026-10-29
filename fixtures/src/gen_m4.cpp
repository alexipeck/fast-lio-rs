// Golden data for milestone 4, produced by the code FAST-LIO runs: the plane
// fit (common_lib.h esti_plane, float), PCL's VoxelGrid downsample as
// configured in laserMapping.cpp, and ikd-Tree operation sequences.

#include <omp.h>  // esekfom.hpp calls omp_get_wtime() without including it

#include <algorithm>
#include <cmath>
#include <memory>
#include <cstdio>
#include <random>
#include <vector>

#include <pcl/filters/voxel_grid.h>

#include "IMU_Processing.hpp"
#include <ikd-Tree/ikd_Tree.h>
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

void write_xyz(const PointVector &pts) {
  out.begin_array();
  for (const auto &p : pts) {
    double v[3] = {p.x, p.y, p.z};
    out.values(v, 3);
  }
  out.end_array();
}

void write_box(const BoxPointType &b) {
  out.begin_object();
  out.key("min"); { double v[3] = {b.vertex_min[0], b.vertex_min[1], b.vertex_min[2]}; out.values(v, 3); }
  out.key("max"); { double v[3] = {b.vertex_max[0], b.vertex_max[1], b.vertex_max[2]}; out.values(v, 3); }
  out.end_object();
}

// The valid points, sorted, since flatten order depends on tree shape.
PointVector valid_sorted(KD_TREE<PointType> &tree) {
  PointVector all;
  tree.flatten(tree.Root_Node, all, NOT_RECORD);
  std::sort(all.begin(), all.end(), [](const PointType &a, const PointType &b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  });
  return all;
}

// Mirrors FAST-LIO's use of the map: Build once, then per scan a downsampled
// insert, a plain insert, box deletes from the moving local map, and 5-NN
// queries. Contents are compared, not tree shape: rebuild timing (background
// thread) and std::nth_element pivots change the shape but never the contents.
void ikd_cases() {
  const float ds = 0.5f;  // filter_size_map
  std::unique_ptr<KD_TREE<PointType>> tree(new KD_TREE<PointType>());
  tree->set_downsample_param(ds);

  PointVector initial;
  for (int i = 0; i < 3000; ++i) initial.push_back(make_point(rand_v3(30)));
  tree->Build(initial);

  out.key("ikd"); out.begin_object();
  out.key("downsample_size"); out.num(ds);
  out.key("build"); write_xyz(initial);
  out.key("rounds"); out.begin_array();
  for (int round = 0; round < 5; ++round) {
    PointVector current = valid_sorted(*tree);
    V3D cluster = V3D(round % 2 ? 15 : -15, round % 3 ? 15 : -15, 0);

    std::vector<BoxPointType> boxes(2);
    for (int a = 0; a < 3; ++a) {
      boxes[0].vertex_min[a] = -40.f; boxes[0].vertex_max[a] = 40.f;
    }
    boxes[0].vertex_min[round % 3] = 30.f - 8.f * round;  // a slab, like the local-map cube moving
    float c = float(uni(-20, 20));
    for (int a = 0; a < 3; ++a) {
      boxes[1].vertex_min[a] = c - 3.f; boxes[1].vertex_max[a] = c + 3.f;
    }

    // Half near existing map points (exercises downsampling against the map),
    // half new, concentrated in one octant per round to unbalance the tree.
    // Every tenth point is snapped to the voxel grid, so it sits exactly on
    // voxel faces: that pins down the half-open box test.
    PointVector add_ds, add_plain;
    for (int i = 0; i < 800; ++i) {
      V3D v;
      if (i % 2 == 0) {
        const PointType &m = current[std::uniform_int_distribution<size_t>(0, current.size() - 1)(rng)];
        v = V3D(m.x, m.y, m.z) + rand_v3(0.3);
      } else {
        v = cluster + rand_v3(8);
      }
      if (i % 10 == 3) v = (v / ds).array().round().matrix() * ds;
      add_ds.push_back(make_point(v));
    }
    for (int i = 0; i < 100; ++i) add_plain.push_back(make_point(cluster + rand_v3(10)));
    // Points exactly on the faces of this round's delete boxes.
    for (int i = 0; i < 10; ++i) {
      const BoxPointType &b = boxes[i % 2];
      V3D v = rand_v3(2) + V3D(b.vertex_min[0] + b.vertex_max[0], b.vertex_min[1] + b.vertex_max[1], b.vertex_min[2] + b.vertex_max[2]) / 2;
      int a = i % 3;
      v[a] = (i / 2) % 2 ? b.vertex_max[a] : b.vertex_min[a];
      add_plain.push_back(make_point(v));
    }

    int ds_added = tree->Add_Points(add_ds, true);
    tree->Add_Points(add_plain, false);

    int deleted = tree->Delete_Point_Boxes(boxes);

    out.begin_object();
    out.key("add_ds"); write_xyz(add_ds);
    out.key("add_ds_count"); out.num(ds_added);
    out.key("add_plain"); write_xyz(add_plain);
    out.key("delete_boxes"); out.begin_array(); for (const auto &b : boxes) write_box(b); out.end_array();
    out.key("delete_count"); out.num(deleted);
    out.key("valid_num"); out.num(tree->validnum());
    out.key("queries"); out.begin_array();
    for (int q = 0; q < 40; ++q) {
      PointType p = make_point(q % 2 ? rand_v3(35) : cluster + rand_v3(6));
      float max_dist = q % 10 == 9 ? 1.0f : INFINITY;
      PointVector near;
      std::vector<float> dist;
      tree->Nearest_Search(p, 5, near, dist, max_dist);
      out.begin_object();
      out.key("p"); { double v[3] = {p.x, p.y, p.z}; out.values(v, 3); }
      out.key("max_dist"); out.num(std::isinf(max_dist) ? -1.0 : max_dist);
      out.key("near"); write_xyz(near);
      out.key("dist"); out.begin_array(); for (float d : dist) out.num(d); out.end_array();
      out.end_object();
    }
    out.end_array();
    out.end_object();
  }
  out.end_array();
  out.key("final_valid"); write_xyz(valid_sorted(*tree));
  out.end_object();
}

}  // namespace

int main() {
  std::FILE *json_stream = json::take_stdout();
  out.reset(json_stream);
  out.begin_object();
  out.key("source"); out.str("hku-mars/FAST_LIO@7cc4175 + PCL VoxelGrid");
  plane_cases();
  voxel_cases();
  ikd_cases();
  out.end_object();
  std::fputc('\n', json_stream);
  std::fclose(json_stream);
  return 0;
}
