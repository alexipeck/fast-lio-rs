// Smoke test for the fixtures image: runs FAST-LIO's filter prediction and an
// ikd-Tree k-nearest search, and prints the results as one JSON object.
// The golden-data generators for each milestone sit next to this file.

#include <omp.h>  // esekfom.hpp calls omp_get_wtime() without including it

#include <cstdio>
#include <memory>
#include <vector>

#include <use-ikfom.hpp>
#include <ikd-Tree/ikd_Tree.h>

static void h_unused(state_ikfom &, esekfom::dyn_share_datastruct<double> &) {}

int main() {
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw, h_unused, 4, epsi);

  state_ikfom x = kf.get_x();
  x.grav = S2(vect3(Eigen::Vector3d(0.0, 0.0, -9.81)));
  kf.change_x(x);

  input_ikfom in;
  in.acc = vect3(Eigen::Vector3d(0.1, 0.0, 9.81));
  in.gyro = vect3(Eigen::Vector3d(0.0, 0.0, 0.01));
  auto Q = process_noise_cov();
  double dt = 0.01;
  for (int i = 0; i < 100; ++i) kf.predict(dt, Q, in);
  x = kf.get_x();

  // KD_TREE embeds a 1,000,000-entry operation queue (about 100 MB), so it must
  // not live on the stack. FAST-LIO keeps it as a global for the same reason.
  std::unique_ptr<KD_TREE<pcl::PointXYZINormal>> tree(new KD_TREE<pcl::PointXYZINormal>());
  std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> grid;
  for (int i = 0; i < 10; ++i)
    for (int j = 0; j < 10; ++j)
      for (int k = 0; k < 10; ++k) {
        pcl::PointXYZINormal p;
        p.x = i * 0.5f; p.y = j * 0.5f; p.z = k * 0.5f;
        grid.push_back(p);
      }
  tree->Build(grid);
  pcl::PointXYZINormal q;
  q.x = 1.1f; q.y = 2.2f; q.z = 0.3f;
  decltype(grid) near;
  std::vector<float> dist;
  tree->Nearest_Search(q, 5, near, dist);

  std::printf("{\"pos\":[%.17g,%.17g,%.17g],\"vel\":[%.17g,%.17g,%.17g],\"P_trace\":%.17g,"
              "\"tree_size\":%d,\"knn_sq_dist\":[",
              x.pos[0], x.pos[1], x.pos[2], x.vel[0], x.vel[1], x.vel[2],
              kf.get_P().trace(), tree->size());
  for (size_t i = 0; i < dist.size(); ++i) std::printf(i ? ",%.9g" : "%.9g", dist[i]);
  std::printf("]}\n");
  return 0;
}
