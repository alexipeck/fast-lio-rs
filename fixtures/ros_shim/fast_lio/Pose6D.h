#pragma once
// Plain struct with the fields of FAST-LIO's msg/Pose6D.msg.
#include <array>

namespace fast_lio {
struct Pose6D {
  double offset_time = 0.0;
  std::array<double, 3> acc{}, gyr{}, vel{}, pos{};
  std::array<double, 9> rot{};
};
}  // namespace fast_lio
