# Deviations from FAST-LIO

By default this implementation behaves exactly like hku-mars/FAST_LIO at commit `7cc4175`. Every exception is listed here with what differs, why, and where in the C++ it departs.

## Always on

These fix memory-safety defects in the C++. They don't change results for inputs the C++ handles correctly.

- **`res_last` and `point_selected_surf` are sized to the scan.** In the C++ they are fixed 100,000-element arrays (`src/laserMapping.cpp:76`, `:94`), and more than 100,000 downsampled points write past their end. Here they are `Vec`s that grow with the scan (`lio-core/src/lio.rs`).

## JT128 input (FAST-LIO has no JT128 handler)

- **Preprocessing** follows FAST-LIO's Velodyne/Ouster handlers (`src/preprocess.cpp`): keep every `point_filter_num`-th point in arrival order, then drop points within `blind`. Each point's time comes from the decoder, so the yaw-based time estimate those handlers fall back on is never used.
- **The decoder departs from Hesai's SDK in three places**, all in `docs/hesai_jt128_decoder.md`:
  - Spin-up packets, whose block azimuths don't advance, are dropped.
  - Zero-range points (no return) are dropped.
  - IMU samples that the SDK's value comparison would merge are split back out.
- **IMU timestamps are packet arrival times.** The JT128's IMU counter field is always 0, so samples are stamped with the time of the packet that first carries them. The resulting latency can be offset with `--time-offset` (`time_offset_lidar_to_imu`).

## Outputs FAST-LIO doesn't have

These read the estimator's results and never feed back into it.

- **Per-point rays** (`lio-core/src/rays.rs`): origin, unit direction and range for every raw point, not only the downsampled ones FAST-LIO uses.
  - Each point's pose is interpolated from its frame's IMU timeline and anchored to the corrected end-of-frame state, the way FAST-LIO places points in its map.
  - Points not later than a frame's first timeline pose are extrapolated from it. FAST-LIO leaves them uncompensated in its own map; that affects at most a point or two per frame.
  - Everything but `Exp(ω·dt)` is composed once per IMU segment, and `Exp(ω·dt)` is applied to each point's vectors with Rodrigues' formula. Against composing every point's pose in full, one ray in 20 million differs, in its last float bit.
- **Frames before IMU initialisation get rays too**, from the current state as a constant pose. The sensor must be still during that time anyway.
  - Their reported state carries the configured extrinsic. FAST-LIO's own state only receives it when the initialisation completes, so the C++ would place those frames with an identity mounting. With the JT128's default identity extrinsic the two agree; they differ only when `--extrinsic-r` or `--extrinsic-t` is set.
  - This is the only difference in the per-frame CSV against the C++ harness, and it shows only in its first row or two.
- **The corrected path** in TUM format, the per-frame CSV, and the ASCII PCD map.
  - FAST-LIO publishes its odometry before filling in that frame's covariance.
  - FAST-LIO's covariance has position and rotation blocks swapped relative to ROS's order.
  - No covariance is written here yet.

## Opt-in (off by default)

None yet.

## Implementation differences that do not change results

These don't change anything the algorithm can observe: map contents, search results, filter outputs. The golden tests check that.

- **ikd-Tree rebuilds in place.** FAST-LIO hands subtrees of 1500 or more points to a background thread and replays the operations logged meanwhile. Here they are rebuilt synchronously. Only timing and tree shape differ.
- **ikd-Tree k-nearest search doesn't push down lazy deletion labels.** Pending labels only exist inside subtrees already marked fully deleted, which the search never enters. The search can therefore take `&self` and run in parallel.
- **ikd-Tree nodes live in an index-addressed arena,** not behind raw pointers.
- **Linear algebra is hand-written** to Eigen's formulas. The reference fixtures are built with `-ffp-contract=off`, because GCC fuses multiply-adds on aarch64 by default and Rust never does.
- **The correspondence search runs on all cores** (`rayon`). FAST-LIO's CMake enables its OpenMP loop only on x86. Results are gathered in index order, as the C++ does after its loop, so the output doesn't depend on thread count.
- **Motion compensation runs on all cores too.** The C++ walks the points back from the newest (`UndistortPcl`). Here that walk is replayed on times alone to find each point's IMU segment, then every point is compensated on its own with the same arithmetic.
  - The C++ walk stops moving once it reaches the first point. That point is then compensated again for every earlier segment whose head it is later than. This is replicated.
  - A unit test checks the result bit for bit against the serial loop, including that case.
- **`sync_packages` moves the frame's cloud** out of the buffer instead of copying it; the C++ copies a pointer.
- **`ImuProcess::last_lidar_end_time_` starts at 0.** The C++ never initialises it. FAST-LIO's global instance gets fresh, zeroed memory at startup, so it reads 0 in practice.

## Floating-point differences

The port evaluates FAST-LIO's arithmetic in the C++'s order and precision, including the order in which Eigen's SIMD build (NEON or SSE) sums inside the plane fit. Single-precision plane fits, residuals, world points and k-nearest distances are bit-identical. Two things are not, both below the last bit:

- **Points inside a voxel are summed in arrival order.** PCL 1.12 sorts points by voxel with Boost's spreadsort, which is unstable. The order a voxel's points are summed in, and so the last bit of its centroid, is therefore unspecified even in C++. Here the sort is stable.
- **No fused multiply-adds.** Eigen's NEON kernels fuse multiply-adds inside some double-precision products, whatever the compiler flags. The double-precision state therefore agrees to about 1e-16 relative, not to the bit.

On the first real capture these are the only differences. They matter only because the data is chaotically sensitive: a last-bit change eventually moves one point across a threshold, and the paths drift apart from there. FAST-LIO itself shows the same spread when only its voxel order changes:

| Settings | Compared | Point counts identical through | Furthest apart | Ends apart |
|---|---|---|---|---|
| JT128 (voxels 0.25 m) | Port against C++ with a stable voxel sort | frame 148 (15 s) | 6.9 mm | 3.1 mm |
| | Port against C++ as shipped | frame 6 | 12 mm | 4.2 mm |
| | C++ as shipped against C++ with a stable voxel sort | frame 6 | 12 mm | 3.1 mm |
| Mid-360 (voxels 0.5 m) | Port against C++ with a stable voxel sort | frame 480 (48 s) | 109 mm | 102 mm |
| | Port against C++ as shipped | frame 477 | 158 mm | 149 mm |
| | C++ as shipped against C++ with a stable voxel sort | frame 477 | 102 mm | 56 mm |

All runs: `room-2.pcap`, 1,037 frames over 104 s, `blind` 1.0, the same recorded input (`--dump-input`) for both implementations. The input was decoded with Hesai's sample angle file, so it used the 180° extrinsic that file needs; the unit's own file needs none (README). The stable-sort C++ is the fixtures image with one line of PCL's `voxel_grid.hpp` changed to `std::stable_sort`, built with `PCL_NO_PRECOMPILE`.

