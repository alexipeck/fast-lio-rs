# Rust implementation plan

A new repository holding a ROS-free Rust implementation of FAST-LIO (FAST-LIO2) for the Hesai JT128.

File and line references to C++ code (`src/…`, `include/…`) point into [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) at commit `7cc4175`.

**Decided 2026-10-28**
- GPL-2.0, published as public open source.
- This repo, local at first; a remote gets added later.
- Development works from pcaps you record.
- Deployment target is an ARM single-board computer.

## Ground rules

1. **The algorithm is followed exactly.** FAST_LIO `main` @ `7cc4175` defines the behaviour: the estimator, motion compensation, map management and every constant.
   - The Rust code is idiomatic Rust, not a line-by-line port. It must still compute the same thing.
   - Any change in behaviour is opt-in, off by default, and listed in `DEVIATIONS.md`.
2. **Everything else is dropped.** ROS (subscribers, publishers, TF, params, logging), the Livox/Velodyne/Ouster/MARSIM handlers, feature extraction, matplotlib, the timing-log arrays and debug files all go. See the table at the end.
3. **Attribution and license.**
   - The README states that this implements FAST-LIO's algorithm as written in hku-mars/FAST_LIO, names the commit, and cites the papers listed in FAST_LIO's README:
     - *FAST-LIO2: Fast Direct LiDAR-inertial Odometry*
     - *FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter*
   - It also credits ikd-Tree and IKFoM.
   - FAST_LIO is **GPL-2.0**, and an exact implementation is very likely a derivative work. **Decided:** the new repo is GPL-2.0 and published publicly.
   - The decoder follows Hesai's SDK, which is Modified BSD. Keep its notice.
   - **Every dependency must be GPL-2.0-compatible.** That means MIT, BSD, ISC or Zlib, or a dual license that includes one of them.
     - **Apache-2.0-only crates are out.** The FSF treats Apache-2.0 as incompatible with GPLv2.
     - That rules out `nalgebra`: it is Apache-2.0 only (0.35.0 on crates.io).
     - CI enforces the allowlist with `cargo-deny`.
4. **Sensor safety comes first.** The JT128 runs hot. Nothing in this repo wakes, configures or powers the sensor. See "Sensor safety and data capture".

## Repository layout

A Cargo workspace:

| Crate | What it is | Depends on |
|---|---|---|
| `jt128` | Turns UDP packets plus calibration files into `Frame` and `ImuSample`. Covers pcap reading, frame splitting, per-point time, IMU de-duplication by counter, and unit conversion. Follows `docs/hesai_jt128_decoder.md`. | none of the others |
| `lio-core` | The algorithm: state and manifold operations, IMU propagation, motion compensation, voxel downsample, ikd-Tree, plane fit, the iterated update, and map management. It knows nothing about sensors or I/O. | `glam` (MIT OR Apache-2.0, used under MIT) for 3-D types; `faer` (MIT) or a small hand-written fixed-size module for the 23×23 / N×12 algebra; `rayon` (MIT OR Apache-2.0) |
| `lio` (bin) | Config (YAML, same keys as FAST_LIO's `config/*.yaml`), threads, replay (pcap) and live (UDP) modes, outputs (odometry stream, PCD map, trajectory file). | the two above |
| `fixtures` (dev only) | C++ harnesses that run FAST_LIO's own math on canned inputs and write golden files. Built in Docker. Not shipped. | Eigen, FAST_LIO headers |

The interface between the crates is fixed early, in `jt128`:

```rust
pub struct ImuSample { pub t: f64, pub acc: [f64; 3], pub gyro: [f64; 3] } // s, g, rad/s
pub struct Point     { pub x: f32, pub y: f32, pub z: f32, pub intensity: f32, pub dt_ms: f32 }
pub struct Frame     { pub t0: f64, pub points: Vec<Point> }
```

## The algorithm contract

This is what "exactly" means. Each row has to match the C++ cited, and is checked against golden output from that same C++ (see Validation).

| Component | C++ source | What must match |
|---|---|---|
| State | `include/use-ikfom.hpp:12-21` | 23-DOF: pos, rot (SO3), LiDAR→IMU rot (SO3) and trans, vel, gyro bias, accel bias, gravity (S2 with length 98090/10000 = 9.809). Same ordering, because covariance indices depend on it (`IMU_Processing.hpp:206-210`, `laserMapping.cpp:597-606`) |
| Manifold ops | `IKFoM_toolkit/mtk/types/{SOn,S2,vect}.hpp` | Boxplus and boxminus for SO3 and S2, including S2's tangent basis (`S2_Mx`, `S2_Nx_yy`) |
| Process model | `use-ikfom.hpp:47-88` | `get_f`, `df_dx`, `df_dw`. Q filled from `gyr_cov`, `acc_cov`, `b_gyr_cov`, `b_acc_cov` |
| Predict | `esekfom.hpp:279` (`predict`) | State integration and covariance propagation, including the SO3 and S2 Jacobian handling |
| IMU init | `IMU_Processing.hpp:159-214`, `:356-380` | Running mean and covariance over IMU **samples**, finishing after more than 10 samples (`MAX_INI_COUNT`). Gravity is `−mean_acc/|mean_acc|·9.81`, then snapped onto S2. Gyro bias starts at mean_gyr. Initial P diagonal as written |
| Accel scaling | `IMU_Processing.hpp:266` | `acc · 9.81 / |mean_acc|`. This is why accel can arrive in g |
| Propagation and motion compensation | `IMU_Processing.hpp:216-346` | Sort by `dt_ms`. Midpoint IMU averaging. Predict at each IMU sample, then to the frame end. Backward per-point compensation into the end-of-frame body, using that formula (including its own "not accurate!" comment, as written) |
| Frame timing | `laserMapping.cpp:368-424` | `lidar_end_time = beg + last point dt`, with the running-mean fallback. IMU samples taken up to `lidar_end_time`. The first frame is skipped. EKF counts as initialised after 0.1 s (`INIT_TIME`) |
| Local map window | `laserMapping.cpp:231-277` | Cube of `cube_side_length`; moves when the sensor comes within `1.5 × det_range` of an edge, by the same distance. The vacated boxes are deleted from the tree |
| Downsample | `laserMapping.cpp:904-905` (PCL `VoxelGrid`) | Leaf `filter_size_surf`. The output point is the centroid of the voxel's points, with every field averaged |
| Correspondence and residual | `laserMapping.cpp:638-754` | 5-NN (`NUM_MATCH_POINTS`). Search only on iterations where `converge` is true. Reject if the 5th squared distance > 5. Plane by QR solve of `A·n = −1` with a 0.1 m tolerance. Gate `1 − 0.9·|d|/sqrt(‖p_body‖) > 0.9`. Jacobian row layout as written, with the extrinsic columns zeroed when `extrinsic_est_en` is false |
| Iterated update | `esekfom.hpp:1619-1931` | Information form (`(P/R)⁻¹ + HᵀH`) with `R = 0.001`. SO3/S2 corrections to dx and P. Convergence at \|dx_i\| ≤ 0.001 on all 23 dims. `max_iteration`. The `t > 1` exit rule and final covariance update |
| Map increment | `laserMapping.cpp:427-474` | Per-point voxel-centre test against the nearest neighbours, splitting points into "add with downsample" and "add without downsample" |
| ikd-Tree | `include/ikd-Tree/ikd_Tree.{h,cpp}` | Build, kNN (results identical; tree shape may differ), box delete, and insert-with-downsample (keep the point nearest the voxel centre, `Add_Points` around line 478). Rebuild criteria from the constructor defaults FAST-LIO uses: balance 0.6, delete 0.5. Minimum unbalanced size 10, background rebuild at ≥ 1500 points. Rebuild changes only the tree's shape, never its contents |
| Preprocess | `preprocess.cpp` (Velodyne/Ouster path, as a pattern) | Keep every `point_filter_num`-th point. Drop anything within `blind`. Per-point time comes from the decoder, so the fallback that estimates time from yaw is never used |

**Known C++ defects. These get fixed, because they are memory-safety issues, not algorithm.**
- The fixed 100 k arrays `res_last` and `point_selected_surf` (`laserMapping.cpp:76`, `:94`). They become sized `Vec`s.
- The 720 k timing-log arrays are dropped.

**Kept as written, even where questionable.** These are logged in `DEVIATIONS.md` as candidates, all off by default:
- G = 9.81 at initialisation against the S2 length of 9.809.
- The approximate motion-compensation formula.
- The 111 µs block spacing the Hesai SDK hard-codes at every frame rate. This belongs to the decoder, not the algorithm.

## Where the speed comes from, without changing results

- **The correspondence loop runs in parallel with `rayon` on every architecture.** The C++ turns OpenMP off on ARM.
  - Results are written per point by index, then reduced serially in index order, exactly as the C++ does after its parallel loop. The output does not depend on how many threads run.
- **No global state, and buffers are allocated once.** The C++ allocates a `vector` per point in its hot loop (`laserMapping.cpp:663`), and allocates new clouds for every publish.
- **The decoder runs on its own thread**, feeding the estimator through a bounded channel.
- **ikd-Tree rebuilds on a background thread**, as in the C++. Rust ownership replaces the pthread-and-raw-pointer protocol.
- **Replay is deterministic:** the same pcap gives byte-identical output. A test enforces this.
- **Profiling happens on the target ARM board** (open question 1). No speed claims until then.

## Validation

You've decided against a C++-shaped port. So "exact" gets proven component by component, from small C++ harnesses that need **no ROS**.

1. **Golden component tests** (`fixtures/`, built in Docker)
   - The harness calls FAST_LIO's own headers and writes inputs and outputs to files that the Rust tests read:
     - IKFoM: `get_f`, `df_dx`, `df_dw`, `predict`, `update_iterated_dyn_share_modified` with canned `h_x`/`h`
     - `esti_plane`
     - ikd-Tree operation sequences
     - PCL `VoxelGrid`
   - Tolerance is about 1e-9 relative for f64 and 1e-5 for f32. kNN must match as point sets.
2. **Decoder against the Hesai SDK.**
   - The same pcap goes through the SDK (built without ROS, in Docker) and through `jt128`.
   - Per point, XYZ and timestamp must agree. IMU samples must agree once our counter-based de-duplication is accounted for.
3. **A synthetic end-to-end test.**
   - Generate a scene and a trajectory, simulate JT128-like frames and IMU samples, and check pose error.
   - This exercises the whole pipeline with ground truth, without a sensor.
4. **Real-sensor checks** on the captures below:
   - At rest: drift stays near 0 and initialisation succeeds.
   - Walk a loop back to the start: end-point error.
   - Map sharpness: plane thickness on walls and floor.
   - Throughput on the target.

## Sensor safety and data capture

- **Nothing here controls the sensor.**
  - The Rust code only listens on UDP (default port 2368).
  - No PTC commands are compiled in; calibration is loaded from files.
- **Your other project owns the sensor's state:** wake, standby and any time limit.
- **Development runs on recordings.**
  - One short capture session gives material for weeks of work.
  - Every capture command runs under `timeout` with a size cap.
  - The capture tool watches the IMU temperature field in each packet and warns above a threshold. That is a warning only; standby remains your project's job.
- **Live runs come late (milestone 7).** They follow the same lifecycle, and the `lio` binary has a mandatory `--max-seconds`.

**Capture session #1** (run by you; about 5 minutes of sensor time in total)

| # | What | Why |
|---|---|---|
| 0 | Export this unit's angle-correction and firetimes files | Required for decoding |
| 1 | 30 s, completely still | IMU units, noise, init, the counter tick |
| 2 | 30 s, slow hand rotations of about 90° around each axis | Confirms gyro is in deg/s, and the IMU↔LiDAR axes |
| 3 | 2–3 min, walk a loop and stop exactly where you started | End-to-end drift |
| 4 | 30 s, sensor's marked front facing a wall | Confirms +Y is forward |

Run each capture on the machine the sensor streams to, while your project has it awake. Replace `<iface>` with that machine's interface name:

```bash
sudo timeout 35 tcpdump -i <iface> -s 0 -B 65536 -w 01_still.pcap udp port 2368
```

- **Duration:** use `timeout 35` for the 30 s captures and `timeout 200` for the loop walk.
- **Size:** at about 1.15 M points/s, expect roughly 5–8 MB/s.
- **Dropped packets:** M2 counts any gaps using the sequence numbers.
- **IMU:** it rides inside the same packets, so nothing else needs capturing.

## Milestones

| # | Deliverable | Done when |
|---|---|---|
| M0 | New repo: GPL-2.0 `LICENSE`, README with attribution, this plan, the decoder spec, a Cargo workspace, CI (`fmt`, `clippy`, `test`, `cargo-deny` licenses), a Docker image for `fixtures/` | CI green on an empty skeleton; the fixtures image builds FAST_LIO's headers; `cargo-deny` rejects an Apache-2.0-only crate |
| M1 | Capture session #1 (you) | pcaps and calibration files in the repo's `data/` (git-ignored, or LFS) |
| M2 | `jt128` decoder | Matches the Hesai SDK on the M1 pcaps; the checklist in §10 of the decoder spec is answered |
| M3 | `lio-core` math: state, manifold ops, process model, predict, IMU init | Golden tests pass |
| M4 | `lio-core` map: voxel downsample, ikd-Tree, plane fit | Golden tests pass, including ikd-Tree operation sequences |
| M5 | `lio-core` pipeline: motion compensation, iterated update, map window and increment. Replay mode in `lio` | Golden tests for the update pass; the synthetic test is within tolerance; replay is deterministic; odometry on the M1 pcaps |
| M6 | Validation and performance | Real-data metrics recorded; profiled on the ARM board (`aarch64-unknown-linux-gnu`, already installed here); container image for that board; holds 10 Hz in real time |
| M7 | Live mode | Live UDP run under your project's lifecycle, with `--max-seconds` enforced |

M1 can run alongside M0 and M3. M2 needs M1's pcaps. M3 and M4 depend only on M0.

## Dropped from FAST_LIO

| Dropped | Reason |
|---|---|
| `roscpp`, TF, publishers, `nh.param`, `ROS_*` logging, the `Pose6D` msg | Ecosystem. Replaced by `lio`'s config, outputs and `tracing` |
| Livox / Velodyne / Ouster / MARSIM handlers, `livox_ros_driver` | Other sensors |
| `give_feature` and feature extraction | Off by default and Livox-oriented |
| `matplotlibcpp`, `T1`/`s_plot*` arrays, `Log/` debug files | Debug tooling |
| `time_sync_en` and `time_offset_lidar_to_imu` hacks | The JT128's IMU shares the LiDAR's clock |
| `StatesGroup`, `esti_normvector`, `Exp_mat.h`, PCL itself | Unused or replaced |

## Open questions

1. **Which ARM board?** For example a Pi 5 (4× A76) or an RK3588 (4× A76 + 4× A55).
   - Cores and memory set how much of the JT128's ~1.15 M points/s survives `point_filter_num` and the voxel downsample at 10 Hz.
   - On big.LITTLE boards, the `rayon` pool may need pinning to the big cores.
   - Needed before M6.
2. **Outputs.** What will consume the odometry? Your other project, a file, a socket? Needed before M5.
3. **Getting recordings here.** Copy them into the new repo's `data/` folder, or some other path? Needed at M1.
