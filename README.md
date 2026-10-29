# fast-lio-rs

A ROS-free Rust implementation of [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (FAST-LIO2) LiDAR-inertial odometry. It targets the Hesai JT128 and reads the sensor's UDP packets directly.

> **Status:** Milestone 0, the skeleton. Nothing runs yet. See [docs/plan.md](docs/plan.md).

## What this is

- **FAST-LIO's algorithm, implemented exactly.** The algorithm is as written in hku-mars/FAST_LIO at commit [`7cc4175`](https://github.com/hku-mars/FAST_LIO/tree/7cc4175): the iterated error-state Kalman filter on manifolds, IMU propagation and motion compensation, the ikd-Tree local map, and every constant.
  - Golden tests generated from FAST-LIO's own C++ check this (`fixtures/`).
  - Any change in behaviour is opt-in and listed in [DEVIATIONS.md](DEVIATIONS.md).
- **Everything else is stripped out.** That covers ROS (nodes, topics, TF, launch files), the other sensor drivers, feature extraction, and the plotting and debug tooling.
- **A native JT128 decoder,** following Hesai's LiDAR SDK 2.0. See [docs/hesai_jt128_decoder.md](docs/hesai_jt128_decoder.md).

## Layout

| Path | Contents |
|---|---|
| `crates/jt128` | JT128 packets → `Frame` and `ImuSample` |
| `crates/lio-core` | The FAST-LIO algorithm; no sensor or I/O code |
| `crates/lio` | The `lio` binary: config, replay and live modes, outputs |
| `fixtures/` | Docker-built C++ harnesses that run FAST-LIO's own code to produce golden test data. Development only |

## Sensor safety

The JT128 runs hot and must stay under external standby management. Nothing in this repository wakes, configures or powers the sensor:

- It only listens for UDP packets.
- Calibration comes from files.
- Live runs require a time limit.

## Credits

This project exists because of FAST-LIO and the work behind it:

- **FAST-LIO,** from the HKU MARS Lab, with contributors Wei Xu, Yixi Cai, Dongjiao He, Fangcheng Zhu, Jiarong Lin, Zheng Liu and Borong Yuan. https://github.com/hku-mars/FAST_LIO
  - *FAST-LIO2: Fast Direct LiDAR-inertial Odometry* ([paper](https://github.com/hku-mars/FAST_LIO/blob/main/doc/Fast_LIO_2.pdf))
  - *FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter* ([arXiv:2010.08196](https://arxiv.org/abs/2010.08196))
- **ikd-Tree,** the incremental k-d tree used for the map. https://github.com/hku-mars/ikd-Tree
- **IKFoM,** the iterated Kalman filter on manifolds toolkit, by Dongjiao He. It ships inside FAST-LIO.
- **Hesai LiDAR SDK 2.0,** the reference for the JT128 packet format. https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0

If you use this in research, cite the FAST-LIO papers above.

## License

GPL-2.0-only, the same as FAST-LIO; see [LICENSE](LICENSE). The notices of the BSD-licensed works this builds on are in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
