# fast-lio-rs

A ROS-free Rust implementation of [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (FAST-LIO2) LiDAR-inertial odometry. It targets the Hesai JT128 and reads the sensor's UDP packets directly.

> **Status:** working end to end on a real JT128 capture. The decoder, FAST-LIO's full mapping loop, per-point rays and the corrected path all run, at about 10× real time on an Apple M4. The math, map and update components pass golden tests against FAST-LIO's own C++. See [docs/plan.md](docs/plan.md) for what remains.

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

## Usage

```bash
cargo build --release
```

Replay a recording and write rays, the path and the map:

```bash
target/release/lio replay data/room-2.pcap \
  --angles data/angle_correction.csv --firetimes data/firetimes.csv \
  --extrinsic-r -1,0,0,0,-1,0,0,0,1 --blind 1.0 \
  --rays rays.bin --path path.tum --frames-csv frames.csv --map map.pcd
```

Run live. It only listens on UDP, and `--max-seconds` is mandatory:

```bash
target/release/lio live --max-seconds 60 --bind 0.0.0.0:2368 \
  --angles data/angle_correction.csv --firetimes data/firetimes.csv \
  --extrinsic-r -1,0,0,0,-1,0,0,0,1 --blind 1.0 --rays rays.bin --path path.tum
```

- **`--extrinsic-r -1,0,0,0,-1,0,0,0,1`** is the JT128's IMU-to-LiDAR rotation, measured on the first capture: the IMU's x and y point opposite to the LiDAR's.
- **`--blind 1.0`** drops returns within 1 m. A handheld hemispherical sensor sees its operator, and those points move with it.

**Outputs:**

| Output | Contents |
|---|---|
| `--rays` | For every raw point: time, LiDAR origin and unit beam direction in the path's world frame, range and intensity. Binary layout in `crates/lio/src/lib.rs` (`RayWriter`). |
| `--path` | The corrected LiDAR pose at each frame end, TUM format (`t x y z qx qy qz qw`). |
| `--frames-csv` | Per-frame state and diagnostics. |
| `--map` | The final map, ASCII PCD. |

**Embedding:** use `lio::Pipeline` to embed the same stage in another program. Push each UDP payload; every finished frame comes back with its rays.

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
