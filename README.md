# fast-lio-rs

A ROS-free Rust implementation of [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (FAST-LIO2) LiDAR-inertial odometry. It targets the Hesai JT128 and reads the sensor's UDP packets directly.

> **Status:** working end to end on a real JT128 capture: the decoder, FAST-LIO's full mapping loop, per-point rays and the corrected path. The whole pipeline, every ray included, runs at about 39× real time on an Apple M4 and 31× in a linux/arm64 container on the same machine.
>
> - **Speed against the C++:** the port's mapping loop runs 2.3× faster than FAST-LIO's C++ at its best, in the same arm64 container on the same input: 2.6 s against 6.2 s for the 104 s capture. Against FAST-LIO as its CMake builds it on ARM, it is 3.9× faster ([fixtures/README.md](fixtures/README.md#speed-against-the-c)).
> - **Decoder:** reproduces Hesai's SDK point for point.
> - **Filter and map:** fed the same input, it reproduces FAST-LIO's own C++ to the last bit in single precision and to about 1e-16 in double. On long runs, chaotic stretches of the data grow those last-bit differences to millimetres or centimetres. FAST-LIO shows the same spread against itself ([DEVIATIONS.md](DEVIATIONS.md#floating-point-differences)).
> - **Rays, against ground truth:** a synthetic JT128-like run in a box room puts ray end points at a 10 mm median and 24 mm at worst. FAST-LIO's C++ gets the same trajectory from the same input.
> - **Calibration:** uses Hesai's *sample* angle file until the unit's own is read out.
>
> See [docs/plan.md](docs/plan.md) for what remains.

## What this is

- **FAST-LIO's algorithm, implemented exactly.** The algorithm is as written in hku-mars/FAST_LIO at commit [`7cc4175`](https://github.com/hku-mars/FAST_LIO/tree/7cc4175): the iterated error-state Kalman filter on manifolds, IMU propagation and motion compensation, the ikd-Tree local map, and every constant.
  - Golden tests generated from FAST-LIO's own C++ check this (`fixtures/`), and so does an end-to-end harness that runs FAST-LIO's unchanged `laserMapping.cpp` on the input a Rust run recorded.
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
  --rays rays.bin --path path.tum --frames-csv frames.csv --map map.pcd
```

Run live. It only listens on UDP, and `--max-seconds` is mandatory:

```bash
target/release/lio live --max-seconds 60 --bind 0.0.0.0:2368 \
  --angles data/angle_correction.csv --firetimes data/firetimes.csv \
  --rays rays.bin --path path.tum
```

The settings are FAST-LIO's Mid-360 configuration except four JT128 defaults, measured on the first capture. Each can be overridden with the flag in brackets (`lio` with no arguments lists them all):

- **IMU-to-LiDAR rotation** `[--extrinsic-r]`: 180° about z (`-1,0,0,0,-1,0,0,0,1`). The IMU's x and y point opposite to the LiDAR's.
- **Blind distance** `[--blind]`: 1.0 m. A handheld hemispherical sensor sees its operator, and those points move with it.
- **Voxel sizes** `[--filter-size-surf]`, `[--filter-size-map]`: 0.25 m, against the Mid-360's 0.5 m. At 0.5 m the room's walls come out doubled. At 0.25 m they are single and half as many 5 cm cells are occupied. The run takes about a third longer: still about 40× real time on an M4.

**Outputs:**

| Output | Contents |
|---|---|
| `--rays` | For every raw point: time, LiDAR origin and unit beam direction in the path's world frame, range and intensity. Binary layout in `crates/lio/src/lib.rs` (`RayWriter`). |
| `--path` | The corrected LiDAR pose at each frame end, TUM format (`t x y z qx qy qz qw`). |
| `--frames-csv` | Per-frame state and diagnostics. |
| `--map` | The final map, ASCII PCD. |

**Threads:** `lio` decodes, runs FAST-LIO, and computes and writes the rays on three threads, joined by bounded queues; the outputs are the same as on one thread. The parallel parts use `--threads` workers. By default these are the fastest cores where Linux reports core capacities, as on ARM big.LITTLE boards, and otherwise one per CPU. Slow cores hold up FAST-LIO's short parallel loops: on the M4 (4 fast + 6 slow cores), 4 workers are nearly as fast as 10 on about 40% less CPU, and in a linux/arm64 container 10 workers take half as long again as 6.

**Embedding:** `lio::Pipeline` runs the same stages in another program, on the caller's thread: push each UDP payload, and every finished frame comes back with its rays. To spread the stages over threads as `lio` does, run a `jt128::Decoder`, a `lio::Estimator` and `lio::Processed::rays` on separate threads, passing events and frames along in order (`crates/lio/src/main.rs`).

**Container:** the `Dockerfile` builds `lio` for the host's architecture, so an ARM board gets an arm64 image. Its usage is in the file's header. Only the sources go into the build context; `data/` never does.

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
