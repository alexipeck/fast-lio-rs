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

### Threads and CPUs

`lio` runs three stages, each on its own thread, joined by bounded queues, plus a pool of workers:

| Stage | Thread name | Work |
|---|---|---|
| `decode` | `lio-decode` | Receives and decodes the packets |
| `lio` | `lio-fastlio` | FAST-LIO's serial steps: IMU propagation, the filter update, the map |
| `output` | `lio` | Collects each frame's rays and writes the outputs |
| `workers` | `lio-worker-N` | The parallel parts: FAST-LIO's matching and motion compensation, and the rays |

The stages overlap: while FAST-LIO works on one frame, the decoder is already decoding the next and the output stage is writing the rays of the one before. A run therefore takes as long as its slowest stage, not the sum of all three. The outputs are the same as on one thread, whatever the number of workers or their placement.

- **`--threads <n>`** sets the number of workers. By default these are the fastest cores where Linux reports core capacities, as on ARM big.LITTLE boards, and otherwise one per CPU.
- **`--cpus <list>`** confines the whole process to a set of CPUs, as `taskset` does: `--cpus 4-7`.
- **`--pin <stage>=<list>`** confines one stage, for example `--pin lio=4 --pin workers=5-7`. Workers get one CPU each when there are as many as CPUs listed. `any` removes a restriction.
- Pinning needs Linux; elsewhere `lio` warns and ignores it.
- **Built-in defaults:** any option can be built in, so a device image needs no flags. Set `LIO_DEFAULT_ARGS` when building, as in `LIO_DEFAULT_ARGS='--cpus 4-7' cargo build --release`, or with `--build-arg` for the `Dockerfile`. Options on the command line override the built-in ones.

What each worker count costs on `room-2.pcap` (104 s of data, natively on an M4; the last column is the CPU the pipeline would use in real time, as a share of one M4 core):

| Workers | Time | CPU time | Real-time CPU share |
|---|---|---|---|
| 1 | 6.6 s | 9.4 s | 9% |
| 2 | 4.0 s | 9.9 s | 9.5% |
| 4 | 2.95 s | 11.7 s | 11% |
| 6 | 2.75 s | 14.4 s | 14% |
| 10 | 2.77 s | 20.1 s | 19% |

Beyond the fast cores, extra workers buy little speed for a lot of CPU, much of it spent spinning while they wait. For two sensors, run one `lio` per sensor, each with its own `--cpus`.

### Embedding and containers

**Embedding:** `lio::Pipeline` runs the same stages in another program, on the caller's thread: push each UDP payload, and every finished frame comes back with its rays. To spread the stages over threads as `lio` does, run a `jt128::Decoder`, a `lio::Estimator` and `lio::Processed::rays` on separate threads, passing events and frames along in order (`crates/lio/src/main.rs`).

**Container:** the `Dockerfile` builds `lio` for the host's architecture, so an ARM board gets an arm64 image. The image is about 50 MB: the binary on Google's distroless base (glibc, no shell). Its usage is in the file's header. Only the sources go into the build context; `data/` never does. The multi-gigabyte images under `fixtures/` (FAST-LIO's C++ with PCL, and Hesai's SDK) are for development only.

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
