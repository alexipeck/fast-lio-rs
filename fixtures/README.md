# fixtures

C++ harnesses that run FAST-LIO's own code on fixed inputs. Their output is the golden data that the Rust tests in `crates/` are checked against. This is how "implements FAST-LIO exactly" gets tested.

The Docker image clones hku-mars/FAST_LIO at the pinned commit; nothing from FAST-LIO is vendored here. The image needs no ROS: only Eigen, Boost headers, PCL and OpenMP.

```bash
docker build -t fast-lio-rs-fixtures fixtures
```

```bash
docker run --rm fast-lio-rs-fixtures
```

`src/smoke.cpp` checks that the toolchain works:
- it runs 100 filter predictions;
- it does one ikd-Tree k-nearest search;
- it prints the results as JSON.

The golden-data generators for each milestone go next to it. `ros_shim/` provides just enough of ROS's headers for FAST-LIO's ROS-coupled sources (such as `IMU_Processing.hpp`) to compile unchanged.

To regenerate every golden file in `golden/`:

```bash
fixtures/generate.sh
```

Generation is deterministic: a fixed seed, and the inputs are written next to the outputs.

| File | Generator | Covers |
|---|---|---|
| `golden/m3.json` | `src/gen_m3.cpp` | MTK exp/log/A-matrix, S2 operations, state boxplus/boxminus/oplus, process model and Jacobians, 40 predict steps, IMU initialisation via `ImuProcess::Process` |
| `golden/m4.json` | `src/gen_m4.cpp` | `esti_plane` (float, 62 neighbourhoods incl. rank-deficient), PCL `VoxelGrid` (4 clouds incl. the index-overflow passthrough), ikd-Tree: build plus 5 rounds of downsampled insert, plain insert, box delete and 40 k-NN queries, with points placed exactly on voxel and box faces |
| `golden/m5.json` | `src/gen_m5.cpp` | Motion compensation through `ImuProcess::Process` (3 still frames, then 3 turning and accelerating frames of 400 points), and the iterated update with fixed point-to-plane matches (200 and 10 measurements, online extrinsic estimation on) |

`src/run_fastlio.cpp` runs FAST-LIO's whole mapping loop. `src/laserMapping.cpp` is compiled unchanged, with its `main` renamed out of the way, and fed an input recorded by `lio replay --dump-input`. It writes the same per-frame CSV as `lio --frames-csv`. Its settings default to FAST-LIO's Mid-360 configuration, so pass the JT128 values `lio` uses (the recorded input already has `blind` and `point_filter_num` applied):

```bash
docker run --rm -v "$PWD/data:/data" fast-lio-rs-fixtures /build/run_fastlio /data/input.bin /data/cpp_frames.csv \
  extrinsic_R=-1,0,0,0,-1,0,0,0,1 filter_size_surf=0.25 filter_size_map=0.25
```

The results on the first JT128 capture (the data stays local) are in [DEVIATIONS.md](../DEVIATIONS.md#floating-point-differences). In short, the two agree to the last bit in single precision until PCL's unspecified voxel order, or a fused multiply-add inside Eigen, changes a last bit. A chaotic stretch of the data then grows that difference, exactly as it does between two builds of FAST-LIO itself.

The image builds with `-ffp-contract=off`, so the compiler adds no fused multiply-adds of its own; Rust never does. Eigen's NEON kernels still use some in double precision.

### Speed against the C++

`crates/lio/examples/run_input.rs` is the Rust twin of `run_fastlio`: same input, same arguments, same CSV. In both, `proc_ms` is the wall time from the input event that completes a frame to its row; reading the input is not counted. `run_fastlio` prints the total at the end.

```bash
cargo run --release -p lio --example run_input -- data/input.bin data/rust_frames.csv \
  extrinsic_R=-1,0,0,0,-1,0,0,0,1 filter_size_surf=0.25 filter_size_map=0.25
```

Results on `room-2.pcap` (1,037 frames over 103.9 s, JT128 settings), both in one linux/arm64 container on an Apple M4 (4 fast and 6 slow cores) with the machine otherwise quiet. Each figure is the median of three interleaved runs of the mapping loop alone; runs agreed within 3%:

| Build | 1 thread | 4 threads | 10 threads |
|---|---|---|---|
| FAST-LIO with its own CMake flags (`-O3 -g`, asserts on). On ARM its CMake leaves OpenMP off | 10.2 s | | |
| The same with OpenMP on, at the 3 threads its CMake picks on large x86 machines | | 6.8 s (3 threads) | |
| FAST-LIO at `-O3 -DNDEBUG` with OpenMP | 9.2 s | 6.2 s | 6.4 s |
| This port | 5.1 s | 2.6 s | 4.4 s |

The port's mapping loop is 2.3× faster than FAST-LIO at its best and 3.9× faster than FAST-LIO as built for ARM. The C++ builds here keep GCC's default fused multiply-adds. Ten threads are slower than four for both: FAST-LIO's parallel loops are short, and the slow cores hold them up.

The whole `lio replay`, which adds decoding and every one of the 114.6 M rays on its own threads, takes 3.4 s with `--threads 4` and 3.3 s with 6 (31× real time) in the same container. Natively on the M4 it takes 2.6–2.7 s with 6 (39× real time).

Where the mapping loop's time goes, in seconds for the whole capture (one run each):

| Stage | C++, 1 thread | Port, 1 thread | C++, 4 threads | Port, 4 threads |
|---|---|---|---|---|
| Undistortion (`UndistortPcl`) | 4.1 | 1.7 | 4.1 | 1.0 |
| Matching (`h_share_model`) | 4.3 | 2.5 | 1.2 | 0.9 |
| Downsampling (PCL `VoxelGrid`) | 0.5 | 0.3 | 0.5 | 0.3 |
| Rest of the iterated update | 0.2 | 0.3 | 0.2 | 0.3 |
| `map_incremental` | 0.1 | 0.1 | 0.1 | 0.1 |

Undistortion is serial in FAST-LIO; the port compensates the points in parallel (DEVIATIONS.md).

## Hesai SDK reference

`hesai/` builds Hesai LiDAR SDK 2.0 at a pinned commit with `jt128_dump`, which decodes a JT128 pcap. It writes the points of chosen frames and every IMU sample as CSV. The `jt128` crate's `sdk_reference` test compares the decoder against them. The CSVs are decoded capture data, so they stay in `data/`:

```bash
docker build -t fast-lio-rs-hesai fixtures/hesai
docker run --rm -v "$PWD/data:/data" fast-lio-rs-hesai /data/room-2.pcap \
  /data/angle_correction.csv /data/firetimes.csv /data/sdk_points.csv /data/sdk_imu.csv 100 101
cargo test --release -p jt128 --test sdk_reference -- --ignored --nocapture
```

On `room-2.pcap`, SDK frames 100 and 101 come out the same: 227,318 points with bit-identical coordinates, and times within 4 ns, the resolution of `dt_ms`. All 20,784 of the SDK's IMU samples match ours by value. The SDK stamps 48 of them one packet (about 222 µs) later than we do:
- the capture's first sample, because the SDK spends the first packet detecting the lidar type;
- every sample whose value first arrives in the packet that starts a new frame, on which the SDK finishes the previous one.

We use the time of the first packet that carries the value.
