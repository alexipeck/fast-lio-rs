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

`src/run_fastlio.cpp` runs FAST-LIO's whole mapping loop. `src/laserMapping.cpp` is compiled unchanged, with its `main` renamed out of the way, and fed an input recorded by `lio replay --dump-input`. It writes the same per-frame CSV as `lio --frames-csv`. On the first JT128 capture (the data stays local), Rust and C++ agree on every frame's point counts and to about 1 µm in position until the estimate meets a chaotic stretch about 48 s in:

```bash
docker run --rm -v "$PWD/data:/data" fast-lio-rs-fixtures /build/run_fastlio /data/input.bin /data/cpp_frames.csv extrinsic_R=-1,0,0,0,-1,0,0,0,1
```

The image builds with `-ffp-contract=off`, so the reference uses plain IEEE arithmetic (no fused multiply-add) and gives the same bits on every architecture.
