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
