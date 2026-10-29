# Conversation Ledger

Date-ordered record of what was claimed, disputed, decided, built and tested.
Append-only: newest date last, and existing entries are never rewritten. A past
entry stands even when it turned out to be wrong — correct it with a new entry
that references it.

Format and conventions: the `journal` skill.

---

## 2026-10-28

### 23:12 · M0: repository skeleton created
`session 70807f77` · `main` @ `unknown` · [transcript](transcripts/2026-10-28--70807f77.jsonl)

This session began in the FAST_LIO checkout. Its earlier entries cover the analysis, the JT128 SDK reading, and the decisions behind this repo: exact algorithm, GPL-2.0, pcap-first, ARM target. They are in `/Users/me/projects/FAST_LIO/docs/journal/LEDGER.md` (22:00, 22:06, 22:22, 22:33). This entry covers only the M0 build.

- **decision** — The repo was created at `~/projects/fast-lio-rs`. The plan and decoder spec moved here from the FAST_LIO checkout, as `docs/plan.md` and `docs/hesai_jt128_decoder.md`. Nothing is committed yet; the user decides when.
- **decision** — License: `GPL-2.0-only` SPDX, with FAST_LIO's GPLv2 text copied verbatim. `THIRD_PARTY_NOTICES.md` reproduces three BSD notices, copied from the source files:
  - LOAM (CMU/SwRI, from `laserMapping.cpp`);
  - IKFoM (HKU);
  - Hesai SDK 2.0.

  ikd-Tree is credited as part of FAST-LIO, since its checkout has no license file.
- **decision** — The fixtures image clones FAST_LIO at the pinned commit `7cc4175de6f8…`, with the ikd-Tree submodule, on Ubuntu 22.04. Nothing from FAST-LIO is vendored into this repo.
- **decision** — Workspace setup:
  - Toolchain pinned to 1.99.0, edition 2024.
  - `unsafe_code = "forbid"` across the workspace.
  - `lio-core` depends only on `glam` 0.34 and `rayon` 1.12. The choice between `faer` and hand-written algebra waits for M3.
- **test** — Skeleton checks pass: `cargo fmt --check`, `cargo clippy -D warnings`, `cargo test` (0 tests), and `cargo deny check`. The deny run reports advisories, bans, licenses and sources all ok, with warnings only for allowed licenses not yet used.
- **test** — `cargo-deny` rejects an Apache-2.0-only crate. Adding `nalgebra = "0.35"` to a throwaway copy gave `error[rejected]: … license is not explicitly allowed` for nalgebra 0.35.0.
- **test** — The first fixtures run failed: the image built, but `smoke` segfaulted with exit 139.
  - Cause: `KD_TREE` embeds `MANUAL_Q Rebuild_Logger` with `q[Q_LEN]` and `Q_LEN = 1000000` (`ikd_Tree.h:18`, `:207`, `:265`). That is about 100 MB, and the harness had the tree on the stack. FAST-LIO keeps it as a global (`laserMapping.cpp:120`).
  - Fix: allocate the tree on the heap in `fixtures/src/smoke.cpp`.
  - For the Rust ikd-Tree: do not reserve a fixed million-entry log.
- **test** — After the fix, the fixtures image builds FAST_LIO's headers and `smoke` exits 0.
  - Output: `vel = [0.0999984, 0.000495, 0.0010000000000012]` after 100 × 10 ms predictions with accel input `(0.1, 0, 9.81)` and gravity `S2(0, 0, −9.81)`; `knn_sq_dist = [0.09, 0.14, 0.14, 0.19, 0.24]`.
  - The z-velocity of exactly 0.001 m/s comes from the S2 gravity length of 9.809 (98090/10000) against the 9.81 input. That is the G-mismatch quirk the plan keeps as written, and now it has a number the Rust side must reproduce.
- **note** — M0 status:
  - Met: the fixtures image builds and runs; `cargo-deny` rejects Apache-2.0-only crates; the skeleton passes the local equivalents of every CI step.
  - Not exercised: the GitHub Actions workflow itself, because the repo has no remote yet.
  - Nothing is committed.

### 23:27 · M3: filter math implemented and matched against FAST-LIO
`session 70807f77` · `main` @ `aea2ea4` · [transcript](transcripts/2026-10-28--70807f77.jsonl)

Continuation of the 23:12 entry above.

- **decision** — The user said "Yes, stop asking for permission to work". M0 was committed as `aea2ea4`. From here, milestones and local commits proceed without asking. Publishing, remotes and anything touching the sensor still need their OK.
- **decision** — Linear algebra in `lio-core` is a hand-written fixed-size module (`mat`, `so3`) instead of `glam`/`faer`. Its quaternion product, vector rotation, `toRotationMatrix` and quaternion-from-matrix mirror Eigen's formulas term for term. `glam` was removed; the only dependency is `rayon`.
- **claim** — IKFoM writes `scalar(1/2)` in three places: `esekfom.hpp:312` and `:344` in predict, and `S2.hpp:277` in `S2_Mx`. Integer division makes that scale 0, so `exp(..)` is exactly the identity. In practice:
  - FAST-LIO's predict uses an identity diagonal block for the rotation states, not `Exp(−ω·dt)`.
  - `S2_Mx` with a nonzero delta omits its rotation term.

  The Rust code reproduces this, with comments explaining why.
- **decision** — Fixtures gained `ros_shim/`: plain message structs, a `ros::Time` and logging macros. With it, FAST-LIO's ROS-coupled `src/IMU_Processing.hpp` compiles unchanged, so IMU initialisation is golden-tested through the real `ImuProcess::Process`.
- **test** — All 7 M3 golden tests pass on the first run against `fixtures/golden/m3.json`:
  - `cos_sinc` (13 cases), SO3 exp (48), log (42), A-matrix (12), S2 operations (11 directions, including both antipodal branches);
  - state boxplus/boxminus/oplus (10), process model and Jacobians (10);
  - 40 chained predict steps, with x and P checked at every step to 1e-12 relative;
  - IMU init over 3 frames. P matches with zero tolerance, and `done` turns true on frame 3, after more than 10 samples.
- **test** — Mutation check on a throwaway copy: setting the rotation block to the textbook `Exp(−ω·dt)` fails `predict_sequence` at step 0. The relative error is 2.4e-5 against a 1e-12 tolerance, so the tests do detect the `scalar(1/2)` behaviour.
- **test** — Golden generation is deterministic: two runs of `fixtures/generate.sh` gave the same SHA-1, `212988fc…`.
- **decision** — The CI fixtures job runs on `ubuntu-24.04-arm` and checks the regenerated golden file byte for byte. The data was generated on arm64, and Eigen's SIMD rounding differs on x86_64.
- **note** — `THIRD_PARTY_NOTICES.md` gained MTK's Universitaet Bremen BSD notice, because `so3.rs` and `s2.rs` follow MTK's formulas.

### 23:33 · M4, first half: plane fit and voxel downsample
`session 70807f77` · `main` @ `1b575e8` · [transcript](transcripts/2026-10-28--70807f77.jsonl)

Continuation of the 23:27 entry above.

- **decision** — `plane::esti_plane` reproduces Eigen 3.4's `ColPivHouseholderQR` in `f32`, step for step:
  - pivot choice, using the first index of the maximum;
  - the column-norm downdate, with the √ε threshold;
  - the rank cut-off;
  - Qᵀb applied H₀ first;
  - column-oriented back-substitution that skips zero entries.

  The reject test is written as "no point exceeds the threshold", so a NaN plane is accepted, as in the C++.
- **decision** — `voxel::downsample` reproduces PCL 1.12 `VoxelGrid` as FAST-LIO uses it, in `f32`:
  - all voxels are kept, and xyz, intensity and curvature are averaged;
  - indices come from `floor(v·inv) − min_b`, computed in float;
  - when the leaf is so small that indices would overflow, the input is returned unchanged.

  PCL sorts with Boost spreadsort, which is unstable, so the order points are summed within a voxel is unspecified even in C++. Rust uses a stable sort.
- **test** — Downsample matches PCL on 4 clouds (3000/3000/2000/500 points). The worst relative error is 0 on three of them and 2.3e-7 on the dense cloud, from summation order. The overflow case passes the input through, as PCL does.
- **test** — The plane fit makes the same accept/reject decision as FAST-LIO in all 62 cases (51 accepted). The median coefficient error is 1.5e-6 and the worst is 1.2e-4.
  - The worst cases are ill-conditioned: a patch 34–47 m out on a plane that passes near the origin.
  - Against a double-precision solution, the C++ float result is off by 2.6e-4 and the Rust one by 3.8e-4 (case 19: true d 0.872159, C++ 0.871897, Rust 0.871777). The spread is float32 conditioning inherent to FAST-LIO, not a bug.
  - The test allows 3e-4 per case and requires the median below 5e-6.
- **test** — Regenerating `m3.json` gave a byte-identical file to the committed one.

### 23:43 · M4 complete: ikd-Tree matches FAST-LIO exactly
`session 70807f77` · `main` @ `0d2d8d1` · [transcript](transcripts/2026-10-28--70807f77.jsonl)

Continuation of the 23:33 entry above.

- **claim** — Everything FAST-LIO can observe from ikd-Tree depends only on which points are live: downsampled-insert outcomes, box-delete counts, k-NN results. Tree shape, `nth_element` pivots and background-rebuild timing do not change it (ties aside). Pending lazy-deletion labels only exist inside subtrees already marked fully deleted, which `Search` never enters, so the k-NN search can be `&self` and run in parallel.
  - evidence: `ikd_Tree.cpp` `Push_Down`/`Update`/`Delete_by_range`/`Search`, read in full
- **decision** — `ikd_tree.rs` uses an index-based arena and rebuilds large subtrees in place rather than on a background thread. Unused operations are left out: `Delete_by_point`, `Add_by_range`, `Radius_Search`, `acquire_removed_points`. Both differences are listed in `DEVIATIONS.md` under "do not change results".
- **test** — First ikd run: the counts matched, but one k-NN distance differed by 1 ulp (3.8200521 vs 3.820052). The cause is GCC's default `-ffp-contract=fast` fusing `a*b+c` into FMA on aarch64; Rust never fuses.
  - **decision** — Fixtures now build with `-ffp-contract=off`. The reference then uses plain IEEE arithmetic and gives the same bits on every architecture.
  - All golden files were regenerated. The plane-fit median error dropped from 1.5e-6 to exactly 0; the worst ill-conditioned case is 5.3e-5.
- **test** — The ikd-Tree golden test passes exactly: the build, then 5 rounds of 800 downsampled inserts, 110 plain inserts, 2 box deletes and 40 k-NN queries.
  - Every insert count, delete count and live count matches.
  - All 200 k-NN results match bit for bit, points and squared distances.
  - The final map matches: 1566 points.
- **test** — Mutation checks on throwaway copies:
  - Skipping "keep the point nearest the voxel centre" fails: 800 inserts against 728.
  - An inclusive upper bound in the box test at first went **undetected**, because random data never lands on a face. Points snapped to the voxel grid and placed on delete-box faces were added; the mutation now fails (10 deleted against 9).
- **test** — The generators had been corrupting their JSON: ikd-Tree's thread `printf`s landed in stdout, at varying positions. Fixed with `json::take_stdout()`, which points stdout at stderr and writes JSON to a dup of the original. Two consecutive runs now give the same hashes (`m3` 61e07f42…, `m4` 80648dd8…).

## 2026-10-29

### 00:28 · First real JT128 capture analysed; FAST-LIO's odometry output pinned down
`session 70807f77` · `main` @ `fce2920` · [transcript](transcripts/2026-10-28--70807f77.jsonl)

Continuation of the 23:43 entry above.

- **note** — The user supplied `room-2.pcap`, a real capture: 540 MB, 104 s, placed in the repo root and git-ignored. They asked exactly what form FAST-LIO's odometry takes; where it should go is still TBD.
- **test** — Read-only analysis with a stdlib Python inspector, kept in the scratchpad.
  - 466,361 packets of 1100 bytes; v1.4; status-info version 137 (JT128); `L` = 128, `B` = 2; distance unit 4 mm.
  - Flags `0x23`: sequence number, IMU and weight-factor byte.
  - Return `0x37` (strongest); motor ~599 RPM; 0 sequence gaps.
  - 450 packets per frame at 9.97 Hz; 222.92 µs per packet.
  - Sensor clock 0.367 s behind the host.
  - Details are in `docs/hesai_jt128_decoder.md` §11.
- **verdict → I was right** — The 22:22 claim that the IMU reports g and deg/s is confirmed. The scale fields read 122 and 3049 (±4 g, ±1000 °/s). At rest |acc| = 0.993, gyro noise is 0.1–0.5 per axis (only plausible in °/s), and the handheld peak is 167.
- **verdict → I was wrong** — The spec told the Rust decoder to de-duplicate IMU samples by the u32 IMU counter. On this unit the counter is 0 in every packet. Samples have to be told apart by a change of value, as the SDK does.
  - Measured: 200 Hz, 22–23 packets per sample, interval jitter 4.89–5.15 ms (1st–99th percentile).
  - 11 of 20,784 changes came after a doubled gap, which are identical consecutive samples. The new rule splits such runs back into samples 5 ms apart.
- **test** — The first 66 packets (~15 ms) report azimuth 0 for both blocks while the motor spins up from standby. The decoder will drop packets whose block azimuths don't advance.
- **test** — At rest the IMU's +z points up, like the LiDAR's. The IMU-to-LiDAR x/y relationship and whether +Y points to the marked front are still open.
- **claim** — FAST-LIO's odometry output carries two quirks that sit in the output layer, not the algorithm (`laserMapping.cpp:589-606`):
  - The published `pose.covariance` has its position and rotation blocks swapped relative to ROS's (x, y, z, rx, ry, rz) order. The `k = i<3 ? i+3 : i-3` remap suits the old rotation-first `StatesGroup`, not the position-first IKFoM state.
  - The message is published *before* its covariance is filled in, so each one carries the previous frame's covariance; the first carries zeros.
  - Twist is never set.

  The plan is to emit a correct covariance and the velocity in the Rust output.
