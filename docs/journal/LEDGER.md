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

### 00:40 · Output rate, IMU de-duplication, calibration sourcing
`session 2a591fe4` · `main` @ `1f3b759` · [transcript](transcripts/2026-10-29--2a591fe4.jsonl)

This continues the conversation logged above under `session 70807f77` (00:28 entry); the session id changed when it was resumed. This block covers only the latest turn.

- **rebuttal** — The user rejected per-frame output: "One pose per LiDAR frame is a problem, there is simply too much physical motion in 100ms (10 Hz)." They also called the JT128's 200 Hz IMU "very low". They have a 12,794 Hz IMU that is not yet mounted.
- **claim** — FAST-LIO already models motion inside each 100 ms frame: it propagates at every IMU sample and corrects each point. The limit is how often a pose comes out and how late, not the estimate.
  - It processes a frame only after IMU data passes the frame end, and publishes one pose for the frame's end.
  - The per-sample states it computes are used only for motion correction.
  - Higher-rate output needs an addition: forward IMU propagation from the latest corrected state. That doesn't change the estimator.
- **claim** — Shorter LiDAR frames are a decoder choice, so FAST-LIO's maths is unchanged. With Hesai's sample angle file, channel azimuth offsets cluster at 135–225°. Channels below 60° elevation then cover about 225° of the horizon in a half rotation (50 ms) and about 135° in a quarter (25 ms).
- **claim** — De-duplication is required:
  - Each 200 Hz sample repeats in about 22 packets. Fed through raw, FAST-LIO would run 22× the predicts.
  - It would also add about 22× less process noise, because the noise term is `(dt·F_w) Q (dt·F_w)ᵀ`, which scales with dt² per step.
  - IMU initialisation, which needs more than 10 samples, would finish on about 11 copies of one sample.
- **test** — The `lio-core` predict step takes 4.09 µs on the M4 in release mode. That is 0.08% of a core at 200 Hz and 5.2% at 12,794 Hz.
- **claim** — Calibration sourcing, from `lidar.h:295-342` and the PTC parser:
  - The angle correction is per-unit and stored on the sensor. Fetch it with PTC 1.0 command `0x05` on TCP 9347, request bytes `47 74 05 00 00 00 00 00`, which returns CSV text.
  - The SDK never fetches firetimes for the JT128 (only for ATX and QT128); it always loads a file. The SDK ships `JT128_Firetime Correction File.csv`.
  - The user does not need to produce either file.
- **decision** — The capture was moved to `data/room-2.pcap` (git-ignored), with the user's permission.

### 01:17 · Target changes to Point-LIO; per-point ray output; throughput measured
`session 2a591fe4` · `main` @ `c8b9c1c` · [transcript](transcripts/2026-10-29--2a591fe4.jsonl)

Continuation of the 00:40 entry above.

- **decision** — The user wants a full Rust port of Point-LIO, not FAST-LIO: "You found Point-LIO, this is actually what I want ... I will realistically want to do a full Rust port of that."
  - It will be a feeder stage for their processing pipeline.
  - The main output is, for each point, the IMU-corrected sensor position and rotation at that point's time plus the range.
  - A 10 Hz angular depth map, which needs a pose per frame, may also be needed.
  - Points should flow downstream before a sweep finishes; for most uses there's no reason to wait for the full pass.
  - The built-in 200 Hz IMU will be used for now, and the 12,794 Hz IMU later once it's in a rig with known offsets.
  - Clock sync "won't be a problem". The angle file comes from the sensor tomorrow.
- **note** — Point-LIO at `4b86a46` (2026-06-13):
  - **Licence:** the top-level LICENSE is LOAM's BSD-3 text and `package.xml` says BSD; the bundled `include/IKFoM/LICENSE` is GPL-2.0. iVox (`include/ivox`, from Faster-LIO) carries no licence header, and Faster-LIO's own licence was not checked. All of this is compatible with this repo's GPL-2.0.
  - **Map:** iVox, not ikd-Tree.
  - **State:** the output model is 30-DOF (pos, rot, offset_R, offset_T, vel, omg, acc, gravity as a plain vect3, bg, ba).
  - **Per-scan pipeline (`laserMapping.cpp:481-735`):** voxel downsample (`filter_size_surf`), sort by time, group equal timestamps, then a sequential loop. IMU readings are applied as measurement updates at the IMU rate; the state is predicted to each point-time group, followed by a point update.
  - **Options:** native `cut_frame` sub-frames, `publish_odometry_without_downsample`, and saturation handling (`satu_acc`, `satu_gyro`).
- **test** — Points surviving Point-LIO's filters on `room-2.pcap` (frames 100–109, sample angle file, 0.5 m blind):
  - 113,582 raw returns per frame become about 400 at the 0.5 m voxel (≈4,000 updates/s), about 1,000 at 0.3 m and about 2,000 at 0.2 m (≈20,000/s).
  - `point_filter_num` 1–4 barely changes this.
  - This matches the README's "4k-8kHz" odometry claim.
- **test** — Per-update cost on the M4 with `lio-core` parts: 5-NN on a 100k-point ikd-Tree map 1.12 µs, `esti_plane` 0.17 µs, predict 3.7 µs.
- **claim** — Point-LIO cannot update at the JT128's raw 1.15 M points/s.
  - Each update depends on the previous one, so the chain is sequential and extra cores don't help.
  - At roughly 2–3 µs per update on the M4, raw rate needs about 3.5 s of compute per second, and more on an ARM board.
  - Per-point poses for every raw point are still feasible by propagating the latest state to each point's time. That is cheap and parallel, and an output addition rather than a change to the estimator.
- **claim** — Of the FAST-LIO work, the SO3 and matrix code, voxel downsample, decoder and fixtures approach carry over. FAST-LIO's S2 gravity, predict/update and ikd-Tree do not, because Point-LIO uses vect3 gravity, its own estimator and iVox. Point-LIO's `esti_plane` still needs diffing against FAST-LIO's.

### 01:28 · Is map search needed at all? IMU-only timeline measured
`session 2a591fe4` · `main` @ `f6afbd3` · [transcript](transcripts/2026-10-29--2a591fe4.jsonl)

Continuation of the 01:17 entry above.

- **rebuttal** — The user asked: "Why is there any search needed at all? I need every point processed, but I don't really need a pose for each point, I need what I described, a vector and a length."
  - Their requirement: turn each 100 ms of IMU data into a position and rotation timeline, then give every point its time-interpolated origin, beam direction and range.
  - If Point-LIO is the way forward, it becomes a new crate in this repo. If their IMU-timeline approach wins, it goes in a fresh repo.
- **test** — Rotation from the JT128 IMU alone on `room-2.pcap`, handheld part, gyro bias-corrected:
  - Bias 0.14 °/s, which uncorrected adds 0.014° per 100 ms.
  - Rotation per 100 ms window: median 1.17°, 95th percentile 6.35°, max 15.5°. Per 50 ms: median 0.60°, 95th percentile 3.25°.
  - Constant-rate interpolation between 200 Hz samples errs by about 0.0045° at the 99th-percentile angular acceleration (1447 °/s²).
- **claim** — Map search isn't needed to produce rays. It exists only to correct the timeline. Within one window the IMU alone gives:
  - **Rotation:** excellent, per the numbers above.
  - **Translation:** only once a start velocity is known. The IMU can't observe velocity, and integrating acceleration from rest drifts within seconds. With a known start velocity, gravity-direction and accelerometer-bias errors cost under 1 mm over 100 ms.
  - Ignoring translation leaves each sweep's origin off by up to v·Δt: about 10–15 cm per 100 ms at walking pace, half that if referenced to mid-sweep.
  - A world frame that stays fixed across sweeps additionally needs drift correction, i.e. LIO.
- **decision** — Pending the user's answer: if each sweep can stand alone (its own frame, translation ignored or supplied), go IMU-only in a fresh repo. Otherwise do the Point-LIO crate here. The per-point ray stage is the same in both designs.

### 01:39 · Translation and the full path are required; IMU-only dead reckoning measured
`session 2a591fe4` · `main` @ `1544e0e` · [transcript](transcripts/2026-10-29--2a591fe4.jsonl)

Continuation of the 01:28 entry above.

- **decision** — User: "Start velocity is supposed to be 0, at least for the start of the capture. It should maintain the full path." Also: "for my actual use case, the physical motion will be smooth. I do need translation."
- **test** — IMU-only dead reckoning of `room-2.pcap` from rest, with v0 = 0, gyro bias and initial tilt (1.41°) taken from the first 6 s, and accel scaled so |acc| at rest equals 9.81. The script is in the session scratchpad, `tools/dead_reckon.py`.
  - Position error after leaving rest: 0.00 m at 0.8 s (still stationary, sanity check), 1.0 m at 5 s, 3.3 m at 10 s, 13.5 m at 20 s, 36 m at 30 s, 170 m at 60 s, 509 m at 97 s.
  - Speed reaches 12.5 m/s by the end, while the true path stays inside a room a few metres across.
  - The first run had a bug in the script: it multiplied by g twice, giving 86 m/s after 1 s. Caught and fixed before reporting.
- **claim** — FAST-LIO's per-frame undistortion *is* the timeline the user described (`IMU_Processing.hpp:238-336`).
  - It is seeded from the previous frame's LiDAR-corrected state, including velocity.
  - It stores (time, world acc, angular rate, vel, pos, rot) at each IMU sample.
  - Each point interpolates R·Exp(ω·dt) and p + v·dt + ½a·dt² from its bracketing sample.
  - The end state is then corrected and seeds the next frame.

  The user's design plus the correction step is FAST-LIO. Per-point rays come from that timeline, anchored to the corrected end pose. With smooth motion, Point-LIO's advantages (aggressive motion, saturation, kHz-rate correction) aren't needed.
- **decision** — Recommended continuing FAST-LIO in this repo, which is already through M3 and M4, and adding the ray output to M5. Point-LIO is deferred. Awaiting the user's confirmation.

### 02:47 · Overnight run: decoder, full pipeline, rays, live mode
`session 2a591fe4` · `main` @ `c3a27c1` · [transcript](transcripts/2026-10-29--2a591fe4.jsonl)

Continuation of the 01:39 entry above.

- **decision** — User: "Yes, FAST-LIO". Then: "You will be left unattended for the night. I need you to get it done. Getting it done implies the entire thing." The outputs are as described before: a ray per point and the full path.
- **test** — M2 decoder (`560f597`). The `room-2.pcap` integration test (ignored by default; the data stays local) passes:
  - 466,361 packets, 0 sequence gaps, 66 spin-up packets dropped, 1,038 frames 100.32 ms apart.
  - 20,795 IMU samples, 11 recovered from merged runs. Stationary acc (0.024, −0.003, 0.992) g.
  - Decoded in 1.6 s for 104 s of data.
  - The spin-up filter was tightened: the first and 66th packets each have one block at azimuth 0, so the rule is now "block step outside (0, 5°]", not "equal azimuths".
- **test** — M5 pipeline (`c3a27c1`). The first full replay ran end to end at 9.9× real time and produced 114.6 M rays, but the path diverged to 13 km.
  - The input really changes in the data, not because of a bug. Frames 1–2 have 113.5k returns; from frame 4, 25% of beams return nothing and about a quarter of returns fall within 0.5 m. Probably the operator standing next to the sensor.
- **test** — IMU-to-LiDAR rotation sweep over the first 30 s, eight axis-aligned candidates: only a 180° turn about z (`-1,0,0,0,-1,0,0,0,1`) stays bounded (3.8 m furthest from start, median 31% of points matched). The others run hundreds to thousands of metres away with 5–14% matched.
- **test** — Full 104 s with the 180° extrinsic:
  - With `blind` 0.5 it still diverges at about 50 s.
  - With `blind` 1.0 it stays bounded: 13.5 m travelled, 1.66 m furthest from start, ending 0.8 m from start.
  - With 0.3 m voxels it is also bounded (9.9 m, 1.69 m).
  - These diverge: `blind` 1.5, time offset ±5 ms, `point_filter_num` 1. Still to settle: whether the Rust port and the C++ diverge alike (C++ harness pending), or whether this data is on FAST-LIO's edge (operator in view, small room, fast hand motion).
- **test** — Live mode against a local UDP replayer (`udp_replay.py`, localhost only; the sensor was not touched):
  - 53,828 packets received with 0 gaps, 120 frames processed at 5.2× real time.
  - It stopped itself at `--max-seconds`.
- **test** — Ray computation takes 1.82 ms per 113k-point frame on all M4 cores (16 ns/point). Predict takes 3.7 µs.
- **note** — Docker Hub pulls hung tonight for 15+ min at `load metadata for ubuntu:22.04`, although the registry answered HTTP. Builds were moved onto the existing local `fast-lio-rs-fixtures` image via derived Dockerfiles in the scratchpad; the committed Dockerfiles are unchanged.
- **decision** — `ImuProcess::last_lidar_end_time_` is uninitialised in the C++. FAST-LIO's global instance gets zeroed memory, so the generator and harness construct it in zeroed static storage, and the Rust code starts it at 0.

### 02:52 · Rust matches C++ FAST-LIO on the real capture
`session 2a591fe4` · `main` @ `ef6fea7` · [transcript](transcripts/2026-10-29--2a591fe4.jsonl)

Continuation of the 02:47 entry above.

- **test** — `run_fastlio`: FAST-LIO's `laserMapping.cpp` compiled unchanged, with `main` renamed out of the way, against the ROS stand-in. It was fed exactly the input the Rust run recorded (`--dump-input`).
  - Settings: 180° extrinsic and `blind` 1.0, first 400 frames. Every frame's kind, downsampled count and matched-plane count are identical. Positions are within 0.0015 mm and rotations within 0.00004°.
  - Same settings, full 1,037 frames: identical through frame 481. At frame 482 (about 48 s in) a one-point downsample difference appears (393 against 394). After that the paths drift apart to at most 186 mm (frame 835) and end 61 mm and 0.78° apart. Both stay inside the room.
  - With `blind` 0.5, full capture: the C++ diverges too, ending 3.9 km from start against Rust's 10 km. They are identical through frame 478; the chaotic divergence starts at the same point.
- **verdict → I was right** — The question was whether the fragility is in the port or in FAST-LIO. It is FAST-LIO's own behaviour on this handheld data. The port reproduces the C++ to float rounding until the trajectory hits a chaotically sensitive stretch, and the C++ itself diverges with the setting that made Rust diverge.
- **test** — M5 golden tests (`ef6fea7`): undistorted points bit-identical, state and P within 8e-16. The 200-measurement update matches to 1.9e-10 and the 10-measurement update to 3e-14. Regenerating `m3` and `m4` from the rebuilt image gave byte-identical files.
- **test** — The render of the ray output (every 20th ray turned back into points) shows a coherent room with the path inside it, but doubled walls. That is slow drift in this data. Hesai's *sample* angle file was used, because the unit's own file only arrives tomorrow.
- **note** — The disk is 96% full (19 GB free). The recorded inputs (0.2–0.6 GB each) were deleted after use.

### 03:23 · SDK match, bit-exact plane fits, JT128 defaults
`session 2a591fe4` · `main` @ `a8742de` · [transcript](transcripts/2026-10-29--2a591fe4.jsonl)

Continuation of the 02:52 entry above.

- **test** — Decoder against Hesai SDK 2.0 (`fixtures/hesai`, built on the local image), SDK frames 100–101:
  - 227,318 points. Coordinates are bit-identical. Times are within 4 ns, the resolution of `dt_ms`.
  - All 20,784 of the SDK's IMU samples match ours by value. It stamps 48 of them one packet (about 222 µs) later than we do. One is the capture's first sample, because the SDK spends that packet detecting the lidar type. The rest are samples whose value first arrives in a frame's first packet. We keep the earlier time.
  - The test is `crates/jt128/tests/sdk_reference.rs`, ignored by default because the data stays local. M2 is done.
- **test** — The linux/arm64 build (Docker on the M4) replays the whole capture at 9.1× real time. Its path is within 1 mm of the macOS build's through frame 478. After that it drifts up to 17 cm away: the platforms' `libm` differ, and chaos amplifies it.
- **test** — Settings sweep: 180° extrinsic, `blind` 1.0, the same every-20th ray for every run.
  - **Sharpness** is the number of occupied 5 cm cells. The Mid-360 voxels (0.5 m) give 190,148 cells and a 13.5 m path. 0.25 m voxels give 97,251 cells and a 9.8 m path, at 11.3× real time.
  - **Other variants** barely move it:
    - 0.25 m with `point_filter_num` 2: 97,223.
    - 0.2 m with `point_filter_num` 1: 95,351, at 5.4× real time.
    - 0.3 m: 107,250.
    - Four iterations: 97,345.
    - `blind` 0.5: 96,992, and stable.
  - **End points:** the 0.25 m variants end within 1 cm of each other, at about (−0.20, −0.11, 0.54). The 0.5 m run ends 40 cm away.
- **decision** — `lio` now defaults to the JT128's settings: 180° extrinsic, `blind` 1.0 and 0.25 m voxels. `lio-core` and the C++ harness keep FAST-LIO's Mid-360 defaults.
- **verdict → I was wrong** — At 02:52 I put the doubled walls down to slow drift in this data, perhaps made worse by the sample angle file. They came from the 0.5 m voxels: at 0.25 m the walls render single with the same angle file.
- **test** — At 0.25 m, Rust and C++ first differed at frame 3, with 463 matched planes against 462. Per-point debug output from both `h_share_model`s showed:
  - **State:** identical to 1e-16.
  - **Downsampled points:** 889 of 989 differed by a few f32 ULPs. PCL 1.12's `VoxelGrid` sorts with Boost's spreadsort, which is unstable; ours is stable.
  - **With a stable sort in PCL** (one-line experiment, not committed): body points, world points and k-nearest distances became bit-identical. Plane coefficients still differed in about a third of fits, by 1 to 16k ULPs.
- **verdict → I was wrong** — DEVIATIONS.md said plane fits matched bit for bit. On real data they didn't.
  - Eigen's SIMD reductions add four terms as `(x0+x2)+(x1+x3)`, then the rest in order. The port added left to right.
  - Fixed with `eigen_sum` in `plane.rs`. Plane coefficients, residuals, world points and distances are now bit-identical to the default SIMD C++ build.
  - The m4 plane golden test is now exact; it had a 3e-4 tolerance.
- **test** — A second port-side difference, fixed: FAST-LIO stores the match score in a `float` before comparing it with 0.9, but the port compared the double.
- **test** — Full capture, both implementations on the same recorded input:
  - **JT128 settings, against the stable-sort C++:** identical point counts through frame 148, at most 6.9 mm apart.
  - **Mid-360 settings, same comparison:** identical through frame 480.
  - **C++ against itself:** the C++ as shipped against the stable-sort C++ differs from frame 6 (JT128) or frame 477 (Mid-360). It drifts to 12 mm and 102 mm, the same spread as port against C++.
  - **What remains:** the voxel tie order, and Eigen's NEON fused multiply-adds in double products (about 1e-16). The table is in DEVIATIONS.md.
- **decision** — PCL's spreadsort, libstdc++'s sort and Eigen's FMA kernels are not replicated:
  - the tie order is unspecified even in C++ and is no part of FAST-LIO's algorithm;
  - libstdc++ is GPL-3 code;
  - FAST-LIO differs from itself by as much.
- **test** — Replay determinism (`crates/lio/tests/replay.rs`): 150 frames and 14.8 M rays hash identically across two runs, and between 1 thread and many.
- **test** — The repo `Dockerfile` (debian:bookworm-slim, with rustup installing the pinned 1.99.0) was only checked with its base image swapped for the local fixtures image, because the registry is still unreachable.
  - It builds, and its image replays at 9.3× real time.
  - The build context excludes `data/` (190 kB).
- **note** — 12 GB of disk free. The large ray files and recorded inputs were deleted after use.
