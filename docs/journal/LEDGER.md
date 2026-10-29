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
