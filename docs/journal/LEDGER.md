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
