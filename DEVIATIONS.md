# Deviations from FAST-LIO

By default this implementation behaves exactly like hku-mars/FAST_LIO at commit `7cc4175`. Every exception gets an entry here. Each entry gives:
- what differs;
- why;
- whether it is on by default, and its config switch;
- the C++ location it departs from.

## Always on

These fix memory-safety defects in the C++. They do not change results for inputs the C++ handles correctly.

None implemented yet. Planned:

- `res_last` and `point_selected_surf` become sized to the scan instead of being fixed 100 000-element arrays. In the C++ (`src/laserMapping.cpp:76`, `:94`), more than 100 000 downsampled points write past their end.

## Opt-in (off by default)

None yet.

## Implementation differences that do not change results

These don't change anything the algorithm can observe: map contents, search results, filter outputs. The golden tests check that.

- **ikd-Tree rebuilds in place.** FAST-LIO hands subtrees of 1500 or more points to a background thread and replays the operations logged meanwhile. Here they are rebuilt synchronously. Only timing and tree shape differ.
- **ikd-Tree k-nearest search doesn't push down lazy deletion labels.** Pending labels only exist inside subtrees already marked fully deleted, which the search never enters. The search can therefore take `&self` and run in parallel.
- **ikd-Tree nodes live in an index-addressed arena,** not behind raw pointers.
- **Voxel downsampling sorts stably.** PCL uses an unstable spreadsort, so within a voxel the order points are summed in, and the last bit of a centroid, is unspecified even in C++.
- **Linear algebra is hand-written** to Eigen's formulas. The reference fixtures are built with `-ffp-contract=off`, because GCC fuses multiply-adds on aarch64 by default and Rust never does. With that, plane fits and k-nearest distances match bit for bit.
