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
