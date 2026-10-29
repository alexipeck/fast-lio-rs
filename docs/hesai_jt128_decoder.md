# Hesai JT128 decoder spec

What a Rust decoder needs in order to turn raw JT128 UDP packets into FAST-LIO inputs: one point frame where every point has a time offset, plus a stream of IMU samples.

**Sources read**
- `HesaiTechnology/HesaiLidar_SDK_2.0` @ `357ff14` (2026-10-27)
- `HesaiTechnology/HesaiLidar_ROS_2.0` @ `3d54078` (2026-10-27)

In the SDK, the JT128 path is the v1.4 parser with type `STR_OTHER`. Paths below are relative to the SDK root. References to `src/…` point into [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) at commit `7cc4175`.

**Confidence tags**
- **[SDK]**: read directly from code.
- **[inferred]**: deduced from code, not stated anywhere.
- **[verify]**: needs a packet capture to settle.

---

## 1. Transport

- **Point data** [SDK]: Sensor → host over UDP, one point packet per datagram. The SDK's default destination port is 2368 (`libhesai/driver_param.h`).
- **Fault messages** [SDK]: These share the stream and start with `CD DC`. Ignore them.
- **Control and calibration** [SDK]: PTC over TCP, port 9347. Command `0x05` returns the angle calibration and `0xA9` returns the firetimes (`libhesai/PtcClient/include/ptc_client.h`). This spec does not cover PTC framing.

## 2. Recognising a JT128 packet [SDK]

| Byte | Value |
|---|---|
| 0–1 | `EE FF` (delimiter) |
| 2 | `1` (protocol major) |
| 3 | `4` (protocol minor) |
| 4 | status-info version **≥ 137 → JT128** (128–136 = OT128, < 128 = Pandar128) |

Reject the datagram if its length differs from the size computed in §3. The SDK does the same (`libhesai/UdpParser/udp_parser.h`).

## 3. Packet layout [SDK]

The packet is packed with no padding and is little-endian throughout, except one UTC form in §4.1. Sources: `libhesai/UdpProtocol/udp_protocol_v1_4.h` and `udp_protocol_header.h`.

### Pre-header and header (12 bytes)

| Off | Size | Field |
|---|---|---|
| 0 | 2 | delimiter `0xFFEE` |
| 2 | 1 | version major |
| 3 | 1 | version minor |
| 4 | 1 | status-info version |
| 5 | 1 | reserved |
| 6 | 1 | `L` = laser (channel) count |
| 7 | 1 | `B` = block count |
| 8 | 1 | echo count (return type of the first block) |
| 9 | 1 | distance unit, in **mm** (metres = value / 1000) |
| 10 | 1 | echo num |
| 11 | 1 | status flags (below) |

**Status flags (byte 11)**

| Bit | Meaning | Effect on layout |
|---|---|---|
| `0x01` | sequence number present | +4 bytes in trailer |
| `0x02` | IMU present | +22 bytes in trailer |
| `0x04` | function safety present | +17 bytes in trailer |
| `0x08` | signature present | +32 bytes at end |
| `0x10` | confidence byte per point | +1 byte per channel unit |
| `0x20` | weight-factor byte per point | +1 byte per channel unit |
| `0x40` | env-light byte per point | +1 byte per channel unit |

Read `L`, `B` and the flags from every packet. Don't hard-code them. The SDK sizes its buffers for at most 400 points per packet and 4000 packets per frame.

### Body (starts at offset 12)

`B` blocks. Each block is a `u16` azimuth in units of 0.01°, followed by `L` channel units in channel order.

**Channel unit**

| Bytes | Field |
|---|---|
| 2 | `u16` distance (× distance unit; 0 = no return) |
| 1 | `u8` reflectivity |
| 0–3 | optional, in this order: confidence (`0x10`), weight factor (`0x20`), env light (`0x40`) |

`unit_size = 3 + popcount(flags & 0x70)`.

On JT128 the weight-factor byte packs two fields: bits 7–6 are the **dirty level** and bits 5–0 are the **noise level**. These can be used to filter points.

### Trailer, in order

| Size | Section | Present when |
|---|---|---|
| 4 | `u32` body CRC | always |
| 17 | function safety | `0x04` |
| 26 | tail (below) | always |
| 4 | `u32` sequence number | `0x01` |
| 22 | IMU (§7) | `0x02` |
| 4 | `u32` tail CRC | always |
| 32 | signature | `0x08` |

```
packet_size = 12 + B*(2 + L*unit_size) + 4 + [17] + 26 + [4] + [22] + 4 + [32]
```

### Tail (26 bytes)

| Off | Size | Field |
|---|---|---|
| +0 | 3 | reserved status 1 (`u16` data, `u8` id) |
| +3 | 3 | reserved status 2 |
| +6 | 3 | reserved status 3 |
| +9 | 2 | azimuth flag: 2 bits per block, block `b` at bit `2*(7-b)`. JT128 does not use it |
| +11 | 1 | operational mode |
| +12 | 1 | return mode (below) |
| +13 | 2 | motor speed. **JT128: units of 0.1 RPM**. The SDK scales by 0.1 for JT128 only |
| +15 | 6 | UTC (§4.1) |
| +21 | 4 | `u32` µs within the current second |
| +25 | 1 | factory info |

**Return mode (tail +12)**

| Code | Mode |
|---|---|
| `0x33` | first |
| `0x34` | second |
| `0x35` | third |
| `0x36` | fourth |
| `0x37` | strongest |
| `0x38` | last |
| `0x39` | dual |
| `0x3B` | first + last |
| `0x3C` | strongest + first |
| `0x3D` | triple |

Codes `0x39` and above mean multi-return.

## 4. Time

### 4.1 Packet time [SDK]

`t_pkt_us = utc_seconds * 1_000_000 + tail_us`

The UTC field has two encodings:

- **Calendar form** (`utc[0] != 0`): the six bytes are year − 1900, month (1–12), day, hour, minute, second, all in UTC. If the year byte is below 70, the SDK returns time 0; treat that as "no time".
  - The SDK converts this with `mktime`, subtracts the local timezone, adds a day and then subtracts 86 400 s. The net result is plain `timegm`. In Rust, compute the epoch directly and never touch the local timezone.
- **Epoch form** (`utc[0] == 0`): bytes 2–5 are a **big-endian** `u32` of Unix seconds.

The SDK can stamp packets with host receive time instead (`use_timestamp_type != 0`). Don't do that here. The IMU and the points share the sensor clock, and FAST-LIO only needs that clock to be consistent.

### 4.2 Per-point time [SDK]

```
t_point_ns = t_pkt_us*1000 + block_offset_ns(b) + firetime_ns[ch]
block_offset_ns(b) = -1_888_000 - 111_000 * floor((B - 1 - b) / k)    // k = 2 if return mode >= 0x39, else 1
```

Source: `udp1_4_parser.h`, `ComputeXYZI`, constants `OTHER_BLOCK_NS_OFFSET{1,2}`.

- The last block fired 1.888 ms before the packet timestamp. Each earlier block steps back a further 111 µs, which matches one 0.4° azimuth step at 10 Hz (900 steps in 100 ms).
- The SDK applies both terms **only when a firetimes file is loaded**. Without one, every point gets the packet time, and that shifts the whole cloud about 1.9 ms relative to the IMU. Keep the block term even without the file.
- **Measured at 10 Hz** (`room-2.pcap`): one packet every 222.92 µs, so 111.46 µs per block, against the SDK's 111 µs.
- **[verify] at 20 Hz:** the SDK hard-codes 111 µs whatever the motor speed.

### 4.3 What FAST-LIO expects

- `t0` = earliest `t_point` in the frame. This becomes `lidar_beg_time`.
- Each point carries `dt_ms = (t_point - t0) / 1e6`. It goes into PCL's `curvature` field (`src/preprocess.cpp:418`), and FAST-LIO sorts by it.

## 5. Geometry [SDK]

```
d   = raw_dist * dist_unit_mm / 1000                               // m; drop raw_dist == 0
az  = raw_az/100 + az_corr[ch] + s * firetime_ns[ch] * (motor_raw*0.1) * 6e-9   // deg
el  = el_corr[ch]                                                   // deg
x = d*cos(el)*sin(az);  y = d*cos(el)*cos(az);  z = d*sin(el)
```

**The firetime azimuth term**
- `s` is +1 when azimuth increases packet to packet and −1 otherwise.
- The SDK applies the term only when `firetimes_flag` is set. Its driver sets the flag on by default.
- The term is the extra rotation during the channel's firing delay. RPM × 6 = deg/s.

**Axes.** Azimuth 0 points along **+Y** and 90° along **+X**, with Z up. That is Hesai's Y-forward, X-right convention, not ROS REP-103's X-forward, Y-left. To convert: `(x_ros, y_ros, z) = (y, −x, z)`. Either do this once in the decoder or fold it into FAST-LIO's extrinsic.

**Optional optical-centre correction** (`distance_correction_flag`, off by default)
- Applies only when `d > 0.09 m`.
- Computes `p = d*u(az,el) + c` with `c = (−0.0076, 0.01367, 0.01274)` m, then takes `d = |p|` and recomputes the angles.
- The JT128 constructor sets `c` in `udp1_4_parser.h`.

**Notes**
- The SDK rounds angles to 1/25600° and looks up sin/cos in two 9.2 M-entry float tables, about 74 MB in total. Compute sin/cos directly in Rust.
- Elevation spans about −4.4° to 90.5°; the SDK's remake config uses this range.
- Per-channel azimuth offsets are large; the sample file has 161°–270°. The channels in one block do not point in one direction.

## 6. Frame splitting [SDK]

**What the SDK does**
- Watches the first-block azimuth of each packet.
- Works out the rotation direction from the last three values.
- Starts a new frame when the azimuth crosses `frame_start_azimuth`, default 0°.
- The packet that triggers the split belongs to the *next* frame. Source: `general_parser.h`, `IsNeedFrameSplit`.

**For Rust:** cut when the first-block azimuth wraps past the start angle in the rotation direction. Use the sequence number (when `0x01` is set) to count dropped packets.

## 7. IMU

It sits in the trailer when status `0x02` is set: 22 bytes, right after the tail and the optional sequence number. Hesai's docs say the IMU data starts at the 31st byte. That matches counting from the start of the tail: 26 + 4 + 1.

| Off | Type | Field | Scale [SDK] |
|---|---|---|---|
| +0 | i16 | temperature | ÷100 → °C |
| +2 | u16 | accel scale | — |
| +4 | u16 | gyro scale | — |
| +6 | u32 | IMU timestamp counter | SDK multiplies by 25, then never uses it. **Always 0** on the unit captured |
| +10 | i16 ×3 | accel x, y, z | `raw * accel_scale / 1e6` |
| +16 | i16 ×3 | gyro x, y, z | `raw * gyro_scale / 1e5` |

### Units [confirmed on `room-2.pcap`]

The SDK never names units. Two pieces of evidence:

- The ROS driver copies the raw values straight into `sensor_msgs/Imu`, next to commented-out helpers named `From_g_To_ms2` and `From_degs_To_rads` (`src/manager/source_driver_ros2.hpp:422-430`).
- On the captured unit the scale fields read 122 and 3049. That is 1.22e-4 g and 0.03049 °/s per LSB, matching a 16-bit IMU set to ±4 g and ±1000 °/s.

So the values are **accel in g** and **gyro in deg/s**. The Hesai ROS driver therefore breaks REP-145.

The capture confirms this:
- Over the first 6.8 s, which are stationary, |accel| is 0.993.
- Gyro noise is 0.1–0.5 per axis. That is plausible in °/s and impossible in rad/s.
- The handheld peak is 167, inside the ±1000 °/s range.

**For FAST-LIO:**
- **Gyro must be converted to rad/s.** Otherwise rotation comes out 57× too large.
- Accel in g is fine as it is. FAST-LIO rescales by the startup mean (`src/IMU_Processing.hpp:266`).

### Timing [SDK]

The SDK stamps each IMU sample with the time of the packet that carries it. Its driver emits a sample only when the six values differ from the previous one (`driver/hesai_lidar_sdk.hpp:270-274`). That causes two problems:

1. **Timing jitter.** The timestamp is off by however long the sample sat before a packet carried it.
2. **Lost samples.** Real consecutive samples with identical values get dropped. That happens most while the sensor is stationary, which is exactly when FAST-LIO estimates gravity and gyro bias.

**The counter can't help on this unit:** it was 0 in all 466,361 packets. Measured instead:
- **Rate:** the six values change every 4.91 ms (median), so the IMU runs at 200 Hz. Each sample repeats in 22 or 23 consecutive packets.
- **Jitter:** the 1st–99th percentile interval between changes is 4.89–5.15 ms. That spread is the timing error from stamping by packet.
- **Merged samples:** 11 of 20,784 changes came after about twice the nominal gap (44–45 packets, ~10 ms). Those are identical consecutive samples, which the SDK's value comparison merges into one.

**For Rust:**
- Detect samples by a change in any of the six raw values.
- Stamp each sample with the packet time of the first packet that carries it.
- Split a run of identical values lasting about k × the nominal 22.4 packets into k samples spaced 5 ms apart. That recovers what the SDK drops.

### Axes [partly measured]

At rest the accelerometer reads (+0.024, −0.004, +0.992) g, so the IMU's +z points up, like the LiDAR's. Whether the IMU's x and y match the LiDAR's x and y is still open.

Nothing in the SDK relates the IMU's axes to the LiDAR's. The LiDAR transform parameter only moves points. Get the relationship from the JT128 manual, or measure it: compare gravity's direction at rest with the ground-plane normal. The result goes into FAST-LIO's `extrinsic_R` / `extrinsic_T`.

## 8. Calibration files [SDK]

These are per-unit files. The copies in the SDK's `correction/` folder are only samples. Pull your own unit's files over PTC or export them with Hesai's tools.

**Angle correction (CSV)**
- Header `Channel,Elevation,Azimuth`. There is also a variant whose first line is `EEFF`, followed by one more line to skip.
- Then 128 rows of `id,elevation_deg,azimuth_deg`.
- The last line is a 64-hex SHA-256 of the preceding text. The SDK only warns when it doesn't match.

**Firetimes (CSV)**
- May start with a UTF-8 BOM.
- Header `channel,firetime/us`.
- Rows of `id,µs` (float). The SDK multiplies by 1000 and truncates to integer ns.

## 9. Left to the Rust decoder

- **CRCs.** Neither CRC field is ever checked by the SDK, and the polynomial is unconfirmed. **[verify]**
- **Multi-return.** FAST-LIO expects one return per beam. Either set the sensor to single return or filter by echo.
- **Noise filtering.** Optionally drop points using the dirty and noise levels (§3).

## 10. Checklist for the first capture

1. Header values: `L`, `B`, distance unit and status flags.
2. Packets per second, and packets per frame at 10 Hz.
3. At rest: |accel| ≈ 1.0 (in g) and gyro ≈ 0. Then rotate the sensor 90° about Z by hand: the integrated gyro should read about 90, which confirms deg/s.
4. The IMU counter tick, from a counter-vs-`t_pkt` fit, and how many packets repeat each sample. Together these give the IMU rate.
5. Whether +Y points to the sensor's marked front.
6. The 20 Hz block spacing (§4.2).

Items 1–4 are answered in §11 from `room-2.pcap`. Items 5 and 6, and the IMU's x/y axes, are still open.

## 11. Measured on `room-2.pcap`

A 104 s indoor capture. The first 6.8 s are stationary; the rest is handheld.

| Item | Value |
|---|---|
| Packets | 466,361 UDP datagrams of 1100 bytes, sensor 192.168.50.201:2368 to host :2368, none other |
| Header | v1.4, status-info version 137 (JT128), `L` = 128, `B` = 2, distance unit 4 mm, echo count 0, echo num 1 |
| Flags | `0x23`: sequence number, IMU, weight-factor byte. So the channel unit is 4 bytes |
| Return / mode | `0x37` strongest (single return); operational mode 0 |
| Motor speed | 5985–5993 raw, i.e. 598.5–599.3 RPM at 0.1 RPM per unit |
| Sequence | 0 gaps, 0 lost |
| Frames | 450 packets and 115,200 point slots per frame, one every 100.32 ms (9.97 Hz); 1038 frames |
| Block step | 0.40° (0.39° twice, encoder jitter) |
| Points | 4.0% no return. Range median 1.81 m, 5th percentile 0.64 m, minimum 0.096 m (the sensor's own surroundings), maximum 62.8 m |
| Clock | Calendar-form UTC. Sensor time runs 0.367 s behind the capture host, so the sensor is not synced to the host |
| IMU temperature | 34.9 °C rising to 37.2 °C over 104 s |

**Decoder rules that follow:**
- **Spin-up packets.** Drop packets whose two block azimuths don't advance by about 0.4°. The first 66 packets after wake (~15 ms) report azimuth 0 for both blocks while the motor spins up.
- **IMU.** De-duplicate and stamp as described in §7.
- **Blind distance.** Set it to about 0.5 m, so returns from the mount and the operator are dropped.

## Interface to the algorithm crate

```rust
pub struct ImuSample { pub t: f64, pub acc: [f64; 3], pub gyro: [f64; 3] }    // s, g or m/s², rad/s
pub struct Point     { pub x: f32, pub y: f32, pub z: f32, pub intensity: f32, pub dt_ms: f32 }
pub struct Frame     { pub t0: f64, pub points: Vec<Point> }                    // t0 in s, sensor clock
```
