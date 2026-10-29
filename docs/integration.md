# Integrating `lio`

This is for anyone, person or model, connecting `lio` to another program. It covers what `lio` does, what it needs, what it produces and how to run it. Everything here was checked against the code and the first real capture, `room-2.pcap`, on 2026-10-29. The repository's [README](../README.md) and [plan](plan.md) have the history and the test evidence.

## 1. What `lio` is

- **What it does:** it turns the Hesai JT128's UDP packets into one **ray** for every returned point. Each ray holds the LiDAR's position at that point's time, the unit beam direction and the measured range, all in one fixed world frame. It also gives the corrected path of the sensor.
- **How:** it runs [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), LiDAR-inertial odometry, exactly as the C++ does. That uses the JT128's built-in 200 Hz IMU, and needs no ROS.
- **Where it sits:** it is a feeder stage. It reads packets (live from a socket, or from a pcap recording), and something downstream consumes the rays.
- **Two forms:**
  - **A program, `lio`,** which writes the rays as a binary stream to a file, a pipe or stdout.
  - **Rust crates (`jt128`, `lio-core`, `lio`),** which deliver the rays in-process.

### What `lio` does not do

- **The sensor:** it never wakes, configures or powers the sensor, and never sends it anything. It only listens on UDP.
- **Calibration:** it doesn't fetch the angle-correction file. Another program must fetch it from the sensor and save it (§3.2).
- **Clocks:** it doesn't synchronise clocks. All times are the sensor's own.
- **Relocalisation and loop closure:** neither is done. Each run starts a new world frame at the sensor's starting pose, and drift accumulates as in any odometry.
- **Not yet:**
  - provisional (pre-correction) rays;
  - a depth map;
  - covariances;
  - 20 Hz operation;
  - an external IMU.

## 2. Rules to keep

1. **The JT128 overheats when left on.** It must stay under the controlling program's standby management. That program decides when the sensor is awake. Never add code to `lio` that sends anything to the sensor.
2. **Live runs have a time limit.** `lio live` refuses to start without `--max-seconds`. When embedding the library, enforce your own limit in the same way.
3. **Recordings are private.** Captures (`data/*.pcap`) and anything decoded from them must never be committed, published or built into images.
4. **Licence: GPL-2.0-only,** as FAST-LIO is.
   - If you link the crates into your own binary and distribute it, the combined binary falls under GPL-2.0.
   - Running `lio` as a separate process and reading its output keeps the two programs separate.

## 3. Inputs

### 3.1 The UDP stream

| Setting | What `lio` expects |
|---|---|
| Protocol | Hesai UDP 1.4, JT128 (status-info version ≥ 137). Other packets on the port, such as fault messages, are skipped |
| Spin rate | 600 RPM: 10 frames per second. 20 Hz is not supported for now |
| Returns | Single return (strongest), as recorded. Dual return is decoded but untested: each echo would become its own point |
| IMU | Must be on in the packets (status flag `0x02`). FAST-LIO cannot run without it |
| Destination | Any address and port that `--bind` covers. The default is `0.0.0.0:2368`, which is also the sensor's usual destination port |

**Data rate.** About 4,500 packets per second, 1,100 bytes each: about 5 MB/s.

**Packet loss.** `lio` counts lost packets as "sequence gaps" in its summary. If the count isn't 0, give the host's UDP sockets a larger receive buffer (`net.core.rmem_default`), or give `lio` more CPU time. Lost packets leave holes in the frames; they don't stop the run.

**Hold the sensor still for the first second after the stream starts.**
- FAST-LIO averages the IMU over the first frames, about 0.2 s, to find gravity and the gyro bias.
- Motion during that time corrupts the bias for the whole run.

**Keep the sensor's clock steady.** If time goes backwards in the stream (a clock step, or PTP locking mid-run), FAST-LIO drops its buffers, as the C++ does. Set up time sync before starting `lio`.

### 3.2 The angle-correction file (`--angles`)

This is the unit's factory calibration: 128 channels, each with an elevation and an azimuth offset. It is different for every unit.
- **Getting it:** the controlling program reads it from the sensor over PTC: TCP port 9347, command `0x05` (decoder spec §8). Save the reply's payload unchanged.
  - The payload is a 4,096-byte flash page: a CSV table, a SHA-256 line, then `0xFF` padding and 4 binary bytes.
  - `lio` reads that page as it is. Do not trim it or convert line endings.
- **Checks `lio` makes:**
  - It checks the SHA-256 and prints a warning if the hash doesn't match.
  - It also warns if it is given Hesai's sample file from the SDK.
  - Use the unit's own file. The sample file holds another unit's azimuth zero, so it turns the point cloud about 170° and leaves the IMU and LiDAR axes misaligned. FAST-LIO then fails.
- **Re-fetching:** fetch the file again only if the sensor is re-calibrated or swapped. Keep it with the unit's serial number.

### 3.3 The firetimes file (`--firetimes`)

These are the per-channel firing delays. They are the same for every JT128 unit.
- **Source:** Hesai's SDK has it at `correction/firetime_correction/JT128_Firetime Correction File.csv`. This repo was checked at SDK commit `357ff14`.
- **Optional, but give it:** without it, point times and azimuths lose the firing-delay correction. That is microseconds and hundredths of a degree.

## 4. Frames, axes and time

### Sensor (LiDAR) frame

These are Hesai's axes, not ROS's (decoder spec §5):
- **+Y:** azimuth 0°. On the housing, the side with the connector.
- **+X:** 90° clockwise from +Y, seen from above. The head spins clockwise seen from above.
- **+Z:** up the dome's axis.

This is right-handed. Converting to ROS REP-103 (x forward, y left) with +Y as forward: `(x_ros, y_ros, z_ros) = (y, −x, z)`. These axes hold only with the unit's own angle file.

### IMU frame

Same axes as the LiDAR. `lio` assumes no rotation and no offset between them.
- **Rotation:** FAST-LIO's own online estimation agrees to within 0.8°.
- **Translation:** taken as 0. The online estimate puts the LiDAR origin about 2 cm above the IMU. That is too small and too weakly measured to apply.

### World frame

All rays and the path are in one world frame per run.
- **Origin:** the IMU's position when FAST-LIO initialises, which is the sensor's position at start-up.
- **Axes:** the sensor's axes at that moment.
- **Gravity:** the world frame is **not levelled**. FAST-LIO keeps the starting orientation and estimates gravity as a vector in this frame, so world +Z is "up" only if the sensor started level.
  - Embedded, the gravity vector is `FrameResult.output.state.grav.vec`, in m/s², about 9.81 long.
  - To level the data, rotate the world so that this vector points along −Z.
  - The `lio` program doesn't output gravity yet.
- **Restarts:** every run of `lio` starts a new world frame. Two runs, or two sensors, have unrelated world frames.

### Time

- **Clock:** all times are in seconds on the sensor's clock, which is Unix time (UTC) as the sensor reports it.
  - The first capture's sensor ran 0.37 s behind its host, because nothing synchronised them.
  - Don't compare these times with the host's clock unless the sensor is synchronised to it.
- **Frames:** a frame is one turn of the head, cut at azimuth 0°: about 100.3 ms and up to 115,200 point slots.
  - About 4% of beams return nothing in a room, and zero-range returns are dropped.
  - A frame therefore has about 110,000 points with a clear view, and fewer when something is close (about 87,000 handheld).

## 5. Outputs

### 5.1 Rays

`lio` makes one ray for every returned point, including points that FAST-LIO itself ignores:
- returns closer than `blind`;
- the points that downsampling drops.

| Field | Type | Meaning |
|---|---|---|
| `t` | f64 | The point's time, in seconds, on the sensor clock |
| `origin` | f32 ×3 | The LiDAR's position in the world frame at `t`, in metres |
| `direction` | f32 ×3 | The unit beam direction in the world frame |
| `range` | f32 | The measured range, in metres. The point is at `origin + range · direction` |
| `intensity` | f32 | Hesai's reflectivity, 0–255 |

**How the pose at `t` is found:**
- Each point's pose comes from the frame's IMU timeline, interpolated at the point's own time as FAST-LIO's motion compensation does it.
- That timeline is then anchored to the frame's LiDAR-corrected end pose.
- Rays are final when they are emitted: a frame's rays are never revised.

**Order:** the decoder's order. That is packet by packet, then block by block, then channel by channel. This is close to time order but not exactly, because the channels fire at staggered times. Use `t` wherever order matters.

**Close returns.**
- **Who is in view:** handheld, the operator is in view. From the first capture, about a quarter of the returns are within 0.5 m.
- **What FAST-LIO does:** it ignores everything closer than `blind`, which is 1.0 m by default.
- **What the ray stream does:** it keeps these returns, so filter them downstream if they get in the way.

**Rays by frame kind:**

| Kind | Code | Rays come from |
|---|---|---|
| `FirstScan` | 0 | The very first, partial frame. One fixed pose (the start) |
| `ImuInit` | 1 | One fixed pose while the IMU is averaged. The sensor must be still |
| `MapInit` | 2 | The IMU timeline. This frame becomes the first map; it gets no correction |
| `Updated` | 3 | The IMU timeline, anchored to the LiDAR-corrected end pose. This is the normal case |
| `Skipped` | 4 | The IMU prediction only: too few points survived for an update |

A normal run starts with one `FirstScan`, one `ImuInit` and one `MapInit` frame (the first 0.3 s), and has only `Updated` frames after that.

**Latency:** a frame's rays come out once the frame ends and has been processed.
- The frame ends about 100 ms after its first point.
- FAST-LIO then waits for an IMU sample past the frame's end, up to about 5 ms.
- Processing per frame, FAST-LIO and rays together, took 3.8 ms median and 5.3 ms at the 99th percentile on an Apple M4. The ARM board will be slower; it hasn't been profiled yet.
- So a frame's last point comes out about 10 ms after it was measured, and its first point about 110 ms after.

**Volume:** about 1.1 million rays per second. That is about 44 MB/s as the binary stream.
- `--ray-stride n` keeps every n-th ray.
- Embedded, the rays are a `Vec<Ray>` per frame, with no copying or encoding.

### 5.2 The binary ray stream (`--rays`)

Everything is little-endian and packed, with no padding.

```text
stream := "LIORAYS1" frame*
frame  := count:u32  begin:f64  end:f64  kind:u8  ray[count]          (21-byte header)
ray    := t:f64  origin:f32×3  direction:f32×3  range:f32  intensity:f32   (40 bytes)
```

- **`begin` and `end`:** the frame's first point time and its end time, in seconds on the sensor clock. `end` is also the time of that frame's path pose.
- **`kind`:** the code from the table in §5.1.
- **Flushing:** `lio` writes and flushes each frame as soon as it's done, so a reader on a pipe gets whole frames as they finish.
- **Stopping:**
  - If `lio` stops normally, the stream ends after the last whole frame.
  - If `lio` is killed, the last frame may be cut short. Readers should drop a short frame.

A reader in Python with NumPy. It was tested on the real stream, including a stream cut mid-frame:

```python
import struct
import sys

import numpy as np

RAY = np.dtype([("t", "<f8"), ("origin", "<f4", 3), ("direction", "<f4", 3),
                ("range", "<f4"), ("intensity", "<f4")])
KINDS = ["FirstScan", "ImuInit", "MapInit", "Updated", "Skipped"]


def frames(stream):
    """Yields (begin, end, kind, rays) for each whole frame of a LIORAYS1 stream."""
    if stream.read(8) != b"LIORAYS1":
        raise ValueError("not a lio ray stream")
    while len(header := stream.read(21)) == 21:
        count, begin, end, kind = struct.unpack("<IddB", header)
        data = stream.read(count * RAY.itemsize)
        if len(data) < count * RAY.itemsize:
            return  # lio stopped part-way through a frame
        yield begin, end, KINDS[kind], np.frombuffer(data, dtype=RAY)


if __name__ == "__main__":
    for begin, end, kind, rays in frames(sys.stdin.buffer):
        points = rays["origin"] + rays["range"][:, None] * rays["direction"]
        print(f"{kind:9} {end:.3f}: {len(rays)} rays, LiDAR at {rays['origin'][-1]}, "
              f"first point {points[0]}")
```

### 5.3 Other outputs

| Flag | Contents |
|---|---|
| `--path <file>` | The corrected LiDAR pose at each frame's end, in TUM format: `t x y z qx qy qz qw`, world frame. One row per `MapInit` and `Updated` frame. Buffered, so rows arrive in batches; for live poses, use the rays' origins |
| `--frames-csv <file>` | One row per frame: `frame,kind,beg,end,downsampled,effective,map_points,x,y,z,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,proc_ms`. The pose is the LiDAR's at the frame's end. `v` is the IMU's velocity in m/s, `bg` the gyro bias in rad/s and `ba` the accelerometer bias. `effective` is how many points matched the map. `proc_ms` is the processing time |
| `--map <file.pcd>` | FAST-LIO's map at exit, as an ASCII PCD (`x y z intensity`). Written once, at the end |
| stderr | Progress and the end-of-run summary (§7). Nothing is written to stdout unless a flag points an output there |

## 6. Ways to integrate

### 6.1 As a separate process

This is the recommended way.
- The process stays isolated, and the licences stay separate (§2).
- You can pin CPUs with flags.
- The other program only needs the stream format.

**Build:** `cargo build --release` gives `target/release/lio`. A container image is in §6.3.

**Rays to the reader's stdin.** Everything `lio` prints goes to stderr, so stdout carries only the rays:

```bash
lio live --max-seconds 600 --bind 0.0.0.0:2368 --angles unit-angles.csv --firetimes JT128-firetimes.csv --rays /dev/stdout | consumer
```

**Rays through a named pipe:**

```bash
mkfifo /run/lio/rays
```

```bash
lio live --max-seconds 600 --angles unit-angles.csv --firetimes JT128-firetimes.csv --rays /run/lio/rays --path /run/lio/path.tum
```

`lio` blocks at start until a reader opens the pipe.

**Lifecycle, as the controlling program sees it:**
1. Wake the sensor, and make sure the angle file is saved.
2. Start `lio live` with `--max-seconds`. It only listens: it waits for packets and needs nothing from the sensor.
3. Hold the sensor still for the first second of packets (§3.1).
4. Read the rays until `lio` exits:
   - it exits on its own at `--max-seconds`, after writing the summary and any `--path`, `--frames-csv` or `--map` files;
   - to stop it early, close the reading end of the pipe. Its next write fails, and it exits with status 1 ("Broken pipe").
5. Put the sensor back in standby.

**Exit status:**
- 0 when the run finished;
- 1 for a runtime error, such as a bad file, the socket already in use, or a broken pipe;
- 2 for bad arguments (the usage text is printed).

**Back-pressure.** `lio`'s stages are joined by bounded queues, so a slow reader slows `lio` down; nothing is dropped inside `lio`.
- If the reader falls behind by more than a few frames, `lio` stops reading its socket.
- Packets are then lost in the kernel and appear as sequence gaps.
- So the reader must keep up with about 44 MB/s, or use `--ray-stride`.

### 6.2 Embedded as a Rust library

**Cargo.** The crates are not on crates.io. Use git or path dependencies, pinned to a commit:

```toml
[dependencies]
lio = { git = "https://github.com/alexipeck/fast-lio-rs", rev = "<commit>" }
jt128 = { git = "https://github.com/alexipeck/fast-lio-rs", rev = "<commit>" }
lio-core = { git = "https://github.com/alexipeck/fast-lio-rs", rev = "<commit>" }
```

- **A private repository:** if it is still private, Cargo needs git credentials. `net.git-fetch-with-cli = true` in `.cargo/config.toml` makes it use the system's `git` and its credentials.
- **Toolchain:** Rust 1.85 or newer, edition 2024.
- **Build in release mode:** the debug build is far too slow for real time.

**On the caller's thread.** `lio::Pipeline` decodes, runs FAST-LIO and computes the rays. Feed it every UDP payload in arrival order. [`crates/lio/examples/embed.rs`](../crates/lio/examples/embed.rs) is a complete, compiled example on a recording. Live, it looks like this:

```rust
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use jt128::packet::PacketError;
use jt128::{AngleCorrection, DecoderConfig, Firetimes};
use lio::{FrameResult, Pipeline};

fn run(limit: Duration, consume: impl Fn(FrameResult)) -> Result<(), Box<dyn std::error::Error>> {
    let angles = AngleCorrection::parse(&std::fs::read("unit-angles.csv")?)?;
    if angles.hash_matches == Some(false) {
        eprintln!("warning: the angle file's SHA-256 does not match");
    }
    let firetimes = Firetimes::parse(&std::fs::read("JT128-firetimes.csv")?)?;
    let mut pipeline = Pipeline::new(&angles, Some(&firetimes), DecoderConfig::default(), lio::jt128_config());

    let socket = UdpSocket::bind("0.0.0.0:2368")?; // listen only; never send to the sensor
    socket.set_read_timeout(Some(Duration::from_millis(200)))?;
    let (mut buf, mut done, start) = (vec![0u8; 65_536], Vec::new(), Instant::now());
    while start.elapsed() < limit {
        let n = match socket.recv(&mut buf) {
            Ok(n) => n,
            Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => continue,
            Err(e) => return Err(e.into()),
        };
        match pipeline.push_packet(&buf[..n], &mut done) {
            Ok(()) | Err(PacketError::NotPointCloud) => {} // fault messages share the port
            Err(e) => return Err(e.into()),
        }
        done.drain(..).for_each(&consume);
    }
    pipeline.finish(&mut done);
    done.into_iter().for_each(consume);
    Ok(())
}
```

**Spreading the work over threads.** One thread does all the work in the example above, so a frame's processing delays the next packets. To do as `lio` does, run three stages on their own threads, joined by bounded channels:
1. a `jt128::Decoder`, which turns payloads into `Event`s;
2. a `lio::Estimator`, whose `process` turns events into `Processed` frames;
3. `Processed::rays`, which turns those into `FrameResult`.

Each stage must see its items in order. The results are then bit-identical to `Pipeline`. `crates/lio/src/main.rs` (`decode`, `estimate`, `write_outputs`) shows this.

**What you get:**

| Item | Meaning |
|---|---|
| `FrameResult { output, rays }` | One finished frame |
| `rays: Vec<lio_core::rays::Ray>` | One ray per returned point, with the fields from §5.1 |
| `output.kind` | `FrameKind`: `FirstScan`, `ImuInit`, `MapInit`, `Updated` or `Skipped` |
| `output.lidar_beg_time`, `output.lidar_end_time` | The frame's time span, in seconds on the sensor clock |
| `output.state` | The IMU state at the frame's end (`pos`, `rot`, `vel`, `bg`, `ba`, `grav`, extrinsic), corrected when `kind` is `Updated` |
| `lio::lidar_pose(&output)` | The LiDAR position and quaternion (w, x, y, z) at the frame's end, as in `--path` |
| `output.downsampled_points`, `output.effective_features`, `output.map_points` | Health figures (§7) |
| `lio::jt128_config()` | The settings `lio` uses for the JT128, as a `LioConfig` |
| `lio::RayWriter` | Writes the §5.2 stream to any `Write` |
| `pipeline.decoder_stats()` | Packets, spin-up packets, sequence gaps and IMU samples so far |

**Threads and your process.** The parallel parts (matching, motion compensation and rays) use rayon's global pool. Configure that pool before the first frame if the host process needs it confined:
- `rayon::ThreadPoolBuilder::new().num_threads(n).build_global()`;
- or set `RAYON_NUM_THREADS`.

### 6.3 In a container

The `Dockerfile` builds a small image (about 50 MB, distroless) for the host's architecture. Mount the calibration files and an output directory; recordings are never built in.

```bash
docker build -t fast-lio-rs .
```

```bash
docker run --rm --network host -v /etc/lio:/cal:ro -v /run/lio:/out fast-lio-rs live --max-seconds 600 --angles /cal/unit-angles.csv --firetimes /cal/JT128-firetimes.csv --rays /out/rays
```

- `--network host` lets `lio` receive the sensor's UDP stream.
- To build in device defaults, such as CPU placement: `docker build --build-arg LIO_DEFAULT_ARGS="--cpus 4-7" ...`

## 7. Settings, CPUs and health

### Settings

`lio` with no arguments prints every flag. The ones that matter for integration:

| Flag | Default | Notes |
|---|---|---|
| `--bind <addr:port>` | `0.0.0.0:2368` | Live only |
| `--max-seconds <s>` | none | Required for live |
| `--rays`, `--ray-stride`, `--path`, `--frames-csv`, `--map` | off | §5 |
| `--blind <m>` | 1.0 | FAST-LIO ignores closer returns. 1.0 keeps a handheld operator out of the map. On a vehicle or mast with nothing near the sensor, 0.5 also works |
| `--filter-size-surf`, `--filter-size-map` | 0.25 m | Voxel sizes. 0.5 m doubles walls indoors |
| `--extrinsic-r`, `--extrinsic-t` | identity, 0 | The LiDAR's rotation (row-major) and position in the IMU frame |
| `--extrinsic-est` | off | Estimates the extrinsic online and prints the result at the end. Use it to check a new mounting, such as an external IMU |
| `--threads`, `--cpus`, `--pin` | fast cores | Below |

### CPUs

`lio` runs three stage threads plus a worker pool:
- `lio-decode`;
- `lio-fastlio`;
- the output stage, on `lio`'s main thread;
- workers named `lio-worker-N`.

| Flag | Effect |
|---|---|
| `--cpus 4-7` | Confines the whole process, as `taskset` does |
| `--pin workers=5-7`, `--pin lio=4` | Confines one stage. The stages are `decode`, `lio`, `output` and `workers` |
| `--threads n` | Sets the number of workers. By default these are the fast cores where Linux reports core capacities (big.LITTLE), otherwise one per CPU |

- Pinning needs Linux.
- **Efficiency:** about 4 workers give most of the speed. On an M4, real-time operation costs about 11% of one core with 4 workers, against 19% with 10, for little gain.
- **Two sensors:** run one `lio` per sensor, each with its own `--bind` port and its own `--cpus`.
  - Their world frames are unrelated.
  - Merging them needs the sensors' relative mounting and synchronised clocks. Neither is defined yet.

### Health

At exit, `lio` prints a summary to stderr. Taken from the first capture:

```text
decoded 466361 packets (66 spin-up, 0 sequence gaps), 20795 IMU samples (11 recovered)
frames: 1037 (first 1, IMU init 1, map init 1, updated 1034, skipped 0); rays: 114616923
processing: 2.98 s for 103.93 s of data (34.9x real time); map holds 5465 points
  busy: decoding 1.24 s, FAST-LIO 2.92 s, rays 1.11 s and writing 1.39 s, on three threads with 10 workers
path: 9.51 m travelled, furthest 1.76 m from start, ends at [-0.269, -0.071, 0.536]
```

A live run reports headroom instead of speed: how many times faster than real time its busiest stage was. Above about 2× is comfortable.

| Sign | Meaning | What to do |
|---|---|---|
| A `warning:` about the angle file | A damaged file, or Hesai's sample instead of the unit's own | Fetch the unit's file again (§3.2) |
| `sequence gaps` above 0 | Packets lost before `lio` read them | Use a larger socket buffer, more CPU, or a faster reader (§6.1) |
| `effective / downsampled` falling below about 0.3, or `Skipped` frames | FAST-LIO is losing track. Normal is 0.7–0.8 with the default settings indoors | Check the calibration files, that the sensor was still at start, and that no clock step happened |
| The path running away: metres per second with no real motion | Divergence. The rays are wrong from then on | Restart `lio`. On the first capture, a wrong IMU rotation diverged by kilometres within the run |

## 8. Where things are

| Need | Look at |
|---|---|
| The command-line program | `crates/lio/src/main.rs` |
| The library API and the ray stream writer | `crates/lio/src/lib.rs` |
| How each ray is computed | `crates/lio-core/src/rays.rs` |
| The packet format, calibration files and axes | [docs/hesai_jt128_decoder.md](hesai_jt128_decoder.md) |
| Where the port differs from FAST-LIO's C++, and why | [DEVIATIONS.md](../DEVIATIONS.md) |
| Decisions and open questions | [docs/plan.md](plan.md) |
| Speed measurements | [README](../README.md#threads-and-cpus), [fixtures/README.md](../fixtures/README.md#speed-against-the-c) |
