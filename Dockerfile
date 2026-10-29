# The `lio` binary in a slim image, built for the host's architecture
# (linux/arm64 on an ARM board). Recordings and calibration files are mounted
# at run time; none are built in.
#
#   docker build -t fast-lio-rs .
#   docker run --rm --network host -v "$PWD/data:/data:ro" -v "$PWD/out:/out" \
#     fast-lio-rs live --max-seconds 60 \
#     --angles /data/angle_correction.csv --firetimes /data/firetimes.csv \
#     --rays /out/rays.bin --path /out/path.tum
FROM debian:bookworm-slim AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl gcc libc6-dev \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY rust-toolchain.toml ./
# rustup installs the toolchain pinned in rust-toolchain.toml.
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain none \
 && /root/.cargo/bin/rustup show active-toolchain
ENV PATH=/root/.cargo/bin:$PATH
COPY Cargo.toml Cargo.lock ./
COPY crates crates
RUN cargo build --release --locked -p lio

FROM debian:bookworm-slim
COPY --from=build /src/target/release/lio /usr/local/bin/lio
ENTRYPOINT ["/usr/local/bin/lio"]
