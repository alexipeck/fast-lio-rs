# Builds C++ harnesses against FAST-LIO's own headers to produce golden data
# for the Rust tests. FAST-LIO is cloned at a pinned commit, never vendored.
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates \
      libeigen3-dev libboost-dev libpcl-dev \
 && rm -rf /var/lib/apt/lists/*

ARG FAST_LIO_COMMIT=7cc4175de6f8ba2edf34bab02a42195b141027e9
RUN git clone https://github.com/hku-mars/FAST_LIO.git /src/FAST_LIO \
 && git -C /src/FAST_LIO checkout --quiet "$FAST_LIO_COMMIT" \
 && git -C /src/FAST_LIO submodule update --init --recursive

COPY CMakeLists.txt /fixtures/
COPY src /fixtures/src
RUN cmake -S /fixtures -B /build -DFAST_LIO_DIR=/src/FAST_LIO -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /build -j"$(nproc)"

CMD ["/build/smoke"]
