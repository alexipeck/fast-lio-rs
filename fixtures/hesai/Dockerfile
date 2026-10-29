# Hesai LiDAR SDK 2.0 at a pinned commit, plus a small tool that dumps its
# decoding of a JT128 pcap. Reference data for the jt128 crate's tests.
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends build-essential cmake git ca-certificates libboost-dev \
 && rm -rf /var/lib/apt/lists/*

ARG SDK_COMMIT=357ff14c0b4486d21ac0de0b218e34546276cacb
RUN git clone https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git /sdk \
 && git -C /sdk checkout --quiet "$SDK_COMMIT"

COPY jt128_dump.cc /sdk/test/jt128_dump.cc
RUN printf '\nadd_executable(jt128_dump test/jt128_dump.cc)\ntarget_link_libraries(jt128_dump PRIVATE hesai_sdk_lib)\n' >> /sdk/CMakeLists.txt \
 && cmake -S /sdk -B /build -DCMAKE_BUILD_TYPE=Release -DWITH_PTCS_USE=OFF -DCMAKE_CXX_FLAGS=-ffp-contract=off \
 && cmake --build /build -j"$(nproc)" --target jt128_dump

ENTRYPOINT ["/build/jt128_dump"]
