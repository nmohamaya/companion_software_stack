# ============================================================================
# Dockerfile — Drone Companion Software Stack
# ============================================================================
# Multi-stage NVIDIA CUDA build for the autonomous drone companion stack.
#
# Build:
#   docker build -t companion-stack:latest -f docker/Dockerfile .
#
# Run (simulated backends, no GPU required):
#   docker run --ipc=host companion-stack:latest
#
# Run (with GPU for CUDA-accelerated inference):
#   docker run --gpus all --ipc=host companion-stack:latest
# ============================================================================

# ── Builder stage ───────────────────────────────────────────────────────────
FROM nvidia/cuda:12.6.3-devel-ubuntu24.04 AS builder

ARG MODEL_PRESET=edge
ARG DEBIAN_FRONTEND=noninteractive

LABEL stage="builder"

# Core build dependencies
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        wget \
        curl \
        libspdlog-dev \
        libeigen3-dev \
        nlohmann-json3-dev \
        libgtest-dev \
        libfmt-dev \
        libgtk-3-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# ── Zenoh 1.7.2 ────────────────────────────────────────────────────────────
ARG ZENOH_VERSION=1.7.2
ARG TARGETARCH
RUN wget -q "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/libzenohc_${ZENOH_VERSION}-1_${TARGETARCH}.deb" \
         -O /tmp/libzenohc.deb && \
    wget -q "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/libzenohc-dev_${ZENOH_VERSION}-1_${TARGETARCH}.deb" \
         -O /tmp/libzenohc-dev.deb && \
    dpkg -i /tmp/libzenohc.deb /tmp/libzenohc-dev.deb && \
    rm /tmp/libzenohc*.deb && \
    ldconfig

RUN git clone --depth 1 --branch ${ZENOH_VERSION} https://github.com/eclipse-zenoh/zenoh-cpp.git /tmp/zenoh-cpp && \
    cd /tmp/zenoh-cpp && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DZENOHCXX_ZENOHC=OFF && \
    make install && rm -rf /tmp/zenoh-cpp

# ── OpenCV 4.10.0 with CUDA DNN support ────────────────────────────────────
ARG OPENCV_VERSION=4.10.0
RUN git clone --depth 1 --branch ${OPENCV_VERSION} \
        https://github.com/opencv/opencv.git /tmp/opencv && \
    git clone --depth 1 --branch ${OPENCV_VERSION} \
        https://github.com/opencv/opencv_contrib.git /tmp/opencv_contrib && \
    mkdir -p /tmp/opencv/build && cd /tmp/opencv/build && \
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
        -DWITH_CUDA=ON \
        -DWITH_CUDNN=ON \
        -DOPENCV_DNN_CUDA=ON \
        -DBUILD_LIST=core,imgproc,dnn,imgcodecs,highgui \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_python3=OFF \
        -DWITH_PROTOBUF=ON \
        -DBUILD_PROTOBUF=ON \
        -DOPENCV_DNN_OPENCL=OFF \
        .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/opencv /tmp/opencv_contrib

# ── Copy project source and build ──────────────────────────────────────────
WORKDIR /app
COPY . /app/

RUN mkdir -p build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DALLOW_INSECURE_ZENOH=ON \
        -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" && \
    make -j$(nproc)

# Validate build with tests
RUN cd build && ctest --output-on-failure -j$(nproc)

# ── Runtime stage ───────────────────────────────────────────────────────────
FROM nvidia/cuda:12.6.3-runtime-ubuntu24.04 AS runtime

ARG MODEL_PRESET=edge
ARG DEBIAN_FRONTEND=noninteractive

LABEL maintainer="Drone Companion Stack Team"
LABEL description="Autonomous drone companion computer software stack (7 processes, Zenoh IPC)"
LABEL model_preset="${MODEL_PRESET}"

# Runtime-only packages
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        libspdlog1.12 \
        libfmt9 \
        libgtk-3-0t64 \
        libavcodec60 \
        libavformat60 \
        libswscale7 \
        libtbb12 \
        libjpeg8 \
        libpng16-16 \
        libtiff6 && \
    rm -rf /var/lib/apt/lists/*

# Zenoh runtime library (not -dev) — merged into single apt layer
ARG ZENOH_VERSION=1.7.2
ARG TARGETARCH
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends wget ca-certificates && \
    wget -q "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/libzenohc_${ZENOH_VERSION}-1_${TARGETARCH}.deb" \
         -O /tmp/libzenohc.deb && \
    dpkg -i /tmp/libzenohc.deb && \
    rm /tmp/libzenohc.deb && \
    apt-get purge -y --auto-remove wget ca-certificates && \
    rm -rf /var/lib/apt/lists/* && \
    ldconfig

# Copy OpenCV shared libraries from builder
COPY --from=builder /usr/local/lib/libopencv* /usr/local/lib/

# Copy OpenCV pkgconfig and cmake files for runtime library resolution
COPY --from=builder /usr/local/lib/cmake/opencv4/ /usr/local/lib/cmake/opencv4/

# Copy companion stack binaries — launch_all.sh expects them at $PROJECT_DIR/build/bin/
COPY --from=builder /app/build/bin/ /app/build/bin/

# Copy runtime configuration
COPY --from=builder /app/config/default.json /app/config/default.json

# Copy launch script
COPY --from=builder /app/deploy/launch_all.sh /app/deploy/launch_all.sh
RUN chmod +x /app/deploy/launch_all.sh

# Ensure shared libraries are discoverable
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
RUN ldconfig

WORKDIR /app

# Create log directory and non-root user for runtime security
RUN mkdir -p /app/drone_logs && \
    useradd -r -u 1001 -m drone && \
    chown -R drone:drone /app

ENV DRONE_LOG_DIR=/app/drone_logs

USER drone

ENTRYPOINT ["/app/deploy/launch_all.sh"]
