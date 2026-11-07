FROM paparazziuav/pprz-dep
LABEL maintainer="felix.ruess@gmail.com"

RUN curl https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain nightly
ENV PATH=$HOME/.cargo/bin:$PATH
RUN /bin/bash -c "source $HOME/.cargo/env; \
    rustup target add thumbv7em-none-eabihf; \
    rustup target add x86_64-unknown-linux-gnu"

# Gazebo (gz-harmonic) and the Bebop compiler with the installer of the pprz-dep image
RUN /opt/pprz-install/install.sh --yes --python=system bebop gazebo \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    rustc cargo \
    python3-lxml \
    python3-empy \
    python3-pexpect \
    build-essential cmake extra-cmake-modules qt6-base-dev qt6-svg-dev qt6-charts-dev libxml2-dev libzip-dev \
    && rm -rf /var/lib/apt/lists/*
