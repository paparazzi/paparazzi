FROM paparazziuav/pprz-dep
LABEL maintainer="felix.ruess@gmail.com"

RUN curl https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain nightly
ENV PATH $HOME/.cargo/bin:$PATH
RUN /bin/bash -c "source $HOME/.cargo/env; \
    rustup target add thumbv7em-none-eabihf; \
    rustup target add x86_64-unknown-linux-gnu"

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    g++-arm-linux-gnueabi \
    libgazebo9-dev \
    rustc cargo \
    python3-lxml \
    && rm -rf /var/lib/apt/lists/*
