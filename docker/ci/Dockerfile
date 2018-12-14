FROM paparazziuav/pprz-dep
LABEL maintainer="felix.ruess@gmail.com"

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN curl https://sh.rustup.rs -sSf | sh
RUN rustup install nightly
RUN rustup default nightly
RUN rustup target add thumbv7em-none-eabihf

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    g++-arm-linux-gnueabi \
    libgazebo8-dev \
    rustc cargo \
    && rm -rf /var/lib/apt/lists/*
