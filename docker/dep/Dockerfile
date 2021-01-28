FROM ubuntu:18.04
LABEL maintainer="felix.ruess@gmail.com"

# Add Tini
ENV TINI_VERSION v0.8.4
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini

# setup environment
ENV LANG C.UTF-8

# add Paparazzi PPA
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:paparazzi-uav/ppa
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa

# setup tzdata
ENV TZ=Etc/UTC
RUN ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install tzdata

# install paparazzi-dev which pull in the dependencies
# also install cross compiler and some stuff for X
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update && apt-get install -y \
    cmake \
    gcc-arm-embedded \
    libcanberra-gtk-module \
    paparazzi-dev \
    paparazzi-jsbsim \
    x11-apps \
    gedit \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/tini", "--"]
CMD ["bash"]
