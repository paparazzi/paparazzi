FROM ubuntu:24.04
LABEL maintainer="felix.ruess@gmail.com"

# Add Tini
ENV TINI_VERSION=v0.8.4
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini

# setup environment
ENV LANG=C.UTF-8

ARG DEBIAN_FRONTEND=noninteractive

# time zone without question
ENV TZ=Etc/UTC
RUN ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# The installer of paparazzi (install.sh, sw/tools/install): the same recipe as on a computer.
# Kept in /opt/pprz-install for the images based on this one (ci).
COPY install.sh /opt/pprz-install/install.sh
COPY sw/tools/install /opt/pprz-install/sw/tools/install

RUN apt-get update && apt-get install -y tzdata \
    && /opt/pprz-install/install.sh --yes --python=system core arm jsbsim \
    && apt-get install -y \
    libcanberra-gtk-module \
    x11-apps \
    gedit \
    python3-future \
    python3-zombie-imp \
    curl \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/tini", "--"]
CMD ["bash"]
