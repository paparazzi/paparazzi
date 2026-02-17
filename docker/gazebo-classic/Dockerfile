FROM ubuntu:22.04
LABEL maintainer="paparazzi-gazebo-classic-sim"

# ── Init ─────────────────────────────────────────────────────────────
ENV TINI_VERSION v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini

# ── Locale & timezone ────────────────────────────────────────────────
ENV LANG C.UTF-8
ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
RUN ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
    && apt-get update \
    && apt-get install -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# ── Repositories ─────────────────────────────────────────────────────
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        software-properties-common curl lsb-release gnupg git ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    # Paparazzi PPA
    && add-apt-repository -y ppa:paparazzi-uav/ppa \
    # kisak-mesa: Mesa 24.x backport — required for Intel Arc (Meteor Lake) iris driver
    && add-apt-repository -y ppa:kisak/kisak-mesa \
    # OSRF: Gazebo Classic 11
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
         http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list

# ── All packages in one layer ─────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Paparazzi build & GCS
    paparazzi-dev \
    pprzgcs \
    # Build tools (simulation only — ARM cross-compiler not needed)
    build-essential \
    cmake \
    python-is-python3 \
    python3-future \
    python3-lxml \
    python3-numpy \
    python3-opencv \
    # Gazebo Classic 11
    gazebo \
    libgazebo-dev \
    # Mesa 24.x for Intel Arc GPU (iris driver)
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    # X11 / GUI
    x11-apps \
    dbus-x11 \
    libcanberra-gtk3-module \
    # Joystick
    joystick \
    libsdl2-dev \
    # Audio (PulseAudio client only — server runs on host)
    libpulse0 \
    pulseaudio-utils \
    # Video (RTP viewer)
    vlc \
    # Speech synthesis (PprzGCS uses --speech for command confirmations)
    speech-dispatcher \
    espeak-ng \
    # Permission helper
    gosu \
    vim \
    && rm -rf /var/lib/apt/lists/*

# ── Runtime environment ───────────────────────────────────────────────
ENV PULSE_SERVER=/run/pulse/native

# ── User setup ───────────────────────────────────────────────────────
ENV USERNAME=pprz
ENV USER_ID=1000
RUN groupadd -f input \
    && useradd --shell /bin/bash -u $USER_ID -o -c "Paparazzi Docker user" -m $USERNAME \
    && usermod -aG sudo,dialout,plugdev,input $USERNAME

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/tini", "--", "/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
