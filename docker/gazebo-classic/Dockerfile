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
    && apt-get install -y --no-install-recommends software-properties-common curl gnupg git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ── Paparazzi with the installer (install.sh, sw/tools/install): the same recipe as on a computer ──
# Built with the paparazzi folder as context:
#   docker build -t paparazziuav/pprz-gazebo-classic -f docker/gazebo-classic/Dockerfile .
COPY install.sh /opt/pprz-install/install.sh
COPY sw/tools/install /opt/pprz-install/sw/tools/install

# core, python, Bebop compiler, Gazebo Classic 11, PprzGCS, video tools (VLC, OpenCV)
RUN /opt/pprz-install/install.sh --yes --no-recommends --python=system core python bebop gazebo_classic gcs video \
    && rm -rf /var/lib/apt/lists/*

# ── What the installer doesn't install ───────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    # the Qt6 platform plugins (xcb) of PprzGCS: only recommended by libqt6gui6, so not installed here
    qt6-qpa-plugins \
    # GStreamer for the video programs (sw/tools/video/video_to_gcs.sh); gstreamer1.0-x: the X window sinks
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-x \
    # Mesa of Ubuntu 22.04 (23.2) for the GPU (the kisak-mesa PPA has no 22.04 packages any more)
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

COPY docker/gazebo-classic/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/tini", "--", "/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
