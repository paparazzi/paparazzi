#!/bin/bash
#
# Run Paparazzi with Gazebo Classic simulation inside Docker.
#
# Usage:
#   ./run_gazebo_sim.sh                  # interactive bash shell
#   ./run_gazebo_sim.sh ./paparazzi      # launch Paparazzi Center directly
#
# Environment variables:
#   DISABLE_USB=1       - skip USB device passthrough
#   DISABLE_JOYSTICK=1  - skip joystick passthrough
#   DISABLE_AUDIO=1     - skip PulseAudio passthrough
#   GPU_TYPE=nvidia      - use NVIDIA GPU (requires nvidia-container-toolkit)
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PAPARAZZI_SRC="$(readlink -m "$SCRIPT_DIR/../..")"
IMAGE_NAME="paparazziuav/pprz-gazebo-classic"
PPRZ_HOME_CONTAINER="/home/pprz/paparazzi"

# Default: interactive bash if no arguments
if [ $# -lt 1 ]; then
    CMD="bash"
else
    CMD="$@"
fi

echo "============================================="
echo " Paparazzi + Gazebo Classic (Docker)"
echo "============================================="
echo " Paparazzi source: $PAPARAZZI_SRC"
echo " Command: $CMD"
echo "============================================="

# ── X11 Display Forwarding ───────────────────────────────────────────
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth.$$
touch "$XAUTH"
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null

X_OPTS=(
    --volume="$XSOCK:$XSOCK"
    --volume="$XAUTH:$XAUTH"
    --env="XAUTHORITY=$XAUTH"
    --env="DISPLAY=$DISPLAY"
    --env="XDG_SESSION_TYPE=x11"
    --env="XDG_RUNTIME_DIR=/tmp/runtime-pprz"
    --env="LIBGL_DRI3_DISABLE=1"
)

# ── GPU Acceleration ─────────────────────────────────────────────────
# The render group owns /dev/dri/renderD128 (mode crw-rw----).
# We pass the host's render and video GIDs into the container so the
# container user can open the DRM device for hardware rendering.
GPU_OPTS=()
if [ "$GPU_TYPE" = "nvidia" ]; then
    echo "[GPU] Using NVIDIA runtime"
    GPU_OPTS+=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
elif [ -d /dev/dri ]; then
    echo "[GPU] Using Mesa/Intel/AMD (DRI)"
    GPU_OPTS+=(--device=/dev/dri/card0 --device=/dev/dri/renderD128)
    RENDER_GID=$(getent group render 2>/dev/null | cut -d: -f3)
    VIDEO_GID=$(getent group video  2>/dev/null | cut -d: -f3)
    [ -n "$RENDER_GID" ] && GPU_OPTS+=(--group-add="$RENDER_GID") && echo "[GPU] Added render group (GID $RENDER_GID)"
    [ -n "$VIDEO_GID"  ] && GPU_OPTS+=(--group-add="$VIDEO_GID")  && echo "[GPU] Added video  group (GID $VIDEO_GID)"
fi

# ── Networking (host mode for IVY bus) ───────────────────────────────
# IVY bus uses UDP broadcast on 127.255.255.255 — host networking is
# the simplest way to allow container ↔ host IVY communication.
NET_OPTS=(--network=host)

# ── PulseAudio ───────────────────────────────────────────────────────
AUDIO_OPTS=()
if [ -z "$DISABLE_AUDIO" ]; then
    USER_UID=$(id -u)
    PULSE_SOCK="/run/user/${USER_UID}/pulse"
    if [ -d "$PULSE_SOCK" ]; then
        echo "[Audio] PulseAudio passthrough enabled"
        AUDIO_OPTS+=(--volume="$PULSE_SOCK:/run/pulse")
    else
        echo "[Audio] PulseAudio socket not found, skipping"
    fi
fi

# ── Joystick / Input Devices ────────────────────────────────────────
INPUT_OPTS=()
if [ -z "$DISABLE_JOYSTICK" ]; then
    # Pass /dev/input for joysticks and gamepads
    if [ -d /dev/input ]; then
        echo "[Input] Passing /dev/input for joystick support"
        INPUT_OPTS+=(--volume=/dev/input:/dev/input)
        # Find and pass specific joystick devices
        for js in /dev/input/js*; do
            [ -e "$js" ] && INPUT_OPTS+=(--device="$js")
        done
        for ev in /dev/input/event*; do
            [ -e "$ev" ] && INPUT_OPTS+=(--device="$ev")
        done
    fi
fi

# ── USB Devices (serial adapters, etc.) ──────────────────────────────
USB_OPTS=()
if [ -z "$DISABLE_USB" ]; then
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        if [ -e "$dev" ]; then
            echo "[USB] Passing $dev"
            USB_OPTS+=(--device="$dev")
        fi
    done
fi

# ── Paparazzi Volume Mount ───────────────────────────────────────────
VOL_OPTS=(
    --volume="$PAPARAZZI_SRC:$PPRZ_HOME_CONTAINER"
    --env="PAPARAZZI_HOME=$PPRZ_HOME_CONTAINER"
    --env="PAPARAZZI_SRC=$PPRZ_HOME_CONTAINER"
    -w "$PPRZ_HOME_CONTAINER"
)

# ── Run Container ────────────────────────────────────────────────────
echo ""
echo "Starting container..."
docker run \
    --rm -it \
    --cap-add=SYS_NICE \
    --ulimit rtprio=99 \
    --env="NO_AT_BRIDGE=1" \
    "${X_OPTS[@]}" \
    "${GPU_OPTS[@]}" \
    "${NET_OPTS[@]}" \
    "${AUDIO_OPTS[@]}" \
    "${INPUT_OPTS[@]}" \
    "${USB_OPTS[@]}" \
    "${VOL_OPTS[@]}" \
    -e LOCAL_USER_ID="$(id -u)" \
    -e LOCAL_GROUP_ID="$(id -g)" \
    "$IMAGE_NAME" \
    $CMD

# ── Cleanup ──────────────────────────────────────────────────────────
rm -f "$XAUTH"
