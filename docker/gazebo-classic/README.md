# Running Paparazzi + Gazebo Classic Simulation via Docker (Ubuntu 24.04 Host)

Gazebo Classic (gazebo11) is not available on Ubuntu 24.04. This Docker setup
runs the full simulation stack (Paparazzi Center, GCS, NPS simulator, Gazebo
Classic) inside an Ubuntu 22.04 container with GUI forwarding to your host.

---

## Prerequisites

### 1. Install Docker

```bash
# Install Docker Engine (not Docker Desktop)
sudo apt-get update
sudo apt-get install -y docker.io

# Add your user to the docker group (avoids needing sudo)
sudo usermod -aG docker $USER

# Log out and log back in, then verify:
docker run hello-world
```

### 2. GPU Drivers (for Gazebo 3D rendering)

**Intel/AMD (Mesa) — usually works out of the box:**
```bash
# Verify DRI is available
ls /dev/dri/
# Should show: card0  renderD128
# If you card names are different, update them accordingly in run_gazebo_sim.sh
```

**NVIDIA — requires nvidia-container-toolkit:**
```bash
# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## Quick Start

### Step 1: Build the Docker Image

```bash
cd paparazzi/docker/gazebo-classic
docker build -t paparazziuav/pprz-gazebo-classic .
```

This takes ~10-15 minutes the first time (downloads Ubuntu 22.04 + all
dependencies + Gazebo Classic 11).

### Step 2: Initialize Git Submodules

Make sure the TU Delft Gazebo models are present (needed for CyberZoo worlds):

```bash
cd paparazzi/
git submodule init
git submodule sync
git submodule update sw/ext/tudelft_gazebo_models
```

### Step 3: Launch the Container

```bash
cd paparazzi/docker/gazebo-classic

# Option A: Interactive shell (recommended for first run)
./run_gazebo_sim.sh

# Option B: Launch Paparazzi Center directly
./run_gazebo_sim.sh ./paparazzi
```

### Step 4: Build and Run Simulation (inside the container)

```bash
# If you used Option A (interactive shell):

# First time only — build Paparazzi
make clean
make

# Launch Paparazzi Center
./paparazzi
```

Then in the Paparazzi Center GUI:
1. Select the aircraft: **bebop_orange_avoid** (or your target aircraft)
2. Select the target: **nps** (in the "Target" dropdown)
3. Click **Build** to compile the NPS simulator with Gazebo Classic
4. Select session: **Simulation**
5. Click **Execute** to start all components

This launches:
- **NPS Simulator** — runs the Gazebo Classic physics + the autopilot
- **Server** — IVY bus message broker
- **GCS (PprzGCS)** — Ground Control Station GUI
- **Data Link** — telemetry bridge

---

## Architecture Overview

```
┌─── Ubuntu 24.04 Host ─────────────────────────────────────┐
│                                                            │
│   X11 Server (Wayland/X)  ◄──── GUI windows forwarded     │
│   PulseAudio Server       ◄──── Audio forwarded            │
│   /dev/input/js*          ◄──── Joystick shared            │
│                                                            │
│  ┌─── Docker Container (Ubuntu 22.04) ──────────────────┐  │
│  │                                                      │  │
│  │   Paparazzi Center                                   │  │
│  │    ├── NPS Simulator (simsitl)                       │  │
│  │    │    └── Gazebo Classic 11 (embedded server)      │  │
│  │    ├── Server (IVY bus)                              │  │
│  │    ├── GCS (PprzGCS)                                 │  │
│  │    └── Data Link                                     │  │
│  │                                                      │  │
│  │   --network=host (shares host network stack)         │  │
│  │   IVY bus broadcasts on 127.255.255.255:2010         │  │
│  │                                                      │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                            │
│   /paparazzi (source)  ←──volume mount──→  container       │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

Key design choices:
- **`--network=host`**: The IVY bus uses UDP broadcast on `127.255.255.255`.
  Host networking lets IVY messages flow freely between container processes
  (and optionally to host-side tools).
- **Volume mount**: Your paparazzi source tree is mounted into the container.
  Build artifacts persist on your host filesystem.
- **UID/GID mapping**: The entrypoint script matches the container user to
  your host UID/GID so file permissions stay correct.

---

## Networking Details

### IVY Bus Communication
All Paparazzi components communicate via the **IVY bus** — a lightweight
publish-subscribe protocol over UDP broadcast.

- Default broadcast address: `127.255.255.255` (Linux)
- Default port: `2010`
- The `--network=host` flag shares the host's network stack with the
  container, so IVY messages are visible to both sides.

**Running tools on the host alongside Docker:**
Because of `--network=host`, you can run IVY-compatible tools on the host
that communicate with the simulation inside Docker. For example, you could
run a custom GCS or logging tool on the host while the simulation runs in
the container.

### If You Cannot Use Host Networking
If host networking is not an option (e.g., on Docker Desktop for Mac/Windows),
you would need to:
1. Run ALL Paparazzi components inside the container (which this setup does)
2. Or configure IVY to use a specific multicast address and expose it via
   Docker port mappings (more complex, not recommended)

---

## Joystick Support

The run script automatically detects and passes joystick devices
(`/dev/input/js*`, `/dev/input/event*`) into the container.

### Verify Joystick Inside Container

```bash
# Inside the container
ls /dev/input/js*          # should list your joystick
jstest /dev/input/js0      # test joystick input

# Use joystick with NPS simulator
# The NPS simulator accepts: --js_dev <index>
# This is passed automatically when using the Paparazzi Center
```

### Disable Joystick Passthrough

```bash
DISABLE_JOYSTICK=1 ./run_gazebo_sim.sh
```

---

## Audio Support

Audio from the container (e.g., Gazebo sounds) is forwarded to your host's
PulseAudio server via a socket mount.

### Verify Audio Inside Container

```bash
# Inside the container
pactl info            # should show connection to host PulseAudio
paplay /usr/share/sounds/freedesktop/stereo/bell.oga   # test sound
```

### Disable Audio

```bash
DISABLE_AUDIO=1 ./run_gazebo_sim.sh
```

---

## GPU Acceleration

Gazebo Classic needs OpenGL for 3D rendering. The run script handles two cases:

### Intel / AMD (default)
The script detects `/dev/dri` and passes it through. No extra setup needed.

```bash
# Verify inside container
glxinfo | grep "OpenGL renderer"
# Should show your GPU, NOT llvmpipe/software
```

### NVIDIA
Set the `GPU_TYPE` environment variable:

```bash
GPU_TYPE=nvidia ./run_gazebo_sim.sh
```

This uses `--gpus all` which requires `nvidia-container-toolkit` (see
Prerequisites above).

---

## Troubleshooting

### "KeyError: 'XDG_SESSION_TYPE'" when running `./paparazzi`
The Paparazzi launcher checks `XDG_SESSION_TYPE` to detect Wayland. Inside
Docker there is no desktop session, so this variable is missing. The run
script sets `XDG_SESSION_TYPE=x11` automatically. If you see this error,
make sure you launched the container via `run_gazebo_sim.sh` (not plain
`docker run`).

### "cannot open display" or blank GUI windows
```bash
# On the host, allow X connections from Docker:
xhost +local:docker

# Or for Wayland (Ubuntu 24.04 default):
# Make sure XWayland is running — it usually is by default.
# If still failing, try switching to an X11 session at login.
```

### Gazebo starts but shows black screen / software rendering
```bash
# Check GPU access inside container
glxinfo | grep "OpenGL renderer"

# If it says "llvmpipe" → GPU passthrough is not working
# For Intel/AMD: make sure /dev/dri exists and is readable
# For NVIDIA: make sure nvidia-container-toolkit is installed
```

### "paparazzi-dev package not found"
The Paparazzi PPA may not have packages for all Ubuntu versions. Since the
Dockerfile uses Ubuntu 22.04, this should work. If it fails, the PPA may
be temporarily down — retry later.

### Build errors about "pkg-config gazebo"
This means Gazebo Classic headers are not found. Inside the container, verify:
```bash
pkg-config --modversion gazebo
# Should print: 11.x.x
```

### IVY bus messages not reaching between processes
```bash
# Verify host networking is active
docker inspect <container_id> | grep NetworkMode
# Should show: "host"

# Test IVY connectivity inside container
ivy-c-logger &    # if available, shows IVY messages
```

### Permission denied on /dev/input or /dev/dri
```bash
# Add your user to the input group on the host
sudo usermod -aG input $USER
# Log out and back in

# Or run the container with --privileged (not recommended for daily use)
```

### Build artifacts from container don't match host architecture
The build happens inside Ubuntu 22.04 (same x86_64 arch), so binaries are
compatible. However, if you also build natively on the host, the two builds
may conflict. Use `make clean` when switching between host and Docker builds.

---

## Environment Variables Reference

| Variable            | Default | Description                                |
|---------------------|---------|--------------------------------------------|
| `GPU_TYPE`          | (auto)  | Set to `nvidia` for NVIDIA GPU support     |
| `DISABLE_USB`       | (unset) | Set to `1` to skip USB device passthrough  |
| `DISABLE_JOYSTICK`  | (unset) | Set to `1` to skip joystick passthrough    |
| `DISABLE_AUDIO`     | (unset) | Set to `1` to skip PulseAudio passthrough  |

---

## File Structure

```
docker/gazebo-classic/
├── Dockerfile           # Ubuntu 22.04 + Gazebo Classic 11 + all deps
├── entrypoint.sh        # UID/GID mapping for volume permissions
├── run_gazebo_sim.sh    # Launch script with X11/GPU/audio/joystick
└── README.md            # This guide
```
