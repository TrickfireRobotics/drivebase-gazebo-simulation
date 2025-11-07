# TrickFire Rover Drivebase Simulation — Cross‑Platform Setup (Docker + ROS 2 Humble + Gazebo Garden)

This repo gives drivebase a **reproducible** environment for simulating the rover drivebase using **ROS 2 Humble**, **Gazebo (Garden)**, and the **ros_gz** bridge — all inside Docker. It runs on **macOS**, **Windows (WSL2)**, and **Linux** with identical commands.

---

## Contents

- [What’s included](#whats-included)
- [Folder layout](#folder-layout)
- [Prerequisites by OS](#prerequisites-by-os)
  - [macOS](#macos)
  - [Windows (WSL2)](#windows-wsl2)
  - [Linux](#linux)
- [First‑time setup](#first-time-setup)
- [Build & run](#build--run)
  - [macOS run (GUI)](#macos-run-gui)
  - [Windows run (GUI via VcXsrv)](#windows-run-gui-via-vcxsrv)
  - [Linux run (GUI)](#linux-run-gui)
  - [Headless run (all OSes)](#headless-run-all-oses)
- [Sanity checks](#sanity-checks)
- [Launching Gazebo & bridging ROS 2](#launching-gazebo--bridging-ros-2)
- [Volumes & persistence](#volumes--persistence)
- [Troubleshooting](#troubleshooting)
- [Appendix: Key config files](#appendix-key-config-files)

---

## What’s included

- **Docker image** based on Ubuntu 22.04 with:
  - ROS 2 Humble (`ros-humble-desktop`)
  - Gazebo **Garden** (`gz-garden`)
  - `ros_gz` bridge packages
  - Developer tools: `colcon`, `vcstool`, `rosdep`
- **Docker Compose** service `rover-sim` pre-wired for cross‑platform GUI
- **ROS 2 workspace** at `/ws/ros2_ws` (mounted from `./ros2_ws`)
- **Scripts** to build and run on each platform (GUI and headless)

---

## Folder layout

```
rover-sim/
├─ docker/
│  ├─ Dockerfile
│  ├─ entrypoint.sh
│  ├─ gazebo.env
├─ docker-compose.yml
├─ scripts/
│  ├─ build.sh
│  ├─ run_linux.sh
│  ├─ run_macos.sh
│  └─ run_headless.sh
└─ ros2_ws/
   └─ src/           # put rover packages here
```

---

## Prerequisites by OS

### macOS
1. **Docker Desktop for Mac**
2. **XQuartz**
   - After install, open **XQuartz → Settings → Security** and check **“Allow connections from network clients.”**
3. (Apple Silicon) You’re covered: the compose file pins `platform: linux/amd64` for maximum compatibility.

### Windows (WSL2)
1. **Docker Desktop for Windows** (with **WSL2 backend** enabled).
2. **VcXsrv** (or another X server) on Windows host.
   - When launching VcXsrv: enable **Disable access control** (for local testing) or add your WSL/Docker IPs explicitly later.

### Linux
1. **Docker Engine** + **Docker Compose plugin**.
2. A working **X11** desktop on the host (for GUI).

---

## First‑time setup

> Do this once per machine.

### macOS
```bash
# Install and start XQuartz
brew install --cask xquartz
defaults write org.xquartz.X11 nolisten_tcp -bool false
open -ga XQuartz

# Allow the container to open windows on your display (repeat per new terminal session)
export DISPLAY=:0
/opt/X11/bin/xhost + 127.0.0.1
/opt/X11/bin/xhost + localhost
```

### Windows (WSL2)
- Start **VcXsrv** on Windows (e.g., `Multi-window` → check **Disable access control** to start).
- In **PowerShell** or **WSL** session where you’ll run Docker commands:
  ```bash
  # Let containers draw to the Windows X server
  export DISPLAY=host.docker.internal:0
  ```

### Linux
```bash
# Allow local containers to use your X server (once per host session)
xhost +local:docker
```

---

## Build & run

From the repo root:

```bash
# 1) Build the image
./scripts/build.sh     # (wraps: docker compose build)

# 2) Start the container (choose one)
./scripts/run_macos.sh    # macOS (GUI)
./scripts/run_linux.sh    # Linux (GUI)
./scripts/run_headless.sh # Headless (all OSes)

# Or run manually if you prefer:
docker compose up -d
docker exec -it rover-sim bash
```

> Inside the container you’ll land at `/ws/ros2_ws` with ROS already sourced by the entrypoint.

### macOS run (GUI)
```bash
# On the host (each new terminal):
open -ga XQuartz
export DISPLAY=:0
/opt/X11/bin/xhost + 127.0.0.1
/opt/X11/bin/xhost + localhost

# Then:
./scripts/run_macos.sh
# In the container:
gz sim -v 3 -r empty.sdf
```

### Windows run (GUI via VcXsrv)
```bash
# On Windows host: start VcXsrv (Disable access control)
# In your terminal (PowerShell/WSL):
export DISPLAY=host.docker.internal:0

docker compose up -d
docker exec -it rover-sim bash
gz sim -v 3 -r empty.sdf
```

### Linux run (GUI)
```bash
# On host
xhost +local:docker

# Run
./scripts/run_linux.sh
# In the container
gz sim -v 3 -r empty.sdf
```

### Headless run (all OSes)
```bash
./scripts/run_headless.sh
# In the container (server-only Gazebo)
gz sim -s -r empty.sdf
```

---

## Sanity checks

In the **container**:
```bash
# ROS & Gazebo present?
ros2 topic list
gz sim --version

# GUI path OK? (for GUI flows)
echo $DISPLAY            # expect: host.docker.internal:0
xclock &                 # simple X test (should open a small clock window)
```

---

## Launching Gazebo & bridging ROS 2

**Start an empty world (GUI):**
```bash
gz sim -v 3 -r empty.sdf
```

**Bridge topics (example `cmd_vel`):**
```bash
# List bridgeable pairs
ros2 run ros_gz_bridge parameter_bridge --print

# Example bridge
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

**Server-only Gazebo (no GUI):**
```bash
gz sim -s -r empty.sdf
```

Drop your packages into `ros2_ws/src/`, then build:
```bash
cd /ws/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Volumes & persistence

Your host folder `./ros2_ws` is mounted into the container at `/ws/ros2_ws`. That means:
- Code you edit on the host is visible in the container.
- Builds (`colcon build`) create `install/` and `build/` under `./ros2_ws` on the host.

---

## Troubleshooting

**“Unable to open display” on macOS**  
- Ensure XQuartz is running and `export DISPLAY=:0` in your host shell.
- Run:
  ```bash
  /opt/X11/bin/xhost + 127.0.0.1
  /opt/X11/bin/xhost + localhost
  ```

**Gazebo GUI crashes with OpenGL errors (Qt can’t create context)**  
- We ship software rendering fallbacks (Mesa llvmpipe) via environment variables in **docker-compose.yml**.
- If needed, add in-container for your session:
  ```bash
  export QSG_RHI_BACKEND=software
  gz sim -v 3 -r empty.sdf
  ```

**Packages not found at build-time (apt)**  
- The Dockerfile enables `universe` and pins correct Gazebo repo (`ubuntu-stable … main`). Rebuild with:
  ```bash
  docker compose build --no-cache
  ```

**Apple Silicon quirkiness**  
- Compose forces `platform: linux/amd64` for maximum compatibility on M‑series Macs.

**Windows: no GUI shows**  
- Confirm VcXsrv is running and `DISPLAY=host.docker.internal:0` is exported before `docker exec`.
- If still stuck, temporarily loosen access control in VcXsrv to verify it’s a permissions issue.

---

## Appendix: Key config files

- **`docker-compose.yml`** — service env vars for cross‑platform GUI, software GL fallbacks, amd64 pin on Apple Silicon. fileciteturn3file1
- **`docker/gazebo.env`** — extra OpenGL / Gazebo env defaults used by Compose. fileciteturn3file0

- **`docker/entrypoint.sh`** — sources ROS 2 and the workspace automatically on container start.
- **`docker/Dockerfile`** — installs ROS 2 Humble, Gazebo Garden, and ros_gz; adds dev tools and sets up the non‑root `dev` user.

---

## TL;DR

```bash
# Build
./scripts/build.sh

# macOS GUI
open -ga XQuartz
export DISPLAY=:0
/opt/X11/bin/xhost + 127.0.0.1
/opt/X11/bin/xhost + localhost
./scripts/run_macos.sh
# then inside: gz sim -v 3 -r empty.sdf

# Headless (any OS)
./scripts/run_headless.sh
# then inside: gz sim -s -r empty.sdf
```