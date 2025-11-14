# Drivebase Simulation — Reproducible Runbook (Start-to-Finish)

This guide is **copy/paste-ready** for a teammate who has not started the container or installed anything yet. It covers Mac (GUI + headless), Windows (WSL2), and Linux, and shows **exact commands for Terminal A/B/C** inside a single container.

> TL;DR: You will build a Docker image, start one container, open 3 shells into it, start Gazebo, spawn the robot, run the ROS↔Gazebo bridge, build & run a tiny ROS 2 node that publishes `/cmd_vel`, and verify.

---

## 0) Prerequisites

### 0.1 Everyone
- Install **Docker Desktop** (macOS/Windows) or **Docker** (Linux). Start it and wait until it’s fully running.

### 0.2 macOS — GUI path (optional)
To see Gazebo windows:
```bash
# Install and start XQuartz (only once)
brew install --cask xquartz
defaults write org.xquartz.X11 nolisten_tcp -bool false
open -ga XQuartz

# Each new terminal session where you want GUI:
export DISPLAY=:0
/opt/X11/bin/xhost + 127.0.0.1
/opt/X11/bin/xhost + localhost
```
If you prefer **headless**, skip XQuartz and the 3 lines above.

### 0.3 Windows (WSL2) notes
- Install Docker Desktop for Windows and **enable WSL2 backend**.
- Use **Ubuntu WSL** terminal for all commands below (not PowerShell).
- GUI via X server is possible but more complex; recommended to use **headless** path.

### 0.4 Linux notes
- If you want GUI, ensure an X server is available (most Linux desktops already have one). For headless, no extra step.

---

## 1) Clone the repo and build the Docker image (host machine)

From the **repo root** (where `docker-compose.yml` lives):
```bash
./scripts/build.sh
# (equivalent to: docker compose build)
```

> If you see permission issues on scripts, run: `chmod +x ./scripts/*.sh`

---

## 2) Start the container (host machine)

Pick **one**:

### 2A) macOS GUI (XQuartz path)
```bash
./scripts/run_macos.sh
# You will land inside the container at: /ws/ros2_ws
```

### 2B) Headless (recommended for all platforms)
```bash
./scripts/run_headless.sh
# You will land inside the container at: /ws/ros2_ws
```

> Alternative (generic):  
> ```bash
> docker compose up -d
> docker exec -it rover-sim bash
> ```

At this point you have **Terminal A** inside the container. Open 2 more **host** terminals and attach them to the **same** container:

```bash
# Terminal B (host):
docker exec -it rover-sim bash

# Terminal C (host):
docker exec -it rover-sim bash
```

You now have **Terminal A/B/C** — all inside the **same** container (`dev@...:/ws/ros2_ws$`).

> Do **not** use `docker attach`; it steals the single TTY. Always use `docker exec -it rover-sim bash` for new shells.

---

## 3) Prepare each container terminal (A, B, C)

In **each** terminal inside the container, source ROS 2 so `ros2` commands are available:

```bash
source /opt/ros/humble/setup.bash
# Optional: make it automatic next time
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

---

## 4) Terminal A — Start Gazebo server

Pick **one**:

### Headless (simplest; works everywhere)
```bash
gz sim -s -r empty.sdf &
sleep 2
```

### GUI (macOS with XQuartz or Linux desktop)
```bash
gz sim -v 3 -r empty.sdf
# If you want your prompt back: press Ctrl+Z, then type: bg
```

> If you see an OpenGL/Qt warning on macOS GUI, you can try:  
> `export QSG_RHI_BACKEND=software` then re-run Gazebo.

---

## 5) Terminal B — Spawn the robot into the world

**Option 1: Inline command (discover the world name, then spawn):**
```bash
# Discover the world create service (copy result like /world/empty/create)
gz service -l | grep -oE '/world/[^/]+/create' | head -n1

# Spawn (use your world name if not 'empty')
gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf_filename: "/ws/ros2_ws/src/rover_description/models/tfr_bot.sdf", name: "tfr_bot", pose: { position: { x: 0, y: 0, z: 0.1 } }'
```

**Option 2: Use the helper script (if present):**
```bash
# Run the spawner script in your repo:
/ws/ros2_ws/../scripts/spawn_tfr_bot.sh
```

**Expected output:** `data: true` (entity created).  
If it times out, the server isn’t running, the world name is different, or the path is wrong.

> **Plugin note (silence a warning):** Ensure your `tfr_bot.sdf` uses the fully qualified plugin class:
> ```xml
> <plugin filename="libgz-sim-diff-drive-system.so" name="gz::sim::systems::DiffDrive">
>   <left_joint>left_wheel_joint</left_joint>
>   <right_joint>right_wheel_joint</right_joint>
>   <wheel_separation>0.30</wheel_separation>
>   <wheel_radius>0.05</wheel_radius>
>   <odom_frame>odom</odom_frame>
>   <robot_base_frame>base_link</robot_base_frame>
>   <topic>/cmd_vel</topic>
> </plugin>
> ```

---

## 6) Terminal C — Start the ROS↔Gazebo bridge and your node

```bash
# 6.1 Start the dynamic bridge (background it so you keep the shell)
ros2 run ros_gz_bridge parameter_bridge &
sleep 1

# 6.2 Build your package and run the node
cd /ws/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select tfr_rover_demo
source install/setup.bash
ros2 run tfr_rover_demo circle_driver
```

Your node publishes `geometry_msgs/Twist` to `/cmd_vel` at ~2 Hz.

---

## 7) Verify (any terminal in the container)

```bash
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```
- GUI users should see the robot drive in a gentle circle.
- Headless users still have full physics; you just won’t see the window locally.

---

## 8) Common errors & fixes

- **`bash: ros2: command not found`**  
  You forgot to source ROS 2 in that terminal. Run:  
  `source /opt/ros/humble/setup.bash`

- **`service call timed out`**  
  Server isn’t running or wrong world name. Start server (`gz sim -s -r empty.sdf &`) and discover the actual path:  
  `gz service -l | grep -oE '/world/[^/]+/create' | head -n1`

- **`SDF file not found`**  
  Confirm the path **inside the container**:  
  `ls -l /ws/ros2_ws/src/rover_description/models/tfr_bot.sdf`

- **`package not found` or `entry point not found`**  
  - Ensure your package has `resource/tfr_rover_demo` (empty file) and `setup.cfg`.
  - Rebuild and re-source:  
    `colcon build --packages-select tfr_rover_demo && source install/setup.bash`

- **GUI on macOS shows OpenGL/Qt errors**  
  - Ensure XQuartz is running and `DISPLAY=:0` is set on the **Mac host**.  
  - Allow clients: `/opt/X11/bin/xhost + 127.0.0.1` and `/opt/X11/bin/xhost + localhost`  
  - Try software backend: `export QSG_RHI_BACKEND=software`

---

## 9) Shutdown / cleanup

From the **host** (not in the container):
```bash
docker compose down
```

To completely remove images/layers (optional):  
```bash
docker system prune -af
```

---

## 10) Reference — Minimal file layout (what should be tracked in git)

```
repo/
├─ .gitignore
├─ docker-compose.yml
├─ docker/
│  └─ Dockerfile
├─ scripts/
│  ├─ build.sh
│  ├─ run_headless.sh
│  ├─ run_macos.sh              # optional for GUI path on macOS
│  └─ spawn_tfr_bot.sh          # optional helper to spawn model
└─ ros2_ws/
   └─ src/
      ├─ rover_description/
      │  └─ models/
      │     └─ tfr_bot.sdf
      └─ tfr_rover_demo/
         ├─ package.xml
         ├─ setup.py
         ├─ setup.cfg
         ├─ resource/
         │  └─ tfr_rover_demo
         └─ tfr_rover_demo/
            ├─ __init__.py
            └─ circle_driver.py
```

Everything else (`ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`, etc.) should be ignored by git.