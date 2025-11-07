#!/usr/bin/env bash
set -euo pipefail
# Headless: just start the container and attach
docker compose up -d
docker exec -it rover-sim bash -lc "echo 'Headless mode: use gz sim -r ... or run ros2 launch files.' && bash"