#!/usr/bin/env bash
set -euo pipefail

# 1) Install XQuartz from https://www.xquartz.org/ (once)
# 2) In XQuartz Preferences > Security, check:
#    [x] Allow connections from network clients
# 3) Start XQuartz, then run this script.

# Open XQuartz if not running
open -ga XQuartz

# Give it a moment
sleep 2

# Allow local connections
xhost + 127.0.0.1 >/dev/null 2>&1 || true
xhost + localhost >/dev/null 2>&1 || true

# Docker Desktop forwards host.docker.internal to the host
export DISPLAY=host.docker.internal:0

docker compose up -d
echo "Entering rover-sim container..."
docker exec -it rover-sim /entrypoint.sh bash -i
