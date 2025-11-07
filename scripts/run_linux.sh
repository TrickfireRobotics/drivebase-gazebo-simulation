#!/usr/bin/env bash
set -euo pipefail
# Allow local docker to talk to X
xhost +local:docker 1>/dev/null 2>&1 || true
# Run container
docker compose up -d
echo "Entering rover-sim container..."
docker exec -it rover-sim bash