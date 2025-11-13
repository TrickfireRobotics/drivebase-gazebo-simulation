#!/usr/bin/env bash
set -euo pipefail

MODEL="/ws/ros2_ws/src/rover_description/models/tfr_bot.sdf"

# Find the world create service path (e.g., /world/empty/create)
WORLD_SERVICE="$(gz service -l | grep -oE '/world/[^/]+/create' | head -n1)"
if [[ -z "$WORLD_SERVICE" ]]; then
  echo "No world create service found. Is Gazebo running? (try: gz sim -s -r empty.sdf)" >&2
  exit 1
fi

# Spawn via here-doc (avoids quoting pain)
gz service -s "$WORLD_SERVICE" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 <<'EOF'
sdf_filename: "/ws/ros2_ws/src/rover_description/models/tfr_bot.sdf"
name: "tfr_bot"
pose: { position: { x: 0, y: 0, z: 0.1 } }
EOF

echo "Spawn request sent to $WORLD_SERVICE"