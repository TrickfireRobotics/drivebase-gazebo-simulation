#!/bin/bash

gz service -s /world/empty/create \
    --reptype gz.msgs.Boolean \
    --reqtype gz.msgs.EntityFactory \
    --req 'sdf_filename: "/ws/ros2_ws/src/rover_description/models/tfr_bot.sdf", name: "tfr_bot", pose: { position: {x: 0, y: 0, z: 0.1} }' \
    --timeout 2000
