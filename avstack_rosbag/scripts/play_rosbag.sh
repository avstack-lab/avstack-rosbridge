#!/usr/bin/env zsh

BAGNAME=${1:-baseline}

set -e

BAGPATH="/data/shared/CARLA/rosbags/${BAGNAME}"

ros2 bag play \
    --loop \
    --delay=1 \
    --rate=0.1 \
    --clock=100 \
    --qos-profile-overrides-path qos_override.yaml \
    $BAGPATH