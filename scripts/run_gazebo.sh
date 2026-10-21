#!/bin/bash
# Launch gz sim with this repo's world + models.
#
# Usage:
#   scripts/run_gazebo.sh                       # fei_lrs_gazebo.world
#   scripts/run_gazebo.sh fei_lrs_gazebo_depth.world
#
# If your machine has both Gazebo Jetty (system) and Harmonic (e.g. from
# `ros-jazzy-ros-gz`) installed, make sure the *matching* build of the
# ardupilot_gazebo plugin is on GZ_SIM_SYSTEM_PLUGIN_PATH for whichever
# Gazebo this resolves to -- see the repo README's "A note on Gazebo
# versions" section.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-fei_lrs_gazebo.world}"

if [ ! -f "$REPO_ROOT/worlds/$WORLD" ]; then
  echo "No such world: $REPO_ROOT/worlds/$WORLD" >&2
  echo "Available worlds:" >&2
  ls "$REPO_ROOT"/worlds/*.world >&2
  exit 1
fi

export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "gz sim version: $(gz sim --version | head -1)"
echo "World: $REPO_ROOT/worlds/$WORLD"
exec gz sim -r -v 4 "$REPO_ROOT/worlds/$WORLD"
