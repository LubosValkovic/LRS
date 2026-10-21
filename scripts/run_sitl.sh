#!/bin/bash
# Launch ArduPilot SITL against the gz sim world started by run_gazebo.sh.
#
# --model JSON is required (not just -f gazebo-iris) so SITL speaks the
# JSON/FDM protocol the ardupilot_gazebo plugin expects; --add-param-file
# is required because current sim_vehicle.py no longer auto-loads a frame's
# default param file once --model is overridden away from the frame name
# (without it FRAME_CLASS stays 0/undefined and the vehicle won't arm).
#
# Env vars (override if your ArduPilot checkout/location differs):
#   ARDUPILOT_DIR   default: $HOME/ardupilot
#   HOME_LOCATION   default: FEI hangar location, "lat,lon,alt,heading"
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
HOME_LOCATION="${HOME_LOCATION:-48.15084570555732,17.072729745416016,150,0}"

if [ ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
  echo "sim_vehicle.py not found under ARDUPILOT_DIR=$ARDUPILOT_DIR" >&2
  echo "Set ARDUPILOT_DIR to your ArduPilot checkout, or clone one:" >&2
  echo "  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git" >&2
  exit 1
fi

mkdir -p "$ARDUPILOT_DIR/ArduCopter"
cd "$ARDUPILOT_DIR/ArduCopter"

exec python3 "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file="$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm" \
  --console -l "$HOME_LOCATION"
