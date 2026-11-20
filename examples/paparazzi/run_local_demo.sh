#!/usr/bin/env bash
# Copyright 2026 OpenUAS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$SCRIPT_DIR"

selected_camera=${CAMERA:-}
if [[ "${1:-}" == --camera ]]; then
  [[ $# -ge 2 && -n "$2" ]] || { echo "Usage: $0 [--camera NAME] [USHER_ARGS...]" >&2; exit 2; }
  selected_camera=$2
  shift 2
fi
if [[ -z "$selected_camera" && -r "$SCRIPT_DIR/camera.conf" ]]; then
  selected_camera=$(awk -F= '/^[[:space:]]*CAMERA[[:space:]]*=/ { gsub(/[[:space:]]/, "", $2); print $2 }' "$SCRIPT_DIR/camera.conf" | tail -n 1)
fi
selected_camera=${selected_camera:-spottercam}

echo "USHER: building local camera application (CAMERA=$selected_camera)"
make native

echo "USHER: starting verbose local mode (CAMERA=$selected_camera)"
echo "USHER: simulator endpoint is /tmp/usher-sim"
if [[ $# -gt 0 ]]; then
  exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug "$@"
fi
case "$selected_camera" in
  spottercam)
    exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug --test \
      --mock-image "$SCRIPT_DIR/resources/mock_aicam_01.jpg" \
      --spottercam --spottercam-settings "$SCRIPT_DIR/modules/spotter_cam/spottercam_settings.ini" \
      --spottercam-targets "$SCRIPT_DIR/examples/pangea/aicam-2028.ini" ;;
  aicam)
    exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug --mocktransform --test --aicam ;;
  lwircam)
    exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug --test --lwircam ;;
  chdk)
    exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug --test --chdk ;;
  earcam)
    exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug --test --earcam-sim 52.171291,4.412444,100 ;;
  loomcam)
    exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug --test --loomcam ;;
  diplacam)
    exec stdbuf -oL -eL "$SCRIPT_DIR/usher" --local --debug --diplacam-config "$SCRIPT_DIR/modules/dipla_cam/diplacam-nps.ini" ;;
  *)
    echo "run_local_demo.sh: unknown CAMERA '$selected_camera' (expected spottercam, aicam, lwircam, chdk, earcam, loomcam, or diplacam)" >&2
    exit 2 ;;
esac
