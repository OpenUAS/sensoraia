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

install_dir=${1:?installation directory required}
stage=${2:?staging directory required}
system_dir=${3:-/etc}
backup="$stage/backup"
changed=false
stopped=false
was_active=false
was_enabled=false

if [[ ! "$install_dir" =~ ^/[[:alnum:]_./+-]+$ || "$install_dir" == *..* ]]; then
  echo 'deploy_mora_remote.sh: installation directory must be a safe absolute path' >&2
  exit 2
fi

if [[ -n "${HOME:-}" && -f "$HOME/pass.txt" && "$(tr -d '\r\n' < "$HOME/pass.txt")" != replacewithyourpass ]]; then
  sudo -S -p "" -v < "$HOME/pass.txt" 2>/dev/null || true
fi

exec 9>"$install_dir/.deploy.lock"
flock -n 9 || { echo 'Another USHER deployment is in progress' >&2; exit 1; }

pushd "$stage" >/dev/null
sha256sum --check --status SHA256SUMS
popd >/dev/null
files=()
modes=()
destinations=()
retired_files=(
  vehicle_detect_server.py models/vehicle/labels.txt models/vehicle/vehicle_detect.json models/vehicle/network.rpk
  soda camera_lens pangea lwircam earcam spottercam loomcam ear_heatmap_replay
  object_detect.json camera_lens_aicam.json camera_lens_fisheye160.json models/object/object_detect.json
  object_detect_basic bin/object_detect_basic bin/object_detect.json models/object/network.rpk
  bin/rpicam-still bin/testbox rpicam/lib/librpicam_app.so.1
  rpicam/lib/rpicam-apps-postproc/core-postproc.so rpicam/lib/rpicam-apps-postproc/imx500-postproc.so
  rpicam/bin/rpicam-still rpicam/bin/testbox
  mock_image_01.jpg mock_aicam_01.jpg mock_chdk_01.jpg mock_lwir_01.jpg
)
while read -r mode name extra || [[ -n "$mode" ]]; do
  if [[ ! "$mode" =~ ^0(644|755)$ || -n "$extra" ||
        ! "$name" =~ ^[[:alnum:]_][[:alnum:]_./+-]*$ || "$name" == *..* ]]; then
    echo 'Invalid release manifest entry' >&2
    exit 1
  fi
  for previous in "${files[@]}"; do
    [[ "$previous" != "$name" ]] || { echo "Duplicate release path: $name" >&2; exit 1; }
  done
  test -s "$stage/$name"
  files+=("$name")
  modes+=("$mode")
  destinations+=("$install_dir/$name")
done < "$stage/install.manifest"
test "${#files[@]}" -gt 0
has_release_file()
{
  printf '%s\n' "${files[@]}" | grep -Fxq -- "$1"
}
destinations+=(
  "$system_dir/systemd/system/usher.service" "$system_dir/udev/rules.d/99-tiny1c.rules"
)
if has_release_file 99-chdkbridge.rules; then destinations+=("$system_dir/udev/rules.d/99-chdkbridge.rules"); fi
for name in "${retired_files[@]}"; do destinations+=("$install_dir/$name"); done

rollback()
{
  local status=$?
  trap - EXIT HUP INT TERM
  if [[ $status -ne 0 && "$stopped" == true ]]; then
    echo "MORA USHER: activation failed; restoring previous installation from $backup" >&2
    if [[ "$changed" == true ]]; then
      local rollback_load
      rollback_load=$(sudo -n systemctl show usher.service -p LoadState --value) || exit "$status"
      if [[ "$rollback_load" != not-found ]] && ! sudo -n systemctl stop usher.service; then
        echo 'Rollback aborted: cannot stop USHER safely; backup retained' >&2
        exit "$status"
      fi
      if [[ "$was_enabled" == false && -e "$system_dir/systemd/system/usher.service" ]]; then
        sudo -n systemctl disable usher.service || exit "$status"
      fi
      for index in "${!destinations[@]}"; do
        if ! sudo -n rm -f -- "${destinations[index]}"; then exit "$status"; fi
        if [[ -e "$backup/$index" || -L "$backup/$index" ]]; then
          if ! sudo -n cp -a -- "$backup/$index" "${destinations[index]}"; then exit "$status"; fi
        fi
      done
      sudo -n systemctl daemon-reload || exit "$status"
      if [[ "$was_enabled" == true ]]; then
        sudo -n systemctl enable usher.service || exit "$status"
      fi
      sudo -n udevadm control --reload-rules || exit "$status"
      sudo -n udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=0bda --attr-match=idProduct=5840 || exit "$status"
      sudo -n udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=04a9 || exit "$status"
    fi
    if [[ "$was_active" == true ]]; then
      sudo -n systemctl start usher.service || exit "$status"
    fi
  fi
  exit "$status"
}
trap rollback EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

for name in "${files[@]}"; do
    if [[ "$name" == bin/earcam || "$name" == bin/aicam || "$name" == bin/spottercam || "$name" == bin/loomcam ||
      "$name" == bin/lwircam || "$name" == bin/libonnxruntime.so.1.31.0 ||
      "$name" == bin/chdkbridge || "$name" == bin/diplacam ]]; then
    if ! dependencies=$(ldd "$stage/$name" 2>&1) ||
        [[ "$dependencies" == *'not found'* ]]; then
      printf 'MORA USHER: dependency check failed for %s\n%s\n' "$name" "$dependencies" >&2
      exit 1
    fi
  fi
done
if has_release_file bin/diplacam; then
  "$stage/bin/diplacam" --field "$stage/share/diplacam/field.csv" \
    --settings "$stage/share/diplacam/diplacam_settings.ini" >/dev/null
fi
for application in spottercam loomcam earcam lwircam; do
  settings="$stage/share/$application/${application}_settings.ini"
  if [[ -f "$settings" ]] && grep -Eq '^inference_backend[[:space:]]*=[[:space:]]*onnx[[:space:]]*$' "$settings"; then
    "$stage/bin/$application" --check-model --settings "$settings" >/dev/null
  fi
done
if grep -Eq '^ExecStart=.*--native-photo([[:space:]]|$)' "$stage/usher.service"; then
  if grep -Eq '^ExecStart=.*--aicam([[:space:]]|$)' "$stage/usher.service" && ! has_release_file bin/aicam; then
    echo 'MORA USHER: --aicam --native-photo requires bin/aicam in the release' >&2
    exit 1
  fi
  if grep -Eq '^ExecStart=.*--spottercam([[:space:]]|$)' "$stage/usher.service" && ! has_release_file bin/spottercam; then
    echo 'MORA USHER: --spottercam --native-photo requires bin/spottercam in the release' >&2
    exit 1
  fi
elif command -v rpicam-still >/dev/null; then
  rpicam-still --version >/dev/null
elif grep -Eq '^ExecStart=.*--aicam([[:space:]]|$)' "$stage/usher.service"; then
  echo 'MORA USHER: --aicam requires the OS-installed rpicam-still executable' >&2
  exit 1
fi
for application in aicam spottercam pangea lwir_calibrate loomcam chdkbridge; do
  if has_release_file "bin/$application"; then
    LD_BIND_NOW=1 "$stage/bin/$application" --version >/dev/null
  fi
done
if has_release_file bin/loomcam; then "$stage/bin/loomcam" --self-test; fi
if has_release_file bin/chdkbridge; then "$stage/bin/chdkbridge" --check-runtime; fi
if has_release_file bin/earcam; then "$stage/bin/earcam" --self-test; fi
# Raw replay validates encoding without a camera or optional mock-analysis code.
if has_release_file bin/lwircam; then
  printf '\120\200\120\200\120\106\120\106' > "$stage/mock-validation.yuyt"
  "$stage/bin/lwircam" --raw "$stage/mock-validation.yuyt" --size 2 1 "$stage/mock-validation.jpg"
  test -s "$stage/mock-validation.jpg"
fi
load_state=$(sudo -n systemctl show usher.service -p LoadState --value)
if [[ "$load_state" != not-found ]]; then
  enabled_state=$(sudo -n systemctl show usher.service -p UnitFileState --value)
  case "$enabled_state" in
    enabled) was_enabled=true ;;
    disabled) ;;
    *) echo "Unsupported prior service enablement: $enabled_state" >&2; exit 1 ;;
  esac
  active_state=$(sudo -n systemctl show usher.service -p ActiveState --value)
  case "$active_state" in
    active|activating|reloading) was_active=true ;;
    inactive|failed) ;;
    *) echo "Service is changing state: $active_state" >&2; exit 1 ;;
  esac
fi
mkdir "$backup"
for index in "${!destinations[@]}"; do
  if [[ -e "${destinations[index]}" || -L "${destinations[index]}" ]]; then
    sudo -n cp -a -- "${destinations[index]}" "$backup/$index"
  fi
done
if [[ "$load_state" != not-found ]]; then
  sudo -n systemctl stop usher.service
  stopped=true
  test "$(sudo -n systemctl show usher.service -p ActiveState --value)" = inactive
fi
stopped=true
if pgrep -af "^$install_dir/(usher|((bin/)?(aicam|lwircam|earcam|loomcam|spottercam|diplacam|object_detect_basic)))( |$)|(^|/)rpicam-still( |$)"; then
  stopped=false
  echo 'Stop the unmanaged camera processes before deploying' >&2
  exit 1
else
  result=$?
  [[ $result -eq 1 ]] || exit "$result"
fi
changed=true
mkdir -p "$install_dir/photos" "$install_dir/earlogs" "${HOME:?}/Pictures" "${HOME:?}/Sensoraia/diagnostics" \
  "$install_dir/models/ai_cam" \
  "$install_dir/bin"
for index in "${!files[@]}"; do
  install -D -m "${modes[index]}" "$stage/${files[index]}" "${destinations[index]}"
done
for name in "${retired_files[@]}"; do rm -f -- "$install_dir/$name"; done
sudo -n install -m 0644 "$stage/usher.service" "$system_dir/systemd/system/usher.service"
sudo -n install -m 0644 "$stage/99-tiny1c.rules" "$system_dir/udev/rules.d/99-tiny1c.rules"
if has_release_file 99-chdkbridge.rules; then
  sudo -n install -m 0644 "$stage/99-chdkbridge.rules" "$system_dir/udev/rules.d/99-chdkbridge.rules"
fi
sudo -n systemd-analyze verify "$system_dir/systemd/system/usher.service"
sudo -n udevadm control --reload-rules
sudo -n udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=0bda --attr-match=idProduct=5840
sudo -n udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=04a9
sudo -n systemctl daemon-reload
card=$(awk -F'[][]' '/USB-Audio/ {print $2; exit}' /proc/asound/cards 2>/dev/null | tr -d ' ') || card=
if [[ -n "$card" ]]; then
  amixer -q -c "$card" set 'Auto Gain Control' off 2>/dev/null || echo 'Warning: cannot disable microphone AGC' >&2
  if ! amixer -q -c "$card" set Mic 14 cap 2>/dev/null && ! amixer -q -c "$card" set Mic 88% cap 2>/dev/null; then
    echo 'Warning: cannot configure microphone gain' >&2
  fi
else
  echo 'Warning: no USB audio capture card enumerated' >&2
fi
sudo -n systemctl enable usher.service
sudo -n systemctl restart usher.service
sudo -n systemctl is-enabled --quiet usher.service
sudo -n systemctl is-active --quiet usher.service
test "$(sudo -n systemctl show usher.service -p SubState --value)" = running
changed=false
stopped=false
echo "MORA USHER: deployment active; previous installation retained in $backup"