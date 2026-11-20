#!/usr/bin/env bash
# Copyright 2026 OpenUAS
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

ROOT=$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")
ACTIVATE=1
REFRESH=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-activate) ACTIVATE=0 ;;
    --refresh) REFRESH=1 ;;
    -h|--help)
      printf 'Usage: ./kickstart.sh [--no-activate] [--refresh]\n'
      printf 'Link Sensoraia into PAPARAZZI_HOME (sw/ext/sensoraia and conf/*/SENSORAIA) and install Center XML files.\n'
      printf 'Existing userconf XML files are kept untouched unless --refresh is given (which backs them up first).\n'
      printf 'By default, also points Paparazzi conf/conf.xml and conf/control_panel.xml to the Sensoraia demo files.\n'
      exit 0 ;;
    *) printf 'Unknown option: %s (use ./kickstart.sh --help)\n' "$1" >&2; exit 2 ;;
  esac
  shift
done

if [[ ! -d "${PAPARAZZI_HOME:-$HOME/paparazzi}" ]]; then
  printf 'Before you can use this project you must have the Paparazzi autopilot installed (set PAPARAZZI_HOME if not in ~/paparazzi).\n' >&2
  exit 1
fi
export PAPARAZZI_HOME=$(realpath -e -- "${PAPARAZZI_HOME:-$HOME/paparazzi}")
export PAPARAZZI_SRC="$PAPARAZZI_HOME"
export SENSORAIA_SRC="$ROOT"

for tool in make xmllint; do
  command -v "$tool" >/dev/null || { printf 'Required installed tool is missing: %s\n' "$tool" >&2; exit 1; }
done
[[ -f "$PAPARAZZI_HOME/conf/flight_plans/flight_plan.dtd" ]] || {
  printf 'PAPARAZZI_HOME must identify a prepared Paparazzi checkout: %s\n' "$PAPARAZZI_HOME" >&2
  exit 1
}

EXAMPLES="$ROOT/examples/paparazzi"
PLAN_DIR="$EXAMPLES/conf/flight_plans/SENSORAIA"
CONFIG="$EXAMPLES/conf/userconf/SENSORAIA/sensoraia_demo_conf.xml"
PANEL="$EXAMPLES/conf/userconf/SENSORAIA/sensoraia_demo_control_panel.xml"
[[ -d "$PLAN_DIR" && -f "$CONFIG" && -f "$PANEL" ]] || {
  printf 'Invalid Sensoraia source root: %s\n' "$ROOT" >&2
  exit 1
}
xmllint --noout "$CONFIG" "$PANEL"

STATE="${XDG_STATE_HOME:-$HOME/.local/state}/sensoraia/kickstart"
umask 077
mkdir -p -- "$STATE"
exec 9>"$STATE/kickstart.lock"
flock -n 9 || { printf 'Another kickstart is already running.\n' >&2; exit 1; }
umask 022

link_path() {
  local source=$1 destination=$2
  if [[ -L "$destination" && "$(readlink -f -- "$destination")" == "$(readlink -f -- "$source")" ]]; then
    return
  fi
  if [[ -e "$destination" || -L "$destination" ]]; then
    printf 'Refusing to replace existing path: %s\n' "$destination" >&2
    exit 1
  fi
  ln -s -- "$source" "$destination"
}

install_center_xml() {
  local source=$1 destination=$2 temporary previous
  if [[ -L "$destination" ]]; then
    if [[ "$(readlink -f -- "$destination")" != "$(readlink -f -- "$source")" ]]; then
      printf 'Refusing to replace existing path: %s\n' "$destination" >&2
      exit 1
    fi
  elif [[ -f "$destination" ]]; then
    if (( ! REFRESH )) || cmp -s "$source" "$destination"; then
      return
    fi
    previous=$(mktemp "$destination.previous.XXXXXXXX")
    cp -p -- "$destination" "$previous"
    printf 'Preserved previous %s in %s\n' "${destination##*/}" "$previous"
  elif [[ -e "$destination" ]]; then
    printf 'Refusing to replace existing path: %s\n' "$destination" >&2
    exit 1
  fi
  temporary=$(mktemp "$(dirname -- "$destination")/.sensoraia-center.XXXXXXXX")
  install -m 0644 -- "$source" "$temporary"
  mv -Tf -- "$temporary" "$destination"
}

mkdir -p "$PAPARAZZI_HOME/conf/userconf/SENSORAIA" \
  "$PAPARAZZI_HOME/conf/flight_plans" \
  "$PAPARAZZI_HOME/conf/airframes" \
  "$PAPARAZZI_HOME/conf/radios" \
  "$PAPARAZZI_HOME/conf/telemetry" \
  "$PAPARAZZI_HOME/sw/ext" \
  "$PAPARAZZI_HOME/doc"

link_path "$PAPARAZZI_HOME/conf/flight_plans/flight_plan.dtd" "$PLAN_DIR/../flight_plan.dtd"
for dtd_pair in 'airframes airframe.dtd' 'radios radio.dtd' 'telemetry telemetry.dtd'; do
  read -r subdir dtd_name <<< "$dtd_pair"
  if [[ -f "$PAPARAZZI_HOME/conf/$subdir/$dtd_name" && -d "$EXAMPLES/conf/$subdir" ]]; then
    link_path "$PAPARAZZI_HOME/conf/$subdir/$dtd_name" "$EXAMPLES/conf/$subdir/$dtd_name"
  fi
done

link_path "$ROOT" "$PAPARAZZI_HOME/sw/ext/sensoraia"
link_path "$PLAN_DIR" "$PAPARAZZI_HOME/conf/flight_plans/SENSORAIA"
if [[ -d "$EXAMPLES/conf/airframes/SENSORAIA" ]]; then
  link_path "$EXAMPLES/conf/airframes/SENSORAIA" "$PAPARAZZI_HOME/conf/airframes/SENSORAIA"
fi
if [[ -d "$EXAMPLES/conf/radios/SENSORAIA" ]]; then
  link_path "$EXAMPLES/conf/radios/SENSORAIA" "$PAPARAZZI_HOME/conf/radios/SENSORAIA"
fi
if [[ -d "$EXAMPLES/conf/telemetry/SENSORAIA" ]]; then
  link_path "$EXAMPLES/conf/telemetry/SENSORAIA" "$PAPARAZZI_HOME/conf/telemetry/SENSORAIA"
fi
if [[ -d "$ROOT/docs" ]]; then
  link_path "$ROOT/docs" "$PAPARAZZI_HOME/doc/sensoraia"
fi

retired_panel="$PAPARAZZI_HOME/conf/userconf/SENSORAIA/sensoraia_demo_control_panel_ex.xml"
if [[ -L "$retired_panel" && ! -e "$retired_panel" &&
      "$(readlink -- "$retired_panel")" == */examples/paparazzi/conf/userconf/SENSORAIA/sensoraia_demo_control_panel_ex.xml ]]; then
  rm -f -- "$retired_panel"
fi

install_center_xml "$CONFIG" "$PAPARAZZI_HOME/conf/userconf/SENSORAIA/sensoraia_demo_conf.xml"
install_center_xml "$PANEL" "$PAPARAZZI_HOME/conf/userconf/SENSORAIA/sensoraia_demo_control_panel.xml"

if (( ACTIVATE )); then
  for name in conf.xml control_panel.xml; do
    if [[ "$name" == conf.xml ]]; then
      target=userconf/SENSORAIA/sensoraia_demo_conf.xml
    else
      target=userconf/SENSORAIA/sensoraia_demo_control_panel.xml
    fi
    current="$PAPARAZZI_HOME/conf/$name"
    [[ ! -d "$current" ]] || { printf 'Refusing to replace a directory: %s\n' "$current" >&2; exit 1; }
    if [[ -L "$current" && "$(readlink -- "$current")" == "$target" ]]; then
      continue
    fi
    if [[ -e "$current" || -L "$current" ]]; then
      backup=$(mktemp -d "$STATE/previous.XXXXXXXX")
      cp -P -- "$current" "$backup/$name"
      printf 'Preserved previous %s in %s\n' "$name" "$backup"
    fi
    temporary=$(mktemp -d "$PAPARAZZI_HOME/conf/.sensoraia-link.XXXXXXXX")
    ln -s -- "$target" "$temporary/$name"
    mv -Tf -- "$temporary/$name" "$current"
    rmdir -- "$temporary"
    printf 'Linked %s -> %s\n' "$name" "$target"
  done
fi

printf '\nSensoraia Paparazzi links are ready in %s\n' "$PAPARAZZI_HOME"
printf 'Next steps:\n'
printf '  1. Build Sensoraia on your PC (if not built yet):  make native\n'
printf '  2. Start Paparazzi Center:                         cd "%s" && ./paparazzi\n' "$PAPARAZZI_HOME"
printf '  3. Pick your camera aircraft (e.g. Easystar_3_SPOTTERCAM), target "nps", and click Build.\n'
printf '  4. In the Session tab, select the matching "Sensoraia <Camera> NPS" session and click Execute.\n'
