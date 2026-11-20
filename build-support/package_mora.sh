#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
package=$(realpath -m -- "${MORA_PACKAGE_DIR:-$root/.build/mora-package}")
install_dir=${MORA_INSTALL_DIR:-}
cross_compile=${CROSS_COMPILE:-aarch64-linux-gnu-}
profile=${MORA_PROFILE:-default}
camera=${MORA_CAMERA:-}
if [[ -z "$camera" && -r "$root/camera.conf" ]]; then
  camera=$(awk -F= '/^[[:space:]]*CAMERA[[:space:]]*=/ { gsub(/[[:space:]]/, "", $2); print $2 }' "$root/camera.conf" | tail -n 1)
fi
camera=${camera:-spottercam}
lwir_spotter_settings=${MORA_LWIR_SPOTTER_SETTINGS:-}
if [[ -n "$lwir_spotter_settings" && "$profile" != lwircam && "$camera" != lwircam ]]; then
  echo 'package_mora.sh: MORA_LWIR_SPOTTER_SETTINGS requires MORA_PROFILE=lwircam or CAMERA=lwircam' >&2
  exit 2
fi

case "$profile" in
  default)
    case "$camera" in
      spottercam|aicam|lwircam|chdk|earcam|loomcam) ;;
      diplacam)
        [[ "${MORA_INCLUDE_DIPLACAM:-0}" == 1 ]] || {
          echo 'package_mora.sh: CAMERA=diplacam requires MORA_INCLUDE_DIPLACAM=1' >&2
          exit 2
        }
        ;;
      *) echo "package_mora.sh: unknown CAMERA: $camera" >&2; exit 2 ;;
    esac
    service_camera=$camera
    ;;
  lwircam)
    service_camera=lwircam
    ;;
  diplacam)
    [[ "${MORA_INCLUDE_DIPLACAM:-0}" == 1 ]] || {
      echo 'package_mora.sh: Diplacam profile requires MORA_INCLUDE_DIPLACAM=1' >&2
      exit 2
    }
    service_camera=diplacam
    ;;
  *) echo "package_mora.sh: unknown MORA_PROFILE: $profile" >&2; exit 2 ;;
esac

if [[ "$package" != "$root/.build/"* ]]; then
  echo 'package_mora.sh: package destination must remain under USHER .build' >&2
  exit 2
fi
if [[ ! "$install_dir" =~ ^/[[:alnum:]_./+-]+$ || "$install_dir" == *..* ]]; then
  echo 'package_mora.sh: set MORA_INSTALL_DIR to a safe absolute target path' >&2
  exit 2
fi
for command_name in "${cross_compile}strip" "${cross_compile}readelf" sha256sum flock; do
  command -v "$command_name" >/dev/null || exit 1
done
mkdir -p -- "$(dirname -- "$package")"
exec 9>"$package.lock"
flock 9
temporary=$(mktemp -d "$package.XXXXXXXX")
trap 'rm -rf -- "$temporary"' EXIT
files=(install.manifest deploy_mora_remote.sh)
while read -r mode origin source destination || [[ -n "$mode" ]]; do
  if [[ "$profile" == lwircam ]]; then
    case "$destination" in
      bin/spottercam) [[ -n "$lwir_spotter_settings" ]] || continue ;;
      usher|usher.service|99-tiny1c.rules|licenses/jpeg/README|bin/lwircam|bin/lwir_calibrate|bin/soda|bin/camera_lens|bin/camera_lens_*.json|bin/libonnxruntime.so.1.31.0|licenses/onnxruntime/*) ;;
      *) continue ;;
    esac
  fi
  if [[ "$profile" == diplacam ]]; then
    case "$destination" in
      usher|usher.service|99-tiny1c.rules|licenses/jpeg/README|bin/libonnxruntime.so.1.31.0|licenses/onnxruntime/*) ;;
      *) continue ;;
    esac
  fi
  case "$origin" in
    arm64|source) input="$root/$source" ;;
    arm64-ear)
      input="$root/$source"
      ;;
    optional-loom)
      input=${MORA_LOOM_MODEL:-$root/$source}
      if [[ ! -e "$input" && -z "${MORA_LOOM_MODEL:-}" && "${MORA_REQUIRE_LOOM_MODEL:-0}" != 1 ]]; then
        printf 'LOOM model absent: packaging framework only; hardware LOOM needs its selected RPK\n' >&2
        continue
      fi
      [[ -f "$input" && -r "$input" ]] || { echo "Missing readable LOOM model: $input" >&2; exit 1; }
      ;;
    *) echo "Unknown package source: $origin" >&2; exit 1 ;;
  esac
  test -s "$input" || { echo "Missing package input: $input" >&2; exit 1; }
  output="$temporary/$destination"
  if [[ "$source" == usher.service.in ]]; then
    mkdir -p -- "$(dirname -- "$output")"
    sed "s|@USHER_INSTALL_DIR@|$install_dir|g" "$input" > "$output"
    case "$service_camera" in
      spottercam) ;;
      aicam)
        sed -i \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/spottercam$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/models/ai_cam/nn_vehicles.rpk$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/share/spottercam/spottercam_settings.ini$|d' \
          -e "s|^ExecStart=.*|ExecStartPre=/usr/bin/test -x $install_dir/bin/aicam\nExecStart=$install_dir/usher --aicam --native-photo|" \
          "$output"
        ;;
      lwircam)
        sed -i \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/spottercam$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/models/ai_cam/nn_vehicles.rpk$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/share/spottercam/spottercam_settings.ini$|d' \
          -e "s|^ExecStart=.*|ExecStartPre=/usr/bin/test -x $install_dir/bin/lwircam\nExecStart=$install_dir/usher --lwircam|" \
          "$output"
        ;;
      chdk)
        sed -i \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/spottercam$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/models/ai_cam/nn_vehicles.rpk$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/share/spottercam/spottercam_settings.ini$|d' \
          -e "s|^ExecStart=.*|ExecStartPre=/usr/bin/test -x $install_dir/bin/chdkbridge\nExecStart=$install_dir/usher --chdk|" \
          "$output"
        ;;
      earcam)
        sed -i \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/spottercam$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/soda$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/models/ai_cam/nn_vehicles.rpk$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/share/spottercam/spottercam_settings.ini$|d' \
          -e "s|^ExecStart=.*|ExecStartPre=/usr/bin/test -x $install_dir/bin/earcam\nExecStart=$install_dir/usher --earcam|" \
          "$output"
        ;;
      loomcam)
        sed -i \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/spottercam$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/models/ai_cam/nn_vehicles.rpk$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/share/spottercam/spottercam_settings.ini$|d' \
          -e "s|^ExecStart=.*|ExecStartPre=/usr/bin/test -x $install_dir/bin/loomcam\nExecStart=$install_dir/usher --loomcam|" \
          "$output"
        ;;
      diplacam)
        sed -i \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/spottercam$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -x .*/bin/soda$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/models/ai_cam/nn_vehicles.rpk$|d' \
          -e '\|^ExecStartPre=/usr/bin/test -r .*/share/spottercam/spottercam_settings.ini$|d' \
          -e "s|^ExecStart=.*|ExecStartPre=/usr/bin/test -x $install_dir/bin/diplacam\nExecStartPre=/usr/bin/test -r $install_dir/share/diplacam/example.ini\nExecStart=$install_dir/usher --diplacam-config $install_dir/share/diplacam/example.ini|" \
          "$output"
        ;;
    esac
    chmod "$mode" "$output"
  else
    install -D -m "$mode" "$input" "$output"
  fi
  if [[ "$origin" == arm64 || "$origin" == arm64-ear ]]; then
    if ! "${cross_compile}readelf" -h "$output" | grep -Eq 'Machine:[[:space:]]+AArch64'; then
      echo "Not an AArch64 ELF: $input" >&2
      exit 1
    fi
    "${cross_compile}strip" --strip-unneeded "$output"
    if [[ "$destination" == bin/pangea ]]; then
      if "${cross_compile}readelf" -lW "$output" | grep -q INTERP ||
          "${cross_compile}readelf" -d "$output" | grep -Eq 'NEEDED|RPATH|RUNPATH'; then
        echo "$destination release binary must be fully static" >&2
        exit 1
      fi
    fi
    if [[ "$destination" == bin/loomcam ]] &&
        "${cross_compile}readelf" -d "$output" | grep -Eq 'NEEDED.*libcamera'; then
      echo 'LOOMcam must retain its pinned static camera libraries' >&2
      exit 1
    fi
    if "${cross_compile}readelf" -SW "$output" | grep -E '\.(debug[[:alnum:]_]*|zdebug[[:alnum:]_]*|symtab|strtab)[[:space:]]' >/dev/null; then
      echo "Debug or static symbol sections remain in $destination" >&2
      exit 1
    fi
  fi
  printf '%s %s\n' "$mode" "$destination" >> "$temporary/install.manifest"
  files+=("$destination")
done < "$root/build-support/mora-package.list"
for pair in 'spottercam spotter_cam' 'loomcam loom_cam' 'earcam ear_cam' 'lwircam lwir_cam'; do
  if [[ "$profile" == diplacam ]]; then break; fi
  read -r application module <<< "$pair"
    if [[ "$profile" == lwircam && "$application" != lwircam
      && !( "$application" == spottercam && -n "$lwir_spotter_settings" ) ]]; then continue; fi
  settings_variable="MORA_${application^^}_SETTINGS"
  settings=${!settings_variable:-"$root/modules/$module/${application}_settings.ini"}
    if [[ "$profile" == lwircam && "$application" == spottercam ]]; then settings=$lwir_spotter_settings; fi
  bash "$root/build-support/package_models.sh" --package "$temporary" \
    --application "$application" --settings "$settings"
  while IFS= read -r -d '' filename; do
    relative=${filename#"$temporary/"}
    printf '0644 %s\n' "$relative" >> "$temporary/install.manifest"
    files+=("$relative")
  done < <(find "$temporary/share/$application" -type f -print0 | sort -z)
done
if [[ -n "$lwir_spotter_settings" ]]; then
  sed -i "s| --lwircam$| --lwircam --lwir-spotter-settings $install_dir/share/spottercam/spottercam_settings.ini|" "$temporary/usher.service"
fi
if [[ "${MORA_INCLUDE_DIPLACAM:-0}" == 1 ]]; then
  dipla="$root/.build/diplacam-package-aarch64"
  pushd "$dipla" >/dev/null
  sha256sum --check --status SHA256SUMS
  while read -r digest relative; do
    [[ "$relative" == bin/* || "$relative" == lib/diplacam/* || "$relative" == share/diplacam/* || "$relative" == licenses/diplacam/* ]] || exit 1
    mode=0644
    if [[ "$relative" == bin/* || "$relative" == lib/* ]]; then mode=0755; fi
    if [[ -f "$temporary/$relative" ]]; then
      cmp -s "$relative" "$temporary/$relative" || { echo "Conflicting shared package asset: $relative" >&2; exit 1; }
      continue
    fi
    install -D -m "$mode" "$relative" "$temporary/$relative"
    printf '%s %s\n' "$mode" "$relative" >> "$temporary/install.manifest"
    files+=("$relative")
  done < SHA256SUMS
  popd >/dev/null
fi
install -m 0755 "$root/deploy_mora_remote.sh" "$temporary/deploy_mora_remote.sh"
pushd "$temporary" >/dev/null
sha256sum "${files[@]}" > SHA256SUMS
sha256sum --check --status SHA256SUMS
popd >/dev/null
if [[ -e "$package" ]]; then
  rm -rf -- "$package"
fi
mv -- "$temporary" "$package"
printf 'MORA package: %s\n' "$package"