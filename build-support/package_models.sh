#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

fail() {
  printf 'package_models.sh: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '%s\n' 'Usage: package_models.sh --package DIR --application NAME --settings FILE [--worker-config FILE]' \
    'Applications: diplacam, spottercam, loomcam, earcam, lwircam'
}

read_fields() {
  local filename=$1 limit=$2 size parsed key value
  size=$(stat -Lc %s -- "$filename") || fail "Cannot read INI: $filename"
  [[ -f "$filename" && "$size" -gt 0 && "$size" -le "$limit" ]] || fail "Invalid bounded INI: $filename"
  parsed=$(awk '
    BEGIN { failed = 0 }
    {
      if ($0 ~ /[^\t\r\040-\176]/) { failed = 1; exit }
      sub(/\r$/, "")
      if ($0 ~ /^[ \t]*([#;]|$)/) next
      indent = match($0, /[^ \t]/) - 1
      if (have_field && indent > previous_indent) { failed = 1; exit }
      separator = index($0, "=")
      if (!separator) { failed = 1; exit }
      key = substr($0, 1, separator - 1)
      value = substr($0, separator + 1)
      gsub(/^[ \t]+|[ \t]+$/, "", key)
      gsub(/^[ \t]+|[ \t]+$/, "", value)
      if (key !~ /^[A-Za-z0-9_.]+$/ || value == "" || value ~ /\r/ || seen[key]++) {
        failed = 1; exit
      }
      have_field = 1
      previous_indent = indent
      print key "=" value
    }
    END { if (failed) exit 1 }
  ' "$filename") || fail "Invalid flat INI: $filename"
  fields=()
  field_names=()
  while IFS='=' read -r key value; do
    [[ -n "$key" ]] || continue
    fields["$key"]=$value
    field_names+=("$key")
  done <<< "$parsed"
}

set_field() {
  local key=$1 value=$2
  if [[ ! -v 'fields[$key]' ]]; then field_names+=("$key"); fi
  fields["$key"]=$value
}

digest() {
  local checksum
  checksum=$(sha256sum -- "$1") || fail "Cannot hash: $1"
  printf '%s\n' "${checksum%% *}"
}

copy_file() {
  local source=$1 destination=$2 name=${1##*/}
  [[ "$name" =~ ^[A-Za-z0-9_.+-]+$ ]] || fail "Unsupported package filename: $name"
  [[ -f "$source" && -s "$source" ]] || fail "Missing model bundle input: $source"
  if [[ -e "$destination" ]]; then
    [[ "$(digest "$source")" == "$(digest "$destination")" ]] || fail "Conflicting model bundle destination: $destination"
    return
  fi
  mkdir -p -- "$(dirname -- "$destination")"
  (set -o noclobber; cat -- "$source" > "$destination") || fail "Cannot copy model bundle input: $source"
  chmod 0644 -- "$destination"
}

resolve_input() {
  local value=$1
  if [[ "$value" != /* ]]; then value="$settings_directory/$value"; fi
  realpath -m -- "$value"
}

package=
application=
settings=
worker_config=
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --package|--application|--settings|--worker-config)
      [[ "$#" -ge 2 && -n "$2" ]] || fail "Missing value for $1"
      case "$1" in
        --package) package=$2 ;;
        --application) application=$2 ;;
        --settings) settings=$2 ;;
        --worker-config) worker_config=$2 ;;
      esac
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown argument: $1" ;;
  esac
done
[[ -n "$package" && -n "$application" && -n "$settings" ]] || { usage >&2; exit 2; }
case "$application" in
  diplacam) adapter=dipla_ddim_v1 ;;
  spottercam) adapter=spotter_boxes_xyxy_v1 ;;
  loomcam) adapter=loom_features_v1 ;;
  earcam) adapter=ear_pcm_classifier_v1 ;;
  lwircam) adapter=lwir_classifier_v1 ;;
  *) fail "Unsupported application: $application" ;;
esac
settings=$(realpath -m -- "$settings")
package=$(realpath -m -- "$package")
settings_directory=$(dirname -- "$settings")
declare -A fields=()
declare -a field_names=()
read_fields "$settings" 8192
backend=${fields[inference_backend]:-native}
if [[ "$application" == diplacam ]]; then backend=${fields[inference_backend]:-onnx}; fi
if [[ "$backend" == auto ]]; then
  model=${fields[model]:-}
  case "$model" in
    '') backend=native ;;
    *.onnx)
      backend=onnx
      if [[ ! -v 'fields[model_descriptor]' ]]; then set_field model_descriptor "${model%.onnx}.ini"; fi
      ;;
    *.rpk) backend=imx500 ;;
    *) fail "Automatic inference requires a .onnx or .rpk model: $settings" ;;
  esac
  set_field inference_backend "$backend"
fi
case "$backend" in onnx|imx500|native) ;; *) fail "Unsupported backend: $backend" ;; esac
destination="$package/share/$application"
mkdir -p -- "$destination"
if [[ "$application" == diplacam ]]; then
  model_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../modules/dipla_cam/models" && pwd -P)
  while IFS= read -r -d '' asset; do
    relative=${asset#"$model_directory/"}
    target="$destination/models/$relative"
    if [[ -d "$asset" && ! -L "$asset" ]]; then
      mkdir -p -- "$target"
    elif [[ -f "$asset" && ! -L "$asset" ]]; then
      if [[ -e "$target" ]]; then
        cmp -s -- "$asset" "$target" || fail "Conflicting model bundle destination: $target"
      else
        install -D -m 0644 -- "$asset" "$target"
      fi
    else
      fail "Unsupported Diplacam model asset: $asset"
    fi
  done < <(find "$model_directory" -mindepth 1 -print0 | sort -z)
fi
if [[ "$backend" == onnx && !( "$application" == diplacam && "${fields[cmaes]:-}" == true ) ]]; then
  [[ -n ${fields[model]:-} && -n ${fields[model_descriptor]:-} ]] || fail "Missing model or model_descriptor: $settings"
  graph=$(resolve_input "${fields[model]}")
  descriptor=$(resolve_input "${fields[model_descriptor]}")
  (
    read_fields "$descriptor" 16384
    [[ "${fields[schema]:-}" == SENSORAIA_MODEL_V1 && "${fields[adapter]:-}" == "$adapter" ]] || fail "Unsupported $application model descriptor: $descriptor"
    [[ "${fields[model_sha256]:-}" == "$(digest "$graph")" ]] || fail "Model checksum mismatch: $graph"
  )
  copy_file "$graph" "$destination/models/${graph##*/}"
  set_field model "models/${graph##*/}"
  copy_file "$descriptor" "$destination/models/${descriptor##*/}"
  set_field model_descriptor "models/${descriptor##*/}"
  if [[ -v 'fields[metadata]' ]]; then
    source=$(resolve_input "${fields[metadata]}")
    copy_file "$source" "$destination/${source##*/}"
    set_field metadata "${source##*/}"
  fi
  set_field onnx_lib_path ../../bin
elif [[ "$backend" == imx500 ]]; then
  [[ -n ${fields[model]:-} ]] || fail "Missing model: $settings"
  source=$(resolve_input "${fields[model]}")
  staged="$package/models/ai_cam/${source##*/}"
  if [[ -f "$staged" ]] && { [[ ! -e "$source" ]] || [[ "$(digest "$staged")" == "$(digest "$source")" ]]; }; then
    set_field model "../../models/ai_cam/${source##*/}"
  elif [[ -f "$source" ]]; then
    copy_file "$source" "$destination/models/${source##*/}"
    set_field model "models/${source##*/}"
  elif [[ "$application" == loomcam ]]; then
    set_field model "../../models/ai_cam/${source##*/}"
  else
    fail "Missing $application RPK: $source"
  fi
fi
if [[ -n ${fields[input]:-} ]]; then
  source=$(resolve_input "${fields[input]}")
  copy_file "$source" "$destination/inputs/${source##*/}"
  set_field input "inputs/${source##*/}"
fi
if [[ "$application" == diplacam && -n ${fields[conditioning_cache]:-${DIPLA_CONDITIONING_CACHE:-}} ]]; then
  [[ "$backend" == onnx && "${fields[cmaes]:-false}" != true ]] || fail 'Conditioning cache needs diffusion ONNX'
  cache=$(resolve_input "${fields[conditioning_cache]:-$DIPLA_CONDITIONING_CACHE}")
  for asset in bundle.ini encoder.ini denoiser.ini encoder.onnx denoiser.onnx manifest.json; do
    copy_file "$cache/$asset" "$destination/conditioning-cache/$asset"
  done
  (
    set -o noclobber
    for key in "${field_names[@]}"; do
      if [[ "$key" != conditioning_cache ]]; then printf '%s=%s\n' "$key" "${fields[$key]}"; fi
    done > "$destination/diplacam_reference_settings.ini"
  ) || fail 'Cannot preserve reference Diplacam settings'
  chmod 0644 -- "$destination/diplacam_reference_settings.ini"
  set_field conditioning_cache conditioning-cache
fi
output="$destination/${application}_settings.ini"
(
  set -o noclobber
  for key in "${field_names[@]}"; do printf '%s=%s\n' "$key" "${fields[$key]}"; done > "$output"
) || fail "Cannot create settings: $output"
chmod 0644 -- "$output"
if [[ "$application" == diplacam && -v 'fields[conditioning_cache]' ]]; then
  copy_file "$output" "$destination/diplacam_cached_settings.ini"
fi
if [[ -n "$worker_config" ]]; then
  [[ "$application" == diplacam ]] || fail 'Worker configuration is supported only for Diplacam'
  (
    read_fields "$worker_config" 8192
    [[ -v 'fields[field]' ]] || fail 'Diplacam worker configuration requires field'
    for key in "${field_names[@]}"; do
      case "$key" in
        field|cadence_ms|planning_pose_max_age_ms|field_size_m|field_rotation_deg|flight_height_m|flight_height_tolerance_m) ;;
        *) fail "Unsupported packaged worker setting: $key; select models through application settings" ;;
      esac
    done
    set_field field share/diplacam/field.csv
    output="$destination/example.ini"
    (
      set -o noclobber
      for key in "${field_names[@]}"; do printf '%s=%s\n' "$key" "${fields[$key]}"; done > "$output"
    ) || fail "Cannot create worker configuration: $output"
    chmod 0644 -- "$output"
  )
fi