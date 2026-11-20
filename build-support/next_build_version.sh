#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 7 ]]; then
  echo "Usage: $0 STATE_FILE LOCK_FILE APPLICATION MACRO OUTPUT_HEADER -- COMMAND [ARGUMENT...]" >&2
  exit 2
fi

state_file=$1
lock_file=$2
application=$3
macro=$4
output_header=$5
shift 5
[[ $1 == -- ]] || {
  echo "Missing -- before build command" >&2
  exit 2
}
shift
[[ $# -gt 0 ]] || {
  echo "Missing build command" >&2
  exit 2
}

[[ "$application" =~ ^[a-z][a-z0-9_]*$ ]] || {
  echo "Invalid application name: $application" >&2
  exit 2
}
[[ "$macro" =~ ^[A-Z][A-Z0-9_]*$ ]] || {
  echo "Invalid version macro: $macro" >&2
  exit 2
}
[[ -f "$state_file" ]] || {
  echo "Version state file not found: $state_file" >&2
  exit 2
}
command -v flock >/dev/null || {
  echo "flock is required to allocate build versions" >&2
  exit 2
}
bump=${VERSION_BUMP:-0}
[[ "$bump" == 0 || "$bump" == 1 ]] || {
  echo "VERSION_BUMP must be 0 or 1" >&2
  exit 2
}
if [[ "$bump" == 1 && " ${MAKEFLAGS:-} " =~ [[:space:]]n[[:space:]] ]]; then
  echo "Version allocation suppressed during make dry-run" >&2
  bump=0
fi

mkdir -p "$(dirname -- "$lock_file")" "$(dirname -- "$output_header")"
exec 9>"$lock_file"
flock -x 9

state_tmp=$(mktemp "${state_file}.tmp.XXXXXX")
version_tmp=$(mktemp "${state_file}.version.XXXXXX")
header_tmp=$(mktemp "${output_header}.tmp.XXXXXX")
trap 'rm -f "$state_tmp" "$version_tmp" "$header_tmp"' EXIT

awk -v wanted="$application" -v version_file="$version_tmp" -v bump="$bump" '
  {
    lines[NR] = $0
    if ($0 ~ /^\[[a-z][a-z0-9_]*\]$/) {
      section = substr($0, 2, length($0) - 2)
      if (section == wanted) section_count++
      next
    }
    if (section != wanted || $0 !~ /^(major|minor|patch)=/) next
    split($0, fields, "=")
    if (fields[2] !~ /^[0-9]+$/) {
      printf "Invalid numeric version field for [%s]: %s\n", wanted, $0 > "/dev/stderr"
      invalid = 1
      next
    }
    if (fields[1] == "major") { major = fields[2]; major_count++ }
    if (fields[1] == "minor") { minor = fields[2]; minor_count++ }
    if (fields[1] == "patch") { patch = fields[2]; patch_count++; patch_line = NR }
  }
  END {
    if (invalid || section_count != 1 || major_count != 1 || minor_count != 1 || patch_count != 1) {
      printf "Invalid or incomplete version section [%s]\n", wanted > "/dev/stderr"
      exit 1
    }
    if (bump == 1) patch++
    for (line = 1; line <= NR; line++) {
      if (line == patch_line) print "patch=" patch
      else print lines[line]
    }
    print major "." minor "." patch > version_file
  }
' "$state_file" >"$state_tmp"

version=$(<"$version_tmp")
guard="${macro}_GENERATED_H"
printf '#ifndef %s\n#define %s\n#define %s "%s"\n#endif\n' \
  "$guard" "$guard" "$macro" "$version" >"$header_tmp"
chmod --reference="$state_file" "$state_tmp"

if ! cmp -s -- "$header_tmp" "$output_header"; then
  mv -f -- "$header_tmp" "$output_header"
else
  rm -f -- "$header_tmp"
  if [[ "$output_header" -ot "$state_file" || "$output_header" -ot "${BASH_SOURCE[0]}" ]]; then
    touch -- "$output_header"
  fi
fi
printf '%s\n' "$version"
"$@"
if [[ "$bump" == 1 ]]; then
  mv -f -- "$state_tmp" "$state_file"
fi