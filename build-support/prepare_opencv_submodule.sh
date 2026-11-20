#!/usr/bin/env bash
# Copyright 2026 OpenUAS
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)

readonly OPENCV_SUBMODULE_PATH="ext/opencv"
readonly OPENCV_EXPECTED_URL="https://github.com/opencv/opencv.git"
readonly OPENCV_EXPECTED_TAG="4.14.0"
readonly OPENCV_EXPECTED_TAG_OBJECT="ea8e60798e0319ccbe902f7994153b574f639809"
readonly OPENCV_EXPECTED_COMMIT="0654a42e19215ef25b1d367d822f3c630447e7c7"

verify_only=0
if [[ "${1:-}" == "--verify-only" ]]; then
  verify_only=1
  shift
fi

opencv_dir=$(realpath -m -- "${1:-$root/$OPENCV_SUBMODULE_PATH}")
stamp_path=${2:-}
if [[ $# -gt 2 ]]; then
  printf 'Usage: %s [--verify-only] [OPENCV_DIR] [STAMP_PATH]\n' "$0" >&2
  exit 2
fi
if [[ "$opencv_dir" != "$root/$OPENCV_SUBMODULE_PATH" ]]; then
  printf 'prepare_opencv_submodule.sh: OpenCV source must live at %s (got %s)\n' \
    "$root/$OPENCV_SUBMODULE_PATH" "$opencv_dir" >&2
  exit 2
fi

verify_version_header() {
  local header=$1
  [[ -f "$header" ]] || {
    printf 'prepare_opencv_submodule.sh: missing OpenCV version header: %s\n' "$header" >&2
    return 1
  }
  awk -v expected="$OPENCV_EXPECTED_TAG" '
    $1 == "#define" && $2 == "CV_VERSION_MAJOR"    { major = $3 }
    $1 == "#define" && $2 == "CV_VERSION_MINOR"    { minor = $3 }
    $1 == "#define" && $2 == "CV_VERSION_REVISION" { rev = $3 }
    $1 == "#define" && $2 == "CV_VERSION_STATUS"   { status = $3 }
    END {
      actual = major "." minor "." rev
      if (actual != expected || status != "\"\"") {
        printf "prepare_opencv_submodule.sh: OpenCV header %s reports %s%s; expected exact release tag %s\n",
          FILENAME, actual, status, expected > "/dev/stderr"
        exit 1
      }
    }
  ' "$header"
}

is_submodule_repo() {
  [[ -e "$opencv_dir/.git" ]] || return 1
  local toplevel resolved_opencv
  resolved_opencv=$(cd -- "$opencv_dir" && pwd -P)
  toplevel=$(git -C "$opencv_dir" rev-parse --show-toplevel 2>/dev/null || true)
  [[ "$toplevel" == "$resolved_opencv" ]]
}

if (( ! verify_only )); then
  command -v flock >/dev/null || {
    printf 'prepare_opencv_submodule.sh: missing required tool: flock\n' >&2
    exit 1
  }
  mkdir -p -- "$root/.build"
  exec 9>"$root/.build/opencv-submodule.lock"
  flock 9
fi

needs_update=0
if [[ -e "$root/.git" ]]; then
  command -v git >/dev/null || {
    printf 'prepare_opencv_submodule.sh: missing required tool: git\n' >&2
    exit 1
  }
  [[ -f "$root/.gitmodules" ]] || {
    printf 'prepare_opencv_submodule.sh: missing %s/.gitmodules\n' "$root" >&2
    exit 1
  }
  configured_path=$(git config --file "$root/.gitmodules" --get "submodule.$OPENCV_SUBMODULE_PATH.path" || true)
  configured_url=$(git config --file "$root/.gitmodules" --get "submodule.$OPENCV_SUBMODULE_PATH.url" || true)
  if [[ "$configured_path" != "$OPENCV_SUBMODULE_PATH" || "$configured_url" != "$OPENCV_EXPECTED_URL" ]]; then
    printf 'prepare_opencv_submodule.sh: .gitmodules must configure %s -> %s (got path=%q url=%q)\n' \
      "$OPENCV_SUBMODULE_PATH" "$OPENCV_EXPECTED_URL" "$configured_path" "$configured_url" >&2
    exit 1
  fi

  index_entry=$(git -C "$root" ls-files --stage -- "$OPENCV_SUBMODULE_PATH")
  index_mode=""
  index_sha=""
  if [[ -n "$index_entry" ]]; then
    read -r index_mode index_sha _ <<< "$index_entry"
  fi
  if [[ "$index_mode" != "160000" || "$index_sha" != "$OPENCV_EXPECTED_COMMIT" ]]; then
    printf 'prepare_opencv_submodule.sh: superproject gitlink for %s must be 160000 %s (OpenCV tag %s); got: %s\n' \
      "$OPENCV_SUBMODULE_PATH" "$OPENCV_EXPECTED_COMMIT" "$OPENCV_EXPECTED_TAG" "${index_entry:-<missing>}" >&2
    exit 1
  fi

  if (( ! verify_only )); then
    if ! is_submodule_repo; then
      if [[ -d "$opencv_dir" ]] && [[ -n "$(ls -A -- "$opencv_dir")" ]]; then
        printf 'prepare_opencv_submodule.sh: %s is non-empty but not an initialized Git submodule; remove or move it and rerun make\n' \
          "$OPENCV_SUBMODULE_PATH" >&2
        exit 1
      fi
      needs_update=1
    else
      git -C "$root" submodule sync --recursive -- "$OPENCV_SUBMODULE_PATH" >/dev/null
      current_commit=$(git -C "$opencv_dir" rev-parse --verify HEAD 2>/dev/null || true)
      if [[ "$current_commit" != "$OPENCV_EXPECTED_COMMIT" ||
            ! -f "$opencv_dir/CMakeLists.txt" ||
            ! -f "$opencv_dir/3rdparty/libjpeg/jpeglib.h" ]]; then
        if ! git -C "$opencv_dir" diff --quiet || ! git -C "$opencv_dir" diff --cached --quiet; then
          printf 'prepare_opencv_submodule.sh: %s has local modifications at %s; refusing to overwrite while restoring OpenCV tag %s (%s)\n' \
            "$OPENCV_SUBMODULE_PATH" "${current_commit:-unknown}" "$OPENCV_EXPECTED_TAG" "$OPENCV_EXPECTED_COMMIT" >&2
          exit 1
        fi
        needs_update=1
      fi
    fi

    if (( needs_update )); then
      printf 'Preparing OpenCV %s submodule in %s...\n' "$OPENCV_EXPECTED_TAG" "$OPENCV_SUBMODULE_PATH"
      git -C "$root" submodule sync --recursive -- "$OPENCV_SUBMODULE_PATH"
      if ! git -C "$root" submodule update --init --recursive --recommend-shallow -- "$OPENCV_SUBMODULE_PATH"; then
        git -C "$root" submodule update --init --recursive -- "$OPENCV_SUBMODULE_PATH"
      fi
      if ! git -C "$opencv_dir" rev-parse --verify --quiet "refs/tags/$OPENCV_EXPECTED_TAG" >/dev/null; then
        git -C "$opencv_dir" fetch --depth 1 --no-tags origin "refs/tags/$OPENCV_EXPECTED_TAG:refs/tags/$OPENCV_EXPECTED_TAG"
      fi
    fi
  fi

  is_submodule_repo || {
    printf 'prepare_opencv_submodule.sh: %s is not an initialized Git submodule (run make opencv-submodule or git submodule update --init --recursive)\n' \
      "$OPENCV_SUBMODULE_PATH" >&2
    exit 1
  }
  head_commit=$(git -C "$opencv_dir" rev-parse --verify HEAD 2>/dev/null || true)
  if [[ "$head_commit" != "$OPENCV_EXPECTED_COMMIT" ]]; then
    printf 'prepare_opencv_submodule.sh: %s HEAD is %s; expected %s (OpenCV tag %s)\n' \
      "$OPENCV_SUBMODULE_PATH" "${head_commit:-unknown}" "$OPENCV_EXPECTED_COMMIT" "$OPENCV_EXPECTED_TAG" >&2
    exit 1
  fi
  if ! git -C "$opencv_dir" diff --quiet || ! git -C "$opencv_dir" diff --cached --quiet; then
    printf 'prepare_opencv_submodule.sh: %s has uncommitted modifications; OpenCV tag %s must remain unmodified\n' \
      "$OPENCV_SUBMODULE_PATH" "$OPENCV_EXPECTED_TAG" >&2
    exit 1
  fi
  if git -C "$opencv_dir" rev-parse --verify --quiet "refs/tags/$OPENCV_EXPECTED_TAG" >/dev/null; then
    tag_object=$(git -C "$opencv_dir" rev-parse --verify "refs/tags/$OPENCV_EXPECTED_TAG")
    tag_commit=$(git -C "$opencv_dir" rev-parse --verify "refs/tags/$OPENCV_EXPECTED_TAG^{commit}")
    if [[ "$tag_object" != "$OPENCV_EXPECTED_TAG_OBJECT" || "$tag_commit" != "$OPENCV_EXPECTED_COMMIT" ]]; then
      printf 'prepare_opencv_submodule.sh: local tag %s (%s -> %s) does not match upstream OpenCV %s (%s -> %s)\n' \
        "$OPENCV_EXPECTED_TAG" "$tag_object" "$tag_commit" \
        "$OPENCV_EXPECTED_TAG" "$OPENCV_EXPECTED_TAG_OBJECT" "$OPENCV_EXPECTED_COMMIT" >&2
      exit 1
    fi
  fi
fi

for required_file in \
  "$opencv_dir/CMakeLists.txt" \
  "$opencv_dir/3rdparty/libjpeg/jpeglib.h" \
  "$opencv_dir/3rdparty/libjpeg/README"; do
  [[ -f "$required_file" ]] || {
    printf 'prepare_opencv_submodule.sh: missing required OpenCV %s file: %s\n' \
      "$OPENCV_EXPECTED_TAG" "$required_file" >&2
    exit 1
  }
done
verify_version_header "$opencv_dir/modules/core/include/opencv2/core/version.hpp"

if [[ -n "$stamp_path" ]]; then
  if (( verify_only )); then
    printf 'prepare_opencv_submodule.sh: --verify-only cannot write stamp %s\n' "$stamp_path" >&2
    exit 2
  fi
  mkdir -p -- "$(dirname -- "$stamp_path")"
  temporary_stamp=$(mktemp "$stamp_path.XXXXXX")
  trap 'rm -f -- "$temporary_stamp"' EXIT
  printf '# Verified OpenCV %s (%s)\nOPENCV_PINNED_TAG := %s\nOPENCV_PINNED_COMMIT := %s\n' \
    "$OPENCV_EXPECTED_TAG" "$OPENCV_EXPECTED_COMMIT" "$OPENCV_EXPECTED_TAG" "$OPENCV_EXPECTED_COMMIT" > "$temporary_stamp"
  if (( needs_update )) || [[ ! -f "$stamp_path" ]] || ! cmp -s -- "$temporary_stamp" "$stamp_path"; then
    mv -f -- "$temporary_stamp" "$stamp_path"
  else
    rm -f -- "$temporary_stamp"
  fi
  trap - EXIT
fi
