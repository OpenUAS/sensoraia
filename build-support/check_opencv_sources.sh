#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
cache=${1:?CMake cache path required}
prefix=$(realpath "${2:?Static install prefix required}")
value() {
    awk -F= -v key="$1" '$1 ~ "^" key ":" {sub(/^[^=]*=/, ""); print}' "$cache"
}
[[ -f "$cache" ]]
[[ "$(value CMAKE_HOME_DIRECTORY)" == "$root/ext/opencv" ]]
[[ -z "$(value OPENCV_EXTRA_MODULES_PATH)" ]]
[[ "$(value BUILD_SHARED_LIBS)" == OFF ]]
[[ "$(value WITH_PTHREADS_PF)" == OFF ]]
[[ "$(value BUILD_opencv_python2)" == OFF ]]
[[ "$(value BUILD_opencv_python3)" == OFF ]]
[[ "$(value CMAKE_INSTALL_PREFIX)" == "$prefix" ]]
manifest="$(dirname -- "$cache")/install_manifest.txt"
[[ -s "$manifest" ]]
for module in core imgproc flann features2d calib3d stitching; do
    archive="$prefix/lib/libopencv_$module.a"
    [[ -s "$archive" ]]
    grep -Fxq "$archive" "$manifest"
done
printf 'Verified local-source static OpenCV: %s\n' "$cache"
