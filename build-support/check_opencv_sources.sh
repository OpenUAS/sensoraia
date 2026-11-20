#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
cache=${1:?CMake cache path required}
prefix=$(realpath "${2:?Static install prefix required}")
expected_version=4.14.0
bash "$root/build-support/prepare_opencv_submodule.sh" --verify-only "$root/ext/opencv"
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
installed_version_header="$prefix/include/opencv4/opencv2/core/version.hpp"
[[ -f "$installed_version_header" ]]
awk -v expected="$expected_version" '
    $1 == "#define" && $2 == "CV_VERSION_MAJOR"    { major = $3 }
    $1 == "#define" && $2 == "CV_VERSION_MINOR"    { minor = $3 }
    $1 == "#define" && $2 == "CV_VERSION_REVISION" { rev = $3 }
    $1 == "#define" && $2 == "CV_VERSION_STATUS"   { status = $3 }
    END {
        if (major "." minor "." rev != expected || status != "\"\"") {
            printf "Unexpected installed OpenCV version in %s: %s.%s.%s%s (expected %s)\n",
                FILENAME, major, minor, rev, status, expected > "/dev/stderr"
            exit 1
        }
    }
' "$installed_version_header"
manifest="$(dirname -- "$cache")/install_manifest.txt"
[[ -s "$manifest" ]]
for module in core imgproc flann features2d calib3d stitching; do
    archive="$prefix/lib/libopencv_$module.a"
    [[ -s "$archive" ]]
    grep -Fxq "$archive" "$manifest"
done
printf 'Verified local-source static OpenCV %s: %s\n' "$expected_version" "$cache"
