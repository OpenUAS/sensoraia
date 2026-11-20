#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
architecture=${1:?architecture required}
field=${2:?field CSV required}
cross_compile=${CROSS_COMPILE:-aarch64-linux-gnu-}
case "$architecture" in
  x64)
    binary="$root/bin/diplacam"
    machine='Advanced Micro Devices X86-64'
    strip_tool=strip
    readelf_tool=readelf
    nm_tool=nm
    ;;
  aarch64)
    binary="$root/bin/diplacam-arm64"
    machine=AArch64
    strip_tool="${cross_compile}strip"
    readelf_tool="${cross_compile}readelf"
    nm_tool="${cross_compile}nm"
    ;;
  *) exit 2 ;;
esac
for command_name in "$strip_tool" "$readelf_tool" "$nm_tool" sha256sum flock; do
  command -v "$command_name" >/dev/null || exit 1
done
if "$readelf_tool" -d "$binary" | grep -Ei 'NEEDED.*opencv'; then
  echo 'diplacam must not dynamically link OpenCV' >&2
  exit 1
fi
if { "$nm_tool" -C "$binary" 2>/dev/null || true; "$nm_tool" -CD "$binary" 2>/dev/null || true; } | grep -E '(^|[[:space:]])(cv::|cv[A-Z][A-Za-z0-9_]*|opencv_)'; then
  echo 'diplacam must not contain linked OpenCV symbols' >&2
  exit 1
fi
backend=$(LC_ALL=C strings -a "$binary" | sed -n 's/^DIPLA_MATH_BACKEND=//p')
case "$backend" in
  aigenlib|eigen) ;;
  *) echo "Missing or ambiguous numerical backend in $binary" >&2; exit 1 ;;
esac
runtime="$root/ext/onnxruntime-linux-$architecture"
runtime_library=${DIPLA_RUNTIME_LIBRARY:-"$runtime/lib/libonnxruntime.so.1.31.0"}
if [[ -z ${DIPLA_RUNTIME_LIBRARY:-} && ! -f "$runtime_library" && "$architecture" == aarch64 && -f "$root/bin/libonnxruntime.so.1.31.0-arm64" ]]; then
  runtime_library="$root/bin/libonnxruntime.so.1.31.0-arm64"
fi
package="$root/.build/diplacam-package-$architecture"
mkdir -p -- "$(dirname -- "$package")"
exec 9>"$package.lock"
flock 9
temporary=$(mktemp -d "$package.XXXXXXXX")
trap 'rm -rf -- "$temporary"' EXIT
for elf in "$binary" "$runtime_library"; do
  "$readelf_tool" -h "$elf" | grep -Fq "$machine" || { echo "Wrong ELF architecture: $elf" >&2; exit 1; }
  if [[ "$architecture" == aarch64 ]]; then
    latest=$("$readelf_tool" --version-info "$elf" | sed -n 's/.*Name: GLIBC_\([0-9.]*\).*/\1/p' | sort -V | tail -n 1)
    [[ "$(printf '%s\n' 2.41 "$latest" | sort -V | tail -n 1)" == 2.41 ]] || {
      echo "Not Trixie-compatible: $elf needs GLIBC_$latest" >&2; exit 1;
    }
  fi
done
install -D -m 0755 "$binary" "$temporary/bin/diplacam"
install -D -m 0755 "$runtime_library" "$temporary/bin/libonnxruntime.so.1.31.0"
"$strip_tool" --strip-unneeded "$temporary/bin/diplacam" "$temporary/bin/libonnxruntime.so.1.31.0"
bash "$root/build-support/package_models.sh" --package "$temporary" --application diplacam \
  --settings "${DIPLA_SETTINGS:-$root/modules/dipla_cam/diplacam_settings.ini}" \
  --worker-config "${DIPLA_WORKER_CONFIG:-$root/modules/dipla_cam/diplacam-example.ini}"
install -m 0644 "$field" "$temporary/share/diplacam/field.csv"
printf '%s\n' "$backend" > "$temporary/share/diplacam/math-backend.txt"
for name in LICENSE ThirdPartyNotices.txt Privacy.md VERSION_NUMBER GIT_COMMIT_ID; do
  install -D -m 0644 "$runtime/$name" "$temporary/licenses/diplacam/onnxruntime/$name"
done
install -D -m 0644 "$root/LICENSE" "$temporary/licenses/diplacam/LICENSE"
if [[ "$backend" == eigen ]]; then
  install -m 0644 /usr/share/doc/libeigen3-dev/copyright "$temporary/licenses/diplacam/eigen-copyright"
else
  install -m 0644 "$root/modules/dipla_cam/aigenlib/NOTICE.md" "$temporary/licenses/diplacam/aigenlib-NOTICE.md"
fi
install -m 0644 "$root/src/jsonbourne.h" "$temporary/licenses/diplacam/jsonbourne.h"
install -m 0644 "$root/src/jsonbourne.c" "$temporary/licenses/diplacam/jsonbourne.c"
install -m 0644 "$root/modules/dipla_cam/c-cmaes/LICENSE" "$temporary/licenses/diplacam/c-cmaes-LICENSE"
if "$readelf_tool" -d "$binary" | grep -Eq '(NEEDED.*libonnxruntime|RPATH|RUNPATH)'; then
  echo 'diplacam must explicitly load its executable-local ONNX Runtime' >&2
  exit 1
fi
"$readelf_tool" -d "$runtime_library" | grep SONAME | grep -Fq libonnxruntime.so.1
pushd "$temporary" >/dev/null
find bin share licenses -type f -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
sha256sum --check --status SHA256SUMS
popd >/dev/null
if [[ -e "$package" ]]; then rm -rf -- "$package"; fi
mv -- "$temporary" "$package"
printf 'diplacam %s (%s) package: %s\n' "$architecture" "$backend" "$package"
