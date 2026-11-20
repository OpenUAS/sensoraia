#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
destination=$(realpath -m -- "${1:-$root/.build/imx500-native-sdk}")
source_root=${2:-source}
read -r -a compiler <<< "${CXX:-g++}"
triplet=$("${compiler[@]}" -dumpmachine)
if [[ "$destination" != "$root/.build/"* ]]; then
  echo 'Native camera SDK destination must remain under USHER .build' >&2
  exit 2
fi
for tool in pkg-config readelf ldd sha256sum flock; do
  command -v "$tool" >/dev/null || { echo "Missing native SDK tool: $tool" >&2; exit 1; }
done
mkdir -p "$root/.build/imx500-native-tooling"
exec 9>"$root/.build/imx500-native-tooling/build.lock"
flock 9
if [[ -e "$destination" ]]; then
    [[ -f "$destination/usher-native-sdk.sha256" && -f "$destination/usher-native-sdk-links" && \
      "$(<"$destination/usher-native-sdk-triplet")" == "$triplet" ]] || {
    echo 'Existing native SDK is incomplete or has the wrong architecture; select a fresh destination' >&2
    exit 1
  }
  pushd "$destination" >/dev/null
  sha256sum --check --status usher-native-sdk.sha256
  find . -type l -printf '%P %l\n' | sort | cmp -s - usher-native-sdk-links
  popd >/dev/null
  printf 'Verified existing native camera SDK: %s\n' "$destination"
  exit 0
fi
if [[ "$source_root" == source ]]; then
  static_sdk="$root/.build/loom-static/install"
  CC="${CC:-gcc}" CXX="${CXX:-g++}" BUILD_JOBS="${BUILD_JOBS:-$(nproc)}" \
    bash "$root/build-support/build_loom_static_sdk.sh" "$static_sdk"
  source_include="$static_sdk/include/libcamera"
  source_lib="$static_sdk/lib"
else
  source_root=$(realpath -e -- "$source_root")
  source_include="$source_root/usr/include/libcamera"
  source_lib="$source_root/usr/lib/$triplet"
fi
[[ -d "$source_include" && -r "$source_lib/libcamera.so" && -r "$source_lib/libcamera-base.so" ]] || {
  echo 'Source must contain libcamera headers, pkg-config files and matching readable libraries' >&2
  exit 1
}
temporary=$(mktemp -d "$root/.build/imx500-native-sdk.XXXXXXXX")
trap 'rm -rf -- "$temporary"' EXIT
sdk="$temporary/sdk"
lib="$sdk/usr/lib/$triplet"
mkdir -p "$lib/pkgconfig" "$sdk/usr/include"
cp -aL "$source_include" "$sdk/usr/include/libcamera"
metadata=(libcamera libcamera-base)
if [[ "$source_root" == source ]]; then
  metadata+=(yaml-0.1)
  cp "$source_lib/libyaml.a" "$lib/"
  cp "$source_include/../yaml.h" "$sdk/usr/include/"
fi
for name in "${metadata[@]}"; do
  sed -e 's|^prefix=.*|prefix=${pcfiledir}/../../..|' \
    -e 's|^exec_prefix=.*|exec_prefix=${prefix}|' \
    -e "s|^libdir=.*|libdir=\${prefix}/lib/$triplet|" \
    -e 's|^includedir=.*|includedir=${prefix}/include|' \
    "$source_lib/pkgconfig/$name.pc" > "$lib/pkgconfig/$name.pc"
done
libraries=("$source_lib/libcamera.so" "$source_lib/libcamera-base.so")
for input in "${libraries[@]}"; do
  ldd "$input" >> "$temporary/dependencies"
done
if grep -q 'not found' "$temporary/dependencies"; then
  echo 'Source camera library dependency closure is incomplete' >&2
  exit 1
fi
while read -r dependency; do
  libraries+=("$dependency")
done < <(awk '$2 == "=>" && $3 ~ /^\// {print $3}' "$temporary/dependencies" | sort -u)
for input in "${libraries[@]}"; do
  soname=$(readelf -d "$input" | sed -n 's/.*(SONAME).*\[\([^]]*\)\].*/\1/p')
  [[ -n "$soname" && "$soname" != */* ]] || { echo "Missing valid ELF SONAME: $input" >&2; exit 1; }
  case "$soname" in
    libc.so.*|libm.so.*|libdl.so.*|libpthread.so.*|librt.so.*) continue ;;
  esac
  if [[ ! -e "$lib/$soname" ]]; then
    cp -L "$input" "$lib/$soname"
    printf '%s %s %s\n' "$soname" "$(realpath -e -- "$input")" "$(sha256sum "$input" | cut -d' ' -f1)" >> "$sdk/usher-native-sdk-provenance"
  fi
done
for name in libcamera libcamera-base; do
  case "$name" in
    libcamera) input="$source_lib/libcamera.so" ;;
    libcamera-base) input="$source_lib/libcamera-base.so" ;;
  esac
  soname=$(readelf -d "$input" | sed -n 's/.*(SONAME).*\[\([^]]*\)\].*/\1/p')
  ln -s "$soname" "$lib/$name.so"
done
printf '%s\n' "$triplet" > "$sdk/usher-native-sdk-triplet"
find "$sdk" -type l -printf '%P %l\n' | sort > "$sdk/usher-native-sdk-links"
pushd "$sdk" >/dev/null
find . -type f ! -name usher-native-sdk.sha256 -print0 | sort -z | xargs -0 sha256sum > usher-native-sdk.sha256
sha256sum --check --status usher-native-sdk.sha256
popd >/dev/null
mkdir -p "$(dirname -- "$destination")"
mv -T -- "$sdk" "$destination"
printf 'Prepared private native camera SDK: %s\n' "$destination"