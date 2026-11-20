#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
install=$(realpath -m -- "${1:?static SDK destination required}")
sysroot=${LOOM_STATIC_SYSROOT:-}
cc=${CC:-gcc}
cxx=${CXX:-g++}
jobs=${BUILD_JOBS:-$(nproc)}
arch_flags=${LOOM_STATIC_ARCH_FLAGS:-}
triplet=$("$cxx" -dumpmachine)
camera_version=0.7.2+rpt20260817
camera_commit=6c1dd9d55573010f710c9e190a73e7e76f0d9432
camera_hash=e294b892bc3d5b2011ecd00827d2961c364501bf0be3dfbe039d4a12e34931de
yaml_commit=2c891fc7a770e8ba2fec34fc6b545c672beb37e6
yaml_hash=77987051b6f5bee8ba472f1efbe0053663a58d8d3e7a2f0f4a27d178adbf8844
sources=$(realpath -m -- "${LOOM_STATIC_SOURCE_CACHE:-$root/.build/loom-static-sources}")
if [[ "$install" != "$root/.build/"* ]]; then
  echo 'Static LOOM SDK destination must remain under Sensoraia .build' >&2
  exit 2
fi
for tool in "$cc" "$cxx" cmake meson ninja curl tar sha256sum flock python3; do
  command -v "$tool" >/dev/null || { echo "Missing static LOOM SDK tool: $tool" >&2; exit 1; }
done
build=$(dirname -- "$install")
mkdir -p "$build"
exec 9>"$build/build.lock"
flock 9
if [[ -n "$sysroot" ]]; then
  sysroot=$(realpath -e -- "$sysroot")
  [[ "$triplet" == aarch64-linux-gnu ]] || { echo 'Static camera SDK cross build must target AArch64' >&2; exit 1; }
  camera_package=$(awk '$1 == "libcamera-dev" { print $2 }' "$sysroot/usher-sdk-packages.sha256")
  case "$camera_package" in
    0.7.2+rpt20260817-*) ;;
    0.7.2+rpt20261009-*)
      camera_version=0.7.2+rpt20261009
      camera_commit=c682979b50695b93526adb1e66db33743484b63b
      camera_hash=27b10b961776ce68312df5ef8d4c374c225d93cd493a832c6415efa3e44c5b7f
      ;;
    *)
      echo "Static LOOM libcamera source pin must match the target SDK release: $camera_package" >&2
      exit 1
      ;;
  esac
fi
fingerprint=$(mktemp "$build/config.XXXXXXXX")
trap 'rm -f -- "$fingerprint"' EXIT
printf '%s\n' "$camera_commit" "$yaml_commit" "$install" "$sysroot" "$cc" "$cxx" "$arch_flags" > "$fingerprint"
sha256sum "${BASH_SOURCE[0]}" >> "$fingerprint"
"$cc" --version >> "$fingerprint"
"$cxx" --version >> "$fingerprint"
if [[ -n "$sysroot" ]]; then
  sha256sum "$sysroot/usher-sdk-packages.sha256" >> "$fingerprint"
fi
if cmp -s "$fingerprint" "$build/config" && [[ -f "$install/loom-static.sha256" ]]; then
  pushd "$install" >/dev/null
  sha256sum --check --status loom-static.sha256
  popd >/dev/null
  exit 0
fi
mkdir -p "$sources"
exec 8>"$sources/source.lock"
flock 8
fetch_source()
{
  local name=$1 repository=$2 commit=$3 expected=$4
  local archive="$sources/$name-$commit.tar.gz"
  if [[ ! -f "$archive" ]]; then
    curl --fail --location --retry 2 "https://codeload.github.com/$repository/tar.gz/$commit" -o "$archive.tmp"
    mv -- "$archive.tmp" "$archive"
  fi
  printf '%s  %s\n' "$expected" "$archive" | sha256sum --check --status
  if [[ ! -d "$sources/$name-$commit" ]]; then
    mkdir "$sources/$name-$commit"
    tar -xzf "$archive" --strip-components=1 -C "$sources/$name-$commit"
  fi
}
fetch_source libcamera raspberrypi/libcamera "$camera_commit" "$camera_hash"
fetch_source libyaml yaml/libyaml "$yaml_commit" "$yaml_hash"
camera="$sources/libcamera-$camera_commit"
yaml="$sources/libyaml-$yaml_commit"
printf '%s\n' "$camera_version" > "$camera/.tarball-version"
if grep -q "= shared_library('libcamera'," "$camera/src/libcamera/meson.build"; then
  sed -i "s/= shared_library('libcamera',/= both_libraries('libcamera',/" "$camera/src/libcamera/meson.build"
  sed -i "s/= shared_library('libcamera-base',/= both_libraries('libcamera-base',/" "$camera/src/libcamera/base/meson.build"
  sed -i 's/extern ElfW(Dyn) _DYNAMIC\[\];/extern ElfW(Dyn) _DYNAMIC[] __attribute__((weak));/;
    /for (const ElfW(Dyn) \*dyn = _DYNAMIC;/i\
\tif (!_DYNAMIC)\
\t\treturn true;\
  ' "$camera/src/libcamera/source_paths.cpp"
fi
sed -i "s/dependency('gnutls', required : false)/dependency('gnutls', required : false, method : 'pkg-config')/;
  s/dependency('libcrypto', required : false)/dependency('libcrypto', required : false, method : 'pkg-config')/" "$camera/src/libcamera/meson.build"
flock -u 8
if [[ ! -f "$build/codegen/ready" ]]; then
  python3 -m venv "$build/codegen"
  "$build/codegen/bin/python" -m pip install 'jinja2==3.1.6' 'PyYAML==6.0.3' 'ply==3.11'
  touch "$build/codegen/ready"
fi
export PATH="$build/codegen/bin:$PATH"
deps="$build/deps"
mkdir -p "$deps/lib/pkgconfig"
cmake_args=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  -DCMAKE_INSTALL_PREFIX="$deps" -DCMAKE_C_COMPILER="$cc" -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF)
if [[ -n "$arch_flags" ]]; then
  cmake_args+=("-DCMAKE_C_FLAGS_RELEASE=-O2 -DNDEBUG $arch_flags")
fi
c_args="[]"
cpp_args="[]"
link_args="[]"
machine=()
if [[ -n "$sysroot" ]]; then
  gcc_lib=$(find "$sysroot/usr/lib/gcc/$triplet" -name libstdc++.so -printf '%h\n' | sort -V | tail -1)
  version=${gcc_lib##*/}
  [[ -n "$gcc_lib" ]] || { echo 'Target SDK lacks the C++ development libraries' >&2; exit 1; }
  cmake_args+=(-DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 -DCMAKE_SYSROOT="$sysroot")
  c_args="['--sysroot=$sysroot', '$arch_flags']"
  cpp_args="['--sysroot=$sysroot', '$arch_flags', '-nostdinc++', '-isystem', '$gcc_lib/include', '-isystem', '$sysroot/usr/include/c++/$version', '-isystem', '$sysroot/usr/include/$triplet/c++/$version']"
  link_args="['--sysroot=$sysroot', '-L$gcc_lib']"
  machine=(--cross-file "$build/cross.ini")
  printf '%s\n' '[binaries]' "c = '$cc'" "cpp = '$cxx'" "ar = '${triplet}-ar'" "strip = '${triplet}-strip'" "pkg-config = 'pkg-config'" \
    '[host_machine]' "system = 'linux'" "cpu_family = 'aarch64'" "cpu = 'aarch64'" "endian = 'little'" \
    '[properties]' "pkg_config_libdir = ['$deps/lib/pkgconfig']" 'needs_exe_wrapper = true' \
    '[built-in options]' "c_args = $c_args" "cpp_args = $cpp_args" "c_link_args = $link_args" "cpp_link_args = $link_args" > "$build/cross.ini"
fi
cmake -S "$yaml" -B "$build/yaml" "${cmake_args[@]}"
cmake --build "$build/yaml"
cmake --install "$build/yaml"
printf '%s\n' "prefix=$deps" 'libdir=${prefix}/lib' 'includedir=${prefix}/include' \
  'Name: LibYAML' 'Description: Static YAML parser' 'Version: 0.2.5' \
  'Libs: -L${libdir} -lyaml' 'Cflags: -I${includedir}' > "$deps/lib/pkgconfig/yaml-0.1.pc"
if [[ "$("$cxx" -dumpversion)" == 16 || "$("$cxx" -dumpversion)" == 16.* ]]; then
  if [[ "$cpp_args" == '[]' ]]; then
    cpp_args="['-fno-devirtualize-speculatively']"
  else
    cpp_args="${cpp_args%]}, '-fno-devirtualize-speculatively']"
  fi
fi
setup=()
if [[ -f "$build/camera/build.ninja" ]]; then setup=(--wipe); fi
env PKG_CONFIG_PATH= PKG_CONFIG_SYSROOT_DIR= PKG_CONFIG_LIBDIR="$deps/lib/pkgconfig" \
  CC="$cc" CXX="$cxx" meson setup "${setup[@]}" "$build/camera" "$camera" "${machine[@]}" \
  --prefix=/usr --libdir="lib/$triplet" --libexecdir=libexec --buildtype=release \
  -Dcpp_args="$cpp_args" \
  -Ddefault_library=both -Dprefer_static=true -Dpipelines=rpi/vc4,uvcvideo -Dipas=rpi/vc4 \
  -Dcam=disabled -Dqcam=disabled -Dgstreamer=disabled -Dpycamera=disabled -Dv4l2=disabled \
  -Ddocumentation=disabled -Dtest=false -Dtracing=disabled -Dudev=disabled \
  -Dlibdw=disabled -Dlibunwind=disabled -Dsoftisp-gpu=disabled -Drpi-awb-nn=disabled
ninja -C "$build/camera" -j"$jobs" src/libcamera/libcamera.a src/libcamera/base/libcamera-base.a
mkdir -p "$install/lib" "$install/include/libcamera/libcamera/base"
cp "$build/camera/src/libcamera/libcamera.a" "$build/camera/src/libcamera/base/libcamera-base.a" "$deps/lib/libyaml.a" "$install/lib/"
if [[ -z "$sysroot" ]]; then
  ninja -C "$build/camera" -j"$jobs"
  mkdir -p "$install/lib/pkgconfig"
  find "$build/camera/src/libcamera" -maxdepth 1 -type f -name 'libcamera.so.*' -exec cp '{}' "$install/lib/" \;
  find "$build/camera/src/libcamera/base" -maxdepth 1 -type f -name 'libcamera-base.so.*' -exec cp '{}' "$install/lib/" \;
  for name in libcamera libcamera-base; do
    library=$(find "$install/lib" -maxdepth 1 -type f -name "$name.so.*" -printf '%f\n' | sort -V | tail -1)
    [[ -n "$library" ]] || { echo "Source build did not produce $name" >&2; exit 1; }
    ln -sfn "$library" "$install/lib/$name.so"
    cp "$build/camera/meson-private/$name.pc" "$install/lib/pkgconfig/"
  done
  cp "$deps/include/yaml.h" "$install/include/"
  sed -e 's|^prefix=.*|prefix=${pcfiledir}/../..|' \
    "$deps/lib/pkgconfig/yaml-0.1.pc" > "$install/lib/pkgconfig/yaml-0.1.pc"
fi
find "$camera/include/libcamera" -maxdepth 1 -type f -name '*.h' -exec cp '{}' "$install/include/libcamera/libcamera/" \;
find "$camera/include/libcamera/base" -maxdepth 1 -type f -name '*.h' -exec cp '{}' "$install/include/libcamera/libcamera/base/" \;
find "$build/camera/include/libcamera" -maxdepth 1 -type f -name '*.h' -exec cp '{}' "$install/include/libcamera/libcamera/" \;
pushd "$install" >/dev/null
find lib include -type f -print0 | sort -z | xargs -0 sha256sum > loom-static.sha256
popd >/dev/null
mv -- "$fingerprint" "$build/config"
printf 'Static LOOM camera SDK prepared: %s\n' "$install"