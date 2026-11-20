#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
destination=$(realpath -m -- "${1:-$root/.build/imx500-arm64-trixie-sdk}")
suite=trixie
if [[ "$destination" != "$root/.build/"* || $# -gt 1 ]]; then
  echo 'prepare_imx500_sdk.sh: destination must remain under USHER .build; target release is Trixie' >&2
  exit 2
fi
camera_version=${IMX500_LIBCAMERA_VERSION:-}
if [[ ! "$camera_version" =~ ^[0-9][0-9A-Za-z.+~-]*$ ]]; then
  echo 'prepare_imx500_sdk.sh: set IMX500_LIBCAMERA_VERSION to the libcamera installed on MORA; newest packages are never selected' >&2
  exit 2
fi
for tool in apt-get curl dpkg-deb flock gpg sha256sum; do
  command -v "$tool" >/dev/null || { echo "Missing SDK tool: $tool" >&2; exit 1; }
done

mkdir -p "$root/.build/imx500-arm64-tooling"
exec 9>"$root/.build/imx500-arm64-tooling/build.lock"
flock 9
if [[ -e "$destination" ]]; then
  if [[ -f "$destination/usher-sdk-packages.sha256" &&
        "$(cat "$destination/usher-sdk-release" 2>/dev/null)" == "suite=$suite" &&
      -f "$destination/usr/lib/aarch64-linux-gnu/pkgconfig/libcamera.pc" &&
      -f "$destination/lib/aarch64-linux-gnu/libm.so.6" ]]; then
    if [[ "$(awk '$1 == "libcamera0.7" {print $2}' "$destination/usher-sdk-packages.sha256")" != "$camera_version" ]]; then
      echo "prepare_imx500_sdk.sh: existing SDK does not contain libcamera $camera_version; choose a fresh IMX500_ARM64_SDK" >&2
      exit 1
    fi
    echo "ARM64 SDK already present: $destination"
    exit 0
  fi
  echo "prepare_imx500_sdk.sh: destination already exists; choose a fresh IMX500_ARM64_SDK: $destination" >&2
  exit 1
fi

temporary=$(mktemp -d "$root/.build/imx500-sdk.XXXXXXXX")
trap 'rm -rf -- "$temporary"' EXIT
mkdir -p "$temporary/apt/lists/partial" "$temporary/apt/archives/partial" \
  "$temporary/apt/empty-sources" "$temporary/apt/empty-preferences" "$temporary/sdk"
: > "$temporary/apt/status"
: > "$temporary/apt/apt.conf"

fetch_key()
{
  local url=$1 fingerprint=$2 output=$3
  curl --fail --location --silent --show-error --retry 3 "$url" -o "$temporary/key.asc"
  if ! gpg --show-keys --with-colons "$temporary/key.asc" | grep -Fxq "fpr:::::::::$fingerprint:"; then
    echo "prepare_imx500_sdk.sh: unexpected archive signing key from $url" >&2
    exit 1
  fi
  gpg --batch --yes --dearmor -o "$output" "$temporary/key.asc"
}
fetch_key https://ftp-master.debian.org/keys/archive-key-13.asc \
  04B54C3CDCA79751B16BC6B5225629DF75B188BD "$temporary/debian.gpg"
fetch_key https://ftp-master.debian.org/keys/archive-key-13-security.asc \
  5E04A1E3223A19A20706E20F9904613D4CCE68C6 "$temporary/security.gpg"
fetch_key https://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
  CF8A1AF502A2AA2D763BAE7E82B129927FA3303E "$temporary/raspberrypi.gpg"

cat > "$temporary/apt/sources.list" <<EOF
deb [arch=arm64 signed-by=$temporary/debian.gpg] https://deb.debian.org/debian $suite main
deb [arch=arm64 signed-by=$temporary/security.gpg] https://security.debian.org/debian-security $suite-security main
deb [arch=arm64 signed-by=$temporary/raspberrypi.gpg] https://archive.raspberrypi.com/debian $suite main
EOF
cat > "$temporary/apt/preferences" <<'EOF'
Package: libcamera* libpisp*
Pin: origin archive.raspberrypi.com
Pin-Priority: 1001
EOF
options=(
  -o "Dir::Etc::main=$temporary/apt/apt.conf"
  -o "Dir::Etc::sourcelist=$temporary/apt/sources.list"
  -o "Dir::Etc::sourceparts=$temporary/apt/empty-sources"
  -o "Dir::Etc::preferences=$temporary/apt/preferences"
  -o "Dir::Etc::preferencesparts=$temporary/apt/empty-preferences"
  -o "Dir::State::status=$temporary/apt/status"
  -o "Dir::State::lists=$temporary/apt/lists"
  -o "Dir::Cache::archives=$temporary/apt/archives"
  -o "Dir::Cache::pkgcache=$temporary/apt/pkgcache.bin"
  -o "Dir::Cache::srcpkgcache=$temporary/apt/srcpkgcache.bin"
  -o 'APT::Architecture=arm64'
  -o 'APT::Architectures=arm64'
  -o 'APT::Update::Error-Mode=any'
  -o 'Acquire::Languages=none'
  -o 'Acquire::IndexTargets::deb::DEP-11::DefaultEnabled=false'
  -o "APT::Sandbox::User=$(id -un)"
)
apt-get "${options[@]}" update
apt-get "${options[@]}" --yes --download-only --no-install-recommends install \
  libstdc++-14-dev libcamera-dev libasound2-dev
# The APT index only offers the newest libcamera; replace it with the pinned target release.
rm -f -- "$temporary"/apt/archives/libcamera*.deb
for package in libcamera-dev libcamera0.7 libcamera-ipa; do
  name="${package}_${camera_version}_arm64.deb"
  expected=$(awk -v name="$name" '$2 == name {print $1}' "$root/build-support/libcamera-arm64.sha256")
  [[ -n "$expected" ]] || { echo "prepare_imx500_sdk.sh: no pinned hash for $name" >&2; exit 1; }
  curl --fail --location --silent --show-error --retry 3 \
    "https://archive.raspberrypi.com/debian/pool/main/libc/libcamera/${name//+/%2B}" -o "$temporary/apt/archives/$name"
  printf '%s  %s\n' "$expected" "$temporary/apt/archives/$name" | sha256sum --check --status ||
    { echo "prepare_imx500_sdk.sh: checksum mismatch for $name" >&2; exit 1; }
done

shopt -s nullglob
packages=("$temporary"/apt/archives/*.deb)
if (( ${#packages[@]} == 0 )); then
  echo 'prepare_imx500_sdk.sh: APT downloaded no ARM64 packages' >&2
  exit 1
fi
for package in "${packages[@]}"; do
  dpkg-deb --extract "$package" "$temporary/sdk"
  printf '%s %s %s\n' "$(dpkg-deb --field "$package" Package)" \
    "$(dpkg-deb --field "$package" Version)" "$(sha256sum "$package" | cut -d' ' -f1)" \
    >> "$temporary/sdk/usher-sdk-packages.sha256"
done
if [[ ! -e "$temporary/sdk/lib" ]]; then
  ln -s usr/lib "$temporary/sdk/lib"
fi
if [[ ! -e "$temporary/sdk/usr/lib/aarch64-linux-gnu/pkgconfig/libcamera.pc" ||
      ! -e "$temporary/sdk/usr/lib/aarch64-linux-gnu/libcamera.so" ||
  ! -e "$temporary/sdk/usr/lib/aarch64-linux-gnu/pkgconfig/alsa.pc" ||
  ! -e "$temporary/sdk/usr/lib/aarch64-linux-gnu/libasound.so" ||
  ! -e "$temporary/sdk/usr/lib/aarch64-linux-gnu/libasound.so.2" ||
      ! -e "$temporary/sdk/lib/aarch64-linux-gnu/libm.so.6" ]] ||
   ! grep -q CnnOutputTensor "$temporary/sdk/usr/include/libcamera/libcamera/control_ids.h"; then
  echo 'prepare_imx500_sdk.sh: selected packages lack Raspberry Pi IMX500 libcamera development files' >&2
  exit 1
fi
if [[ "$(awk '$1 == "libcamera0.7" {print $2}' "$temporary/sdk/usher-sdk-packages.sha256")" != "$camera_version" ]]; then
  echo "prepare_imx500_sdk.sh: APT did not select libcamera $camera_version" >&2
  exit 1
fi
printf 'suite=%s\n' "$suite" > "$temporary/sdk/usher-sdk-release"
if [[ -e "$destination" ]]; then
  echo "prepare_imx500_sdk.sh: SDK destination appeared while downloading: $destination" >&2
  exit 1
fi
mv -T -- "$temporary/sdk" "$destination"
printf 'ARM64 IMX500 camera SDK prepared: %s\n' "$destination"