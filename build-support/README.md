# Build Support

This directory contains the scripts, pinned patches, manifests and CMake inputs
needed to build and package Sensoraia on the development PC or CI. Install the
documented PC prerequisites; the complete-suite build prepares its SDKs and
source dependencies automatically.

The root Makefile is the build entry point; `make help` lists the available
build, packaging and deployment commands. Simulation shell launchers are in
`examples/paparazzi/`.

## Complete Suite

`make` and `make all` prepare missing dependencies and build every application
for both architectures, including LOOMcam, Spottercam, PANGEA and Diplacam:

```sh
make all
```

Preparation builds the native camera library from pinned source and stages a
private native SDK. A private local libcamera tree or system `libcamera-dev`
is not required. Trixie ARM64 development packages are extracted into a private
SDK. JPEG encoding and decoding use only OpenCV's bundled IJG sources. No
system package installation, SSH or deployment happens during this build.

The build prepares dependencies before starting the application graph, and
`suite-check` verifies all 27 expected executables before success is reported.
`SUITE_APPLICATIONS` also derives the cleanup and verification inventories.

The top-level build uses the CPU count reported by `nproc` automatically, and
recursive Make calls inherit its jobserver. Explicit settings take precedence:
use `make -j4 all` to limit jobs or `make -j1 all` for a serial build. Nested
builders use the same job count by default; `BUILD_JOBS` remains an explicit
override, for example `make BUILD_JOBS=4 all`.

`make native` builds the PC applications; `make arm64` builds the ARM64
applications. Both use the same `SUITE_APPLICATIONS` list in the root Makefile.
The map-overlay utility `ear_heatmap_overlay` is an additional PC-only product.
AIcam and CHDK camera backends are compiled into USHER. The suite also builds
the native AIcam photo worker and CHDKbridge; the stock AIcam camera tool remains
an alternative supplied by the board OS.

These targets keep normal dependency-based incremental builds. Use
`make -B all` to force rebuilding without deleting prepared SDKs.
`make clean-apps` removes compiled application objects and executables while
preserving prepared SDKs and static OpenCV caches. `make clean` removes
products, SDKs and caches; `make all` recreates them.
Combined `make clean all` goals are serialized before parallel sub-builds.
Explicit SDK-preparation targets remain available, but are not extra steps
required before `make all`. Documentation generation, packaging and deployment
remain separate commands. Building Diplacam does not enable it in USHER or change
the optional `MORA_INCLUDE_DIPLACAM=1` packaging selection.

Every subdirectory under `modules/` (`ai_cam`, `chdkbridge`, `dipla_cam`,
`ear_cam`, `loom_cam`, `lwir_cam`, `pangea`, `spotter_cam`) also provides a
module `Makefile` supporting the same targets (`make`, `make native`,
`make arm64`, `make both`, `make verify[-arm64]`, `make clean`, `make help`)
without creating local `modules/*/.build` directories. Sensoraia-owned modules
include [make/module-shim.mk](make/module-shim.mk) to delegate to the root
graph, whereas `modules/chdkbridge/Makefile` is dual-mode so it builds both
integrated inside Sensoraia and standalone as an independent directory.

Model bundles are prepared by [package_models.sh](package_models.sh), shared by
the MORA and standalone Diplacam package scripts. It uses Bash, `awk` and standard
GNU file utilities, with no Python dependency. The helper validates bounded flat
settings and model descriptors, verifies graph SHA256 hashes, rejects conflicting
copies and writes package-relative model, metadata and input paths.

## Build Ownership

The root Makefile owns the product graph, shared objects, architecture profiles,
release aggregation and package/deploy ordering. Module files own compilation
only where they define a real independent build. This table is an orientation
map; the linked build files remain authoritative.

OpenCV source lives at `ext/opencv`. After cleaning, `make all` configures and
builds it directly from that location. All application JPEG code uses
`ext/opencv/3rdparty/libjpeg` directly.
There is no separate JPEG download, provider selection or image-codec library.
OpenCV's `imgcodecs` module and its JPEG provider remain disabled. LWIRcam
retains its direct-object link audit, without a JPEG-specific CMake project.

| Product or artifact | Build owner | Build system | Packaging / runtime role |
| --- | --- | --- | --- |
| `usher`, `soda`, `ear_heatmap_replay` | Root `Makefile` | GNU Make, shared project objects and bundled JPEG | `build-support/package_mora.sh` |
| `ear_heatmap_overlay` | Root `Makefile` | Host-only C utility; shared EAR heatmap core and bundled JPEG | Local tile cache; optional system `curl` for cache misses; not packaged for MORA |
| `camera_lens`, `pangea` | Root `Makefile`; PANGEA sources in `modules/pangea` | GNU Make, source-built static OpenCV and bundled IJG JPEG objects | `make camera-lens`, `make pangea[-arm64]`; MORA package |
| `lwir_calibrate` | Root `Makefile` | Host/ARM64 C++ calibration utility; OpenCV core/calib3d/imgproc closure plus bundled libjpeg, no imgcodecs | `make lwir_calibrate` or `make arm64`; included in the MORA package, not started by USHER |
| `earcam` and EAR core archive | Root `Makefile`; sources in `modules/ear_cam` | GNU Make, ALSA and shared EAR core | `make earcam[-arm64]`; MORA package |
| `lwircam` | Root `Makefile` | Native Linux V4L2 and USBFS, bundled IJG JPEG | `make lwircam[-arm64]`; MORA package |
| `chdkbridge` | Root `Makefile` delegates to dual-mode `modules/chdkbridge/Makefile` | Module GNU Make (integrated or standalone); native or Trixie ARM64 compiler | `make chdkbridge[-arm64]`; MORA package |
| `aicam` | Root `Makefile`; entry point in `modules/ai_cam/aicam.cpp` | Shared IMX500 capture and bundled IJG JPEG | `make aicam[-arm64]`; MORA package |
| `spottercam` | Root `Makefile`; source in `modules/spotter_cam` | Direct C++ compile against the selected IMX500 SDK | `make spottercam[-arm64]`; MORA package |
| `loomcam` | Root `Makefile`; application source in `modules/loom_cam` | Direct C/C++ compile plus `build_loom_static_sdk.sh` for pinned static dependencies | `make loomcam[-arm64]`; MORA package |
| `diplacam` | Root `Makefile` delegates to `modules/dipla_cam/CMakeLists.txt` | CMake; ONNX Runtime selected by root architecture profile | `make diplacam[-arm64]`; optional MORA package |
| Trixie camera/ALSA SDKs and static LOOM SDK | `build-support/prepare_imx500_sdk.sh`, `prepare_imx500_native_sdk.sh`, `build_loom_static_sdk.sh` | Signed package extraction, then CMake or Meson/Ninja as required upstream | Prepared on the PC; no target-side provisioning |
| MORA release package | Root Make orchestration; manifest and implementation in `build-support/` | Staged, validated package assembly | `make mora-package`; `make mora-deploy` is a separate activation step |

Module builds may delegate to the shared version allocator, but
should not keep a parallel version state or policy. Shared USHER objects,
architecture/sysroot profiles and release packaging remain root-owned.

The ARM64 camera SDK never takes the newest libcamera. `IMX500_LIBCAMERA_VERSION`
in `make/profiles.mk` must equal the libcamera installed on MORA, and the
matching package hashes live in `libcamera-arm64.sha256`. The SDK directory name
includes that version. Change the pin and hashes only together with MORA's OS
packages; an SDK built against a newer libcamera fails to start on MORA.

The native AIcam and Spottercam workers share `src/imx500_camera.cpp`,
`src/usher_frame_timing.c`, `src/exifite.c` and the bundled IJG objects. USHER
uses `src/imx500_pipe.c` to supervise either worker. `make all` prepares the
camera SDKs; targeted `make aicam[-arm64]` and `make spottercam[-arm64]`
expect the matching SDK to be prepared already. MORA supplies the compatible
libcamera and libcamera-base runtime libraries, their Raspberry Pi IPA/tuning
resources and IMX500 kernel driver. The measured MORA uses libcamera
`0.7.2+rpt20260817-1`. Its distribution libcamera depends on libpisp, which
itself has Boost runtime dependencies; the native camera code does not directly
use or link Boost. No additional codec library or model is needed for photos.

Source documentation and original image assets stay in the main repository;
generated documentation output is ignored. Python
started through Make does not write bytecode caches. Direct Python commands
should set `PYTHONDONTWRITEBYTECODE=1` or use `python3 -B`. Ordinary temporary
files use OS `/tmp` (or `TMPDIR`), not a project-local temporary directory.

## JSON Metadata

[JSONbourne](../src/jsonbourne.h) provides a C11 JSON reader, writer and DOM
utility with a C++11-compatible public header and a separate
[implementation](../src/jsonbourne.c). Compile the implementation once and link
its object into each consumer. Public types and constants follow yyjson-style
naming: `jsonbourne_val`, `jsonbourne_read_err`, `JSONBOURNE_TYPE_*` and
`JSONBOURNE_READ_*`. This is a naming convention, not a yyjson ABI or file-format
extension claim. Existing custom allocation, relaxed syntax flags, source
locations and deep-copy behavior remain available. Strict parsing validates
UTF-8 and surrogate pairs, preserves exact numeric tokens and duplicate member
order, and rejects trailing input.

`jsonbourne_read_buf()` uses caller-owned aligned storage without heap allocation;
`jsonbourne_read_measure()` reports its exact required size before parsing. Runtime limits
can bound input size, nesting, value count, string length and total DOM bytes.
`jsonbourne_val_write_buf()` and `jsonbourne_val_write_pretty_buf()` likewise support
caller buffers. Writer sizes include the terminating NUL. Object lookups can
require unique decoded keys; JSON Pointer and checked 64-bit integer conversion
are available without implicit string or floating-point coercion.

Diplacam keeps its 16 KiB input, 64 KiB temporary arena and 16-container nesting
limits. Its existing application-level schema and finite-number checks remain
in place. No OpenCV JSON reader is linked into Diplacam.

Ordinary optimized builds enable bounded SSE2 scanning on x86-64 and NEON on
ARM64. Cortex-A53 builds use the existing `-mcpu=cortex-a53` profile. No SSSE3,
AVX, host-native instruction requirement or fast-math flag is added. Define
`JSONBOURNE_DISABLE_SIMD=1` to select the scalar path. Inputs need no padding,
alignment or terminating NUL. The header documents buffer ownership, diagnostics,
relaxed-number output policy and the bounded exact hexadecimal conversion.

`JSONBOURNE_READER_DEPTH_LIMIT` bounds the compiled reader workspace and policy;
`JSONBOURNE_WRITER_DEPTH_LIMIT` independently bounds writing and deep copies.
Both are positive hard limits and default to 1000. Pass these options when
compiling the C implementation, not only when including the public header.
The root Diplacam build compiles the implementation with its existing limit of
16. Both files, with their notices, are packaged under `licenses/diplacam/`.
