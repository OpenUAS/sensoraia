# Copyright 2026 OpenUAS
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

.DEFAULT_GOAL := all
ifneq ($(filter clean clean-apps,$(MAKECMDGOALS)),)
.NOTPARALLEL:
endif
export PYTHONDONTWRITEBYTECODE := 1
export SENSORAIA_ROOT := $(CURDIR)
override OPENCV_SOURCE := $(CURDIR)/ext/opencv
LIBJPEG = $(OPENCV_SOURCE)/3rdparty/libjpeg
include build-support/make/profiles.mk
include build-support/make/rules.mk
include build-support/make/onnxruntime.mk
-include camera.conf
CAMERA ?= spottercam
ifneq ($(words $(CAMERA)),1)
$(error Select exactly one CAMERA in camera.conf or on the command line)
endif
ifneq ($(filter-out spottercam aicam lwircam chdk earcam loomcam diplacam,$(CAMERA)),)
$(error CAMERA must be spottercam, aicam, lwircam, chdk, earcam, loomcam, or diplacam)
endif
MORA_SSH_TARGET ?= air@theatre.local
MORA_INSTALL_DIR ?= /home/air/sensoraia
MORA_PHOTO_DIR ?= ~/Pictures/sensoraia
MORA_SPOTTERCAM_PHOTO_DIR ?= $(MORA_PHOTO_DIR)
MORA_DEBUG_DIR ?= ~/Sensoraia/diagnostics
MORA_PACKAGE_DIR ?= $(CURDIR)/.build/mora-package
MORA_PROFILE ?= default
MORA_INCLUDE_DIPLACAM ?= $(if $(filter default diplacam,$(MORA_PROFILE)),1,0)
USHER_OUTPUT = usher$(OUTPUT_SUFFIX)
SODA_OUTPUT = $(BIN_DIR)/soda$(OUTPUT_SUFFIX)
LWIR_OUTPUT = $(BIN_DIR)/lwircam$(OUTPUT_SUFFIX)
EAR_OUTPUT = $(BIN_DIR)/earcam$(OUTPUT_SUFFIX)
CHDKBRIDGE_OUTPUT = $(BIN_DIR)/chdkbridge$(OUTPUT_SUFFIX)
REPLAY_OUTPUT = $(BIN_DIR)/ear_heatmap_replay$(OUTPUT_SUFFIX)
EAR_HEATMAP_OVERLAY_OUTPUT = $(BIN_DIR)/ear_heatmap_overlay
LWIR_CALIBRATE_OUTPUT = $(BIN_DIR)/lwir_calibrate$(OUTPUT_SUFFIX)
LENS_OUTPUT = $(BIN_DIR)/camera_lens$(OUTPUT_SUFFIX)
LENS_PROFILES = $(BIN_DIR)/camera_lens_aicam.json $(BIN_DIR)/camera_lens_fisheye160.json
PANGEA_OUTPUT = $(BIN_DIR)/pangea$(OUTPUT_SUFFIX)

USHER_SERIAL_DEVICE ?= /dev/ttySAC0
USHER_MOCK_IMAGE ?= resources/mock_image_01.jpg
USHER_SODA ?= ./bin/soda
USHER_LOCAL_PHOTO_DIR ?= ~/Pictures/sensoraia
USHER_PHOTO_DIR ?= ~/Pictures/sensoraia

USHER_AI_CAM_PHOTO_DIR ?= $(USHER_PHOTO_DIR)
USHER_AI_CAM_COMMAND ?= rpicam-still
USHER_AI_CAM_NATIVE_COMMAND ?= ./bin/aicam
USHER_AI_CAM_WIDTH ?= 2028
USHER_AI_CAM_HEIGHT ?= 1520

USHER_SPOTTERCAM_PHOTO_DIR ?= $(USHER_PHOTO_DIR)
USHER_SPOTTERCAM_COMMAND ?= ./bin/spottercam
USHER_SPOTTERCAM_MODEL ?= ./models/ai_cam/nn_vehicles.rpk

USHER_LOOMCAM_COMMAND ?= ./bin/loomcam
USHER_LOOMCAM_MODEL ?= ./models/ai_cam/nn_loom.rpk
USHER_LOOMCAM_REFERENCE_DIR ?= $(USHER_PHOTO_DIR)/loom-reference

USHER_LWIR_CAM_PHOTO_DIR ?= $(USHER_PHOTO_DIR)
USHER_LWIR_CAM_COMMAND ?= ./bin/lwircam

USHER_CHDK_PHOTO_DIR ?= $(USHER_PHOTO_DIR)
USHER_CHDK_COMMAND ?= ./bin/chdkbridge
USHER_CHDK_ISO_MODE ?= 1600
USHER_CHDK_FLASH ?= 2
USHER_CHDK_RESOLUTION ?= 0
USHER_CHDK_QUALITY ?= 2
USHER_CHDK_DATE_STAMP ?= 0
USHER_CHDK_AF_ASSIST ?= 0
USHER_CHDK_RAW ?= 0
USHER_CHDK_FOCUS_MODE ?=
USHER_CHDK_FOCUS_DISTANCE ?=
USHER_CHDK_FOCUS_TIMEOUT ?=
USHER_CHDK_FOCUS_TOLERANCE ?=
USHER_CHDK_SHUTTER ?=
USHER_CHDK_TV96 ?=
USHER_CHDK_ISO_REAL ?=
USHER_CHDK_ISO_MARKET ?=
USHER_CHDK_SV96 ?=
USHER_CHDK_APERTURE ?=
USHER_CHDK_AV96 ?=
USHER_CHDK_ND ?=
USHER_CHDK_EXPOSURE_TIMEOUT ?=
USHER_CHDK_EXPOSURE_TOLERANCE ?=
USHER_CHDK_TIMEOUT ?= 60
USHER_CHDK_USB_TIMEOUT ?= 5000
USHER_CHDK_MAX_DOWNLOAD_MIB ?= 64
USHER_CHDK_DIRECT ?= 0
USHER_CHDK_KEEP_AWAKE ?= 1
USHER_CHDK_PORT ?=
USHER_STRING_OPTIONS = SERIAL_DEVICE MOCK_IMAGE SODA LOCAL_PHOTO_DIR \
 AI_CAM_PHOTO_DIR AI_CAM_COMMAND AI_CAM_NATIVE_COMMAND AI_CAM_WIDTH AI_CAM_HEIGHT \
 SPOTTERCAM_PHOTO_DIR SPOTTERCAM_COMMAND SPOTTERCAM_MODEL \
 LOOMCAM_COMMAND LOOMCAM_MODEL LOOMCAM_REFERENCE_DIR \
 LWIR_CAM_PHOTO_DIR LWIR_CAM_COMMAND CHDK_PHOTO_DIR CHDK_COMMAND \
 CHDK_ISO_MODE CHDK_FLASH CHDK_RESOLUTION CHDK_QUALITY CHDK_DATE_STAMP CHDK_AF_ASSIST CHDK_RAW CHDK_TIMEOUT CHDK_USB_TIMEOUT CHDK_MAX_DOWNLOAD_MIB CHDK_DIRECT CHDK_KEEP_AWAKE CHDK_PORT CHDK_BUS CHDK_DEVICE \
 CHDK_FOCUS_MODE CHDK_FOCUS_DISTANCE CHDK_FOCUS_TIMEOUT CHDK_FOCUS_TOLERANCE \
 CHDK_SHUTTER CHDK_TV96 CHDK_ISO_REAL CHDK_ISO_MARKET CHDK_SV96 CHDK_APERTURE CHDK_AV96 CHDK_ND CHDK_EXPOSURE_TIMEOUT CHDK_EXPOSURE_TOLERANCE \
 EAR_CAM_COMMAND EAR_CAM_DEVICE EAR_CAM_LOG_DIR EAR_CAM_PHOTO_DIR \
 $(if $(USHER_POSE_LOG_DIR),POSE_LOG_DIR)

EXIF_ATOMIC_REPLACE ?= 0
CAMERA_CAPTURE ?= 1

CFILES = ai_cam_pipe.c \
	 sha256fst.c \
	 imx500_pipe.c \
	 inference_settings.c \
	 camera_control.c \
	 camera_profile.c \
	 camera_backend_registry.c \
	 chdk_camera_backend.c \
	 capture_arbiter.c \
	 coverage_scheduler.c \
	 spotter_cam_pipe.c \
	 spotter_camera_backend.c \
	 lwir_spotter.c \
	 loom_cam.c \
	 loom_camera_backend.c \
	 camera_speedtest.c \
	 ear_camera_backend.c \
	 dipla_camera_backend.c \
	 ear_cam_pipe.c \
	 lwir_cam_pipe.c \
	 serial.c \
	 serial_tx.c \
	 clock_alignment.c \
	 chdk_pipe.c \
	 image_exif.c \
	 image_mock_transform.c \
	 image_lens.c \
	 local_pipe.c \
	 socket.c \
	 pose_log.c \
	 usher_protocol.c
EAR_CORE_SOURCES = modules/ear_cam/ear_fusion.c modules/ear_cam/ear_heatmap.c
EAR_CORE_OBJECTS = $(addprefix $(BUILD_DIR)/project/,$(notdir $(EAR_CORE_SOURCES:.c=.o)))
EAR_CORE_ARCHIVE = $(BUILD_DIR)/project/libearcam_core.a
HFILES = $(addprefix src/,$(CFILES:.c=.h)) $(wildcard src/*.h) \
	modules/ear_cam/earcam_core.h modules/ear_cam/ear_heatmap.h modules/spotter_cam/spotter_targets.h
PROJECT_CFILES = $(addprefix src/,$(CFILES)) $(EAR_CORE_SOURCES) modules/loom_cam/loom_return.c modules/spotter_cam/spotter_targets.c
PROJECT_OBJECTS = $(addprefix $(BUILD_DIR)/project/,$(CFILES:.c=.o)) $(EAR_CORE_ARCHIVE) \
	$(BUILD_DIR)/project/loom_return.o $(BUILD_DIR)/project/spotter_targets.o \
	$(addprefix $(BUILD_DIR)/project/,lwir_container.o lwir_frame.o lwir_render.o)
LIBJPEG_CFILES = $(filter-out $(LIBJPEG)/jmemnobs.c,$(wildcard $(LIBJPEG)/*.c))
EXIF_OBJECTS = $(BUILD_DIR)/project/exifite.o
IMAGE_EXIF_HEADERS = src/image_exif.h src/exifite.h src/sensor_capture.h src/jpeg_marker_guard.h modules/loom_cam/loom_features.h
LIBJPEG_OBJECTS = $(patsubst $(LIBJPEG)/%.c,$(BUILD_DIR)/libjpeg/%.o,$(LIBJPEG_CFILES))

SUITE_APPLICATIONS = $(USHER_OUTPUT) soda camera_lens pangea lwircam lwir_calibrate \
	earcam ear_heatmap_replay loomcam spottercam $(if $(filter 1,$(CAMERA_CAPTURE)),aicam) chdkbridge diplacam
SUITE_NATIVE_OUTPUTS = usher $(addprefix $(BIN_DIR)/,$(filter-out $(USHER_OUTPUT),$(SUITE_APPLICATIONS)))
SUITE_ARM64_OUTPUTS = $(addsuffix -arm64,$(SUITE_NATIVE_OUTPUTS))
SUITE_PREPARATION_TARGETS = $(if $(and $(filter 1,$(CAMERA_CAPTURE)),$(filter-out 1,$(IMX500_USE_HOST_LIBS))),imx500-native-sdk) $(if $(IMX500_SYSROOT),,imx500-sdk)

.PHONY: suite-prerequisites suite-check
all: suite-prerequisites
	$(MAKE) native arm64
	$(MAKE) --no-print-directory suite-check

suite-prerequisites:
	@if test -n '$(IMX500_SYSROOT)' && { \
	    test ! -r '$(IMX500_SYSROOT)/usr/include/stdio.h' || \
	    test ! -r '$(IMX500_SYSROOT)/usr/lib/aarch64-linux-gnu/pkgconfig/libcamera.pc'; }; then \
	  printf '%s\n' 'Incomplete custom IMX500_SYSROOT: $(IMX500_SYSROOT)' \
	    'Complete-suite build stopped before building applications. Supply a prepared custom sysroot or omit IMX500_SYSROOT for automatic SDK preparation.' >&2; exit 1; \
	fi
	$(if $(strip $(SUITE_PREPARATION_TARGETS)),$(MAKE) $(SUITE_PREPARATION_TARGETS),@:)
	$(if $(filter 1,$(CAMERA_CAPTURE)),$(MAKE) imx500-deps,@:)

suite-check:
	@missing=0; count=0; \
	for binary in $(SUITE_NATIVE_OUTPUTS) $(SUITE_ARM64_OUTPUTS) $(EAR_HEATMAP_OVERLAY_OUTPUT); do \
	  if test -x "$$binary"; then \
	    printf '  built %s\n' "$$binary"; count=$$((count + 1)); \
	  else \
	    printf '  MISSING executable: %s\n' "$$binary" >&2; missing=1; \
	  fi; \
	done; \
	if test "$$missing" != 0; then \
	  printf '%s\n' 'Incomplete Sensoraia suite: make all did not produce every application.' >&2; exit 1; \
	fi; \
	printf 'Complete Sensoraia suite: %s executables verified.\n' "$$count"

native: architecture-build ear_heatmap_overlay

local-earcam: USHER_EAR_CAM_DEVICE = default
local-earcam: usher earcam

architecture-build: $(ONNXRUNTIME_BIN_LIBRARIES) $(SUITE_APPLICATIONS)

$(BIN_DIR):
	@mkdir -p $@

$(BIN_DIR)/camera_lens_%.json: resources/camera_lens_%.json | $(BIN_DIR)
	@temporary=$$(mktemp '$@.XXXXXX'); install -m 0644 '$<' "$$temporary" && mv -f "$$temporary" '$@'

soda: $(SODA_OUTPUT)
camera_lens: $(LENS_OUTPUT) $(LENS_PROFILES)
pangea: $(PANGEA_OUTPUT)

include build-support/make/diplacam.mk

.PHONY: pangea-arm64
pangea-arm64:
	$(MAKE) pangea OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64

include build-support/make/chdkbridge.mk

include build-support/make/usher.mk

include build-support/make/lwircam.mk

include build-support/make/camera-modules.mk

.PHONY: arm64 mora imx500-sdk mora-package mora-deploy help

arm64: $(ONNXRUNTIME_BIN_LIBRARIES)
	$(MAKE) architecture-build BUILD_DIR=.build/arm64 OUTPUT_SUFFIX=-arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)' OPTFLAGS='$(ARM64_OPTFLAGS)'

mora: arm64

imx500-sdk:
	IMX500_LIBCAMERA_VERSION='$(IMX500_LIBCAMERA_VERSION)' bash build-support/prepare_imx500_sdk.sh '$(IMX500_ARM64_SDK)'

mora-package: mora
	$(if $(filter 1,$(MORA_INCLUDE_DIPLACAM)),$(MAKE) diplacam-package-stage,@:)
	MORA_PACKAGE_DIR='$(MORA_PACKAGE_DIR)' MORA_INSTALL_DIR='$(MORA_INSTALL_DIR)' \
	  CROSS_COMPILE='$(CROSS_COMPILE)' MORA_PROFILE='$(MORA_PROFILE)' MORA_CAMERA='$(CAMERA)' \
	  MORA_INCLUDE_DIPLACAM='$(MORA_INCLUDE_DIPLACAM)' MORA_LWIR_SPOTTER_SETTINGS='$(MORA_LWIR_SPOTTER_SETTINGS)' \
	  MORA_LOOMCAM_SETTINGS='$(MORA_LOOMCAM_SETTINGS)' \
	  bash build-support/package_mora.sh

.PHONY: mora-release
mora-release:
	$(MAKE) mora-package VERSION_BUMP=1

mora-deploy: mora-package
	MORA_PACKAGE_DIR='$(MORA_PACKAGE_DIR)' bash deploy_mora.sh '$(MORA_SSH_TARGET)' '$(MORA_INSTALL_DIR)'

help::
	@printf '%s\n' \
	  'make all                                  Prepare dependencies and build the complete PC/ARM64 suite' \
	  'make suite-check                          Verify every expected PC and ARM64 executable' \
	  'make native                               Build all PC applications' \
	  'make arm64 / make mora                    Cross-build all MORA applications on the PC; no SSH' \
	  'make clean-apps                           Remove compiled application objects/binaries while keeping SDKs' \
	  'make -C modules/<module> [arm64|clean]    Build, verify, or clean an individual module from its directory' \
	  'make imx500-sdk                           Prepare the Trixie ARM64 camera SDK on the PC' \
	  'make imx500-native-sdk                    Build the pinned-source native camera SDK locally' \
	  'JPEG                                    Bundled IJG sources only; no separate codec install' \
	  'IMX500_USE_HOST_LIBS=1                    Explicitly use ambient native camera libraries' \
	  'make local-earcam                         Build USHER and EARcam using the ALSA default device' \
	  'make earcam[-arm64]                       Build the EARcam acoustic search worker' \
		  'make ear_heatmap_replay ear_heatmap_overlay  Replay a saved sound log and add map context (PC)' \
	  'make chdkbridge[-arm64]                   Build the CHDK USB/PTP controller' \
	  'make lwircam[-arm64]                      Build LWIRcam; LWIR_ENABLE_ANALYSIS=1 / LWIR_ENABLE_MOCK=1 opt in' \
	  'ARM64_BOARD=pi-zero2w|radxa-zero3w|orange-pi5b|generic  Select target CPU tuning; Pi Zero 2 W is default' \
	  'make spottercam[-arm64]                   Build the native IMX500 vehicle detector' \
	  'make aicam[-arm64]                        Build USHER and the native photo-only camera worker' \
	  'make loomcam[-arm64]                      Build the LOOM framework and IMX500 capture process' \
	  'make spottercam loomcam CAMERA_CAPTURE=0  Build hardware-free ONNX picture applications' \
	  'ONNXRUNTIME_ROOT=/path                   Select architecture-matched ONNX SDK headers/runtime' \
	  'make camera-lens                          Build the native lens application' \
	  'make lwir_calibrate                         Build the native JPG-only LWIR calibration workshop tool' \
	  'make lwir-calibrate-arm64                  Cross-build the calibration tool on the PC' \
	  'make pangea[-arm64]                       Build the PANGEA map stitcher' \
	  'make pangea-native                        Build PANGEA optimized for this workstation CPU' \
	  'make diplacam[-arm64]                     Build Diplacam (DIPLA_MATH_BACKEND=aigenlib or eigen)' \
	  'make diplacam-aigenlib[-arm64]            Build the Eigen-free numerical library' \
	  'make diplacam-package[-native]            Package Diplacam with its matching runtime' \
	  'CAMERA=spottercam|aicam|lwircam|chdk|earcam|loomcam|diplacam  Select MORA camera (or edit camera.conf)' \
	  'make mora-package MORA_INSTALL_DIR=/path  Build and package a MORA release' \
	  'make mora-release                         Allocate product versions and build a MORA release package' \
	  'make mora-package                         Include Diplacam and all its model/data assets by default' \
	  'make mora-deploy CAMERA=lwircam           Deploy MORA configured for a specific camera' \
	  'make mora-deploy MORA_PROFILE=lwircam     Deploy an LWIR-only UART service' \
	  'make mora-deploy MORA_PROFILE=diplacam    Deploy a complete Diplacam UART service' \
	  'make mora-deploy MORA_INSTALL_DIR=/path   Package and deploy to MORA' \
	  './kickstart.sh                            Install Paparazzi simulation links and configuration' \
	  'examples/paparazzi/run_local_demo.sh      Build and run local USHER for simulation' \
	  'See docs/usher-camera-pipeline.md#building-and-deploying-usher'

include build-support/make/vision.mk

clean-apps:
	rm -f $(SUITE_NATIVE_OUTPUTS) $(SUITE_ARM64_OUTPUTS) $(EAR_HEATMAP_OVERLAY_OUTPUT) $(LENS_PROFILES)
	$(MAKE) -C modules/chdkbridge clean SENSORAIA_ROOT='$(CURDIR)'
	rm -rf .build/project .build/libjpeg .build/inference .build/lwir .build/ear_cam \
	  .build/pangea .build/imx500 .build/diplacam .build/diplacam.lock \
	  .build/diplacam-aigenlib .build/diplacam-aigenlib.lock .build/versions \
	  .build/config.stamp .build/lens-config.stamp .build/pangea-config.stamp \
	  .build/camera_lens.* .build/lwir_calibrate.* .build/lens-sha256fst.* \
	  .build/arm64/project .build/arm64/libjpeg .build/arm64/inference .build/arm64/lwir \
	  .build/arm64/ear_cam .build/arm64/pangea .build/imx500-arm64 \
	  .build/arm64/diplacam .build/arm64/diplacam.lock \
	  .build/arm64/diplacam-aigenlib .build/arm64/diplacam-aigenlib.lock .build/arm64/versions \
	  .build/arm64/config.stamp .build/arm64/lens-config.stamp .build/arm64/pangea-config.stamp \
	  .build/arm64/camera_lens.* .build/arm64/lwir_calibrate.* .build/arm64/lens-sha256fst.*

clean:
	rm -f $(SUITE_NATIVE_OUTPUTS) $(SUITE_ARM64_OUTPUTS) $(EAR_HEATMAP_OVERLAY_OUTPUT) $(LENS_PROFILES)
	$(MAKE) -C modules/chdkbridge clean SENSORAIA_ROOT='$(CURDIR)'
	rm -rf $(BIN_DIR) $(BUILD_DIR) ./.near-horizon-*

.PHONY: all native local-earcam arm64 architecture-build clean clean-apps soda camera_lens pangea loomcam ear_heatmap_replay ear_heatmap_overlay lwircam lwir_calibrate lwir-calibrate-arm64 earcam aicam chdkcam FORCE

-include $(patsubst %.o,%.d,$(filter %.o,$(PROJECT_OBJECTS))) $(EAR_CORE_OBJECTS:.o=.d) $(EXIF_OBJECTS:.o=.d) $(LIBJPEG_OBJECTS:.o=.d)

ifneq ($(wildcard ../sensoraia_dev/handover.mk),)
include ../sensoraia_dev/handover.mk
endif
