# Shared native and Raspberry Pi Zero 2 W compiler/build profiles.
CC = gcc
CXX = g++
OPTFLAGS ?= -O2
ARM64_OPTFLAGS = $(if $(filter -g0,$(OPTFLAGS)),$(OPTFLAGS),$(OPTFLAGS) -g0)
AR = $(if $(OUTPUT_SUFFIX),$(CROSS_COMPILE)ar,ar)
RANLIB = $(if $(OUTPUT_SUFFIX),$(CROSS_COMPILE)ranlib,ranlib)
NM = $(if $(OUTPUT_SUFFIX),$(CROSS_COMPILE)nm,nm)
STRIP = $(if $(OUTPUT_SUFFIX),$(CROSS_COMPILE)strip,strip)
STRIPFLAGS ?= --strip-all
BUILD_DIR = .build
BIN_DIR = bin
OUTPUT_SUFFIX ?=
CROSS_COMPILE ?= aarch64-linux-gnu-
_SENSORAIA_NPROC = $(if $(_SENSORAIA_NPROC_SET),,$(eval _SENSORAIA_NPROC_VAL := $$(shell nproc))$(eval _SENSORAIA_NPROC_SET := 1))$(_SENSORAIA_NPROC_VAL)
_SENSORAIA_MAKE_PID = $(if $(_SENSORAIA_MAKE_PID_SET),,$(eval _SENSORAIA_MAKE_PID_VAL := $$(shell printf '%s' "$$$$PPID"))$(eval _SENSORAIA_MAKE_PID_SET := 1))$(_SENSORAIA_MAKE_PID_VAL)
BUILD_JOBS ?= $(or $(patsubst -j%,%,$(filter -j%,$(MAKEFLAGS))),$(_SENSORAIA_NPROC))
ifeq ($(MAKELEVEL),0)
ifeq ($(filter -j% --jobserver-auth=% --jobserver-fds=%,$(MAKEFLAGS)),)
MAKEFLAGS := -j$(BUILD_JOBS) $(MAKEFLAGS)
endif
endif
VERSION_BUMP ?= 0
ifneq ($(filter-out 0 1,$(VERSION_BUMP)),)
$(error VERSION_BUMP must be 0 or 1)
endif
VERSION_BUMP_FORCE = $(if $(filter 1,$(VERSION_BUMP)),FORCE)
MAKE_DRY_RUN = $(findstring n,$(filter-out --%,$(firstword $(MAKEFLAGS))))
# Must equal the libcamera installed on MORA; change only together with MORA's OS packages.
IMX500_LIBCAMERA_VERSION ?= 0.7.2+rpt20260817-1
IMX500_ARM64_SDK ?= $(CURDIR)/.build/imx500-arm64-trixie-sdk-$(IMX500_LIBCAMERA_VERSION)
ARM64_SYSROOT ?=
ARM64_PKG_CONFIG_LIBDIR ?= $(ARM64_SYSROOT)/usr/lib/aarch64-linux-gnu/pkgconfig:$(ARM64_SYSROOT)/usr/share/pkgconfig
ARM64_BOARD ?= pi-zero2w
ARM64_CPU_pi-zero2w = cortex-a53
ARM64_CPU_radxa-zero3w = cortex-a55
ARM64_CPU_orange-pi5b = cortex-a76.cortex-a55

ifeq ($(OUTPUT_SUFFIX),-arm64)
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
ifeq ($(ARM64_BOARD),generic)
TARGET_ARCH_FLAGS = -march=armv8-a -mtune=generic
else
ifeq ($(strip $(ARM64_CPU_$(ARM64_BOARD))),)
$(error ARM64_BOARD must be pi-zero2w, radxa-zero3w, orange-pi5b or generic)
endif
TARGET_ARCH_FLAGS = -mcpu=$(ARM64_CPU_$(ARM64_BOARD))
endif
USHER_SERIAL_DEVICE ?= /dev/serial0
USHER_MOCK_IMAGE ?= resources/mock_image_01.jpg
USHER_SODA ?= ./bin/soda
USHER_PHOTO_DIR ?= $(MORA_PHOTO_DIR)
USHER_LOCAL_PHOTO_DIR ?= $(MORA_PHOTO_DIR)
USHER_AI_CAM_COMMAND ?= rpicam-still
USHER_SPOTTERCAM_PHOTO_DIR ?= $(MORA_SPOTTERCAM_PHOTO_DIR)
USHER_SPOTTERCAM_COMMAND ?= ./bin/spottercam
USHER_SPOTTERCAM_MODEL ?= ./models/ai_cam/nn_vehicles.rpk
USHER_LOOMCAM_COMMAND ?= ./bin/loomcam
USHER_LOOMCAM_MODEL ?= ./models/ai_cam/nn_loom_v001.rpk
USHER_LOOMCAM_REFERENCE_DIR ?= $(MORA_PHOTO_DIR)/loom-reference
USHER_LWIR_CAM_COMMAND ?= ./bin/lwircam
USHER_EAR_CAM_COMMAND ?= ./bin/earcam
USHER_EAR_CAM_LOG_DIR ?= $(MORA_DEBUG_DIR)
USHER_POSE_LOG_DIR ?= $(MORA_DEBUG_DIR)
endif
USHER_EAR_CAM_COMMAND ?= ./bin/earcam
USHER_EAR_CAM_DEVICE ?= auto
USHER_EAR_CAM_LOG_DIR ?= ~/Sensoraia/diagnostics
USHER_EAR_CAM_PHOTO_DIR ?= $(USHER_PHOTO_DIR)
USHER_POSE_LOG_DIR ?=
NEAR_HORIZON_CASE_BLACK ?= 0
NEAR_HORIZON_ROLL_DEG ?= 75.0
NEAR_HORIZON_PITCH_DEG ?= 75.0
NEAR_HORIZON_COMBINED_TILT_DEG ?= 70.0

# Shared ABI, sysroot, and compile/link configuration for target SDK consumers.
IMX500_SYSROOT ?=
IMX500_NATIVE_SDK ?= $(CURDIR)/.build/imx500-native-sdk
IMX500_NATIVE_SOURCE ?= source
IMX500_USE_HOST_LIBS ?= 0
ifneq ($(IMX500_USE_HOST_LIBS),0)
ifneq ($(IMX500_USE_HOST_LIBS),1)
$(error IMX500_USE_HOST_LIBS must be 0 or 1)
endif
endif
IMX500_ARM64_SYSROOT = $(if $(IMX500_SYSROOT),$(IMX500_SYSROOT),$(IMX500_ARM64_SDK))
ifeq ($(strip $(ARM64_SYSROOT)),)
ARM64_SYSROOT = $(IMX500_ARM64_SYSROOT)
endif
IMX500_PKG_CONFIG ?= pkg-config
IMX500_BUILD_DIR = .build/imx500$(OUTPUT_SUFFIX)
IMX500_TRIPLET = $(if $(_IMX500_TRIPLET_SET$(OUTPUT_SUFFIX)),,$(eval _IMX500_TRIPLET_VAL$(OUTPUT_SUFFIX) := $$(shell $$(CXX) -dumpmachine))$(eval _IMX500_TRIPLET_SET$(OUTPUT_SUFFIX) := 1))$(_IMX500_TRIPLET_VAL$(OUTPUT_SUFFIX))
IMX500_CROSS_VERSION = $(if $(_IMX500_CROSS_VER_SET$(OUTPUT_SUFFIX)),,$(eval _IMX500_CROSS_VER_VAL$(OUTPUT_SUFFIX) := $$(shell find '$$(IMX500_SYSROOT)/usr/lib/gcc/$$(IMX500_TRIPLET)' \
 -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort -V | tail -1))$(eval _IMX500_CROSS_VER_SET$(OUTPUT_SUFFIX) := 1))$(_IMX500_CROSS_VER_VAL$(OUTPUT_SUFFIX))
IMX500_PKG_ENV = $(if $(IMX500_SYSROOT),env PKG_CONFIG_PATH= PKG_CONFIG_SYSROOT_DIR='$(IMX500_SYSROOT)' \
 PKG_CONFIG_LIBDIR='$(IMX500_SYSROOT)/usr/lib/$(IMX500_TRIPLET)/pkgconfig:$(IMX500_SYSROOT)/usr/share/pkgconfig',\
 $(if $(filter 1,$(IMX500_USE_HOST_LIBS)),,env PKG_CONFIG_PATH= PKG_CONFIG_SYSROOT_DIR= \
 PKG_CONFIG_LIBDIR='$(IMX500_NATIVE_SDK)/usr/lib/$(IMX500_TRIPLET)/pkgconfig'))
_IMX500_PKG_CFLAGS = $(if $(_IMX500_PKG_CFLAGS_SET$(OUTPUT_SUFFIX)),,$(eval _IMX500_PKG_CFLAGS_VAL$(OUTPUT_SUFFIX) := $$(shell $$(IMX500_PKG_ENV) $$(IMX500_PKG_CONFIG) --cflags libcamera 2>/dev/null))$(eval _IMX500_PKG_CFLAGS_SET$(OUTPUT_SUFFIX) := 1))$(_IMX500_PKG_CFLAGS_VAL$(OUTPUT_SUFFIX))
_IMX500_PKG_LIBS = $(if $(_IMX500_PKG_LIBS_SET$(OUTPUT_SUFFIX)),,$(eval _IMX500_PKG_LIBS_VAL$(OUTPUT_SUFFIX) := $$(shell $$(IMX500_PKG_ENV) $$(IMX500_PKG_CONFIG) --libs libcamera 2>/dev/null))$(eval _IMX500_PKG_LIBS_SET$(OUTPUT_SUFFIX) := 1))$(_IMX500_PKG_LIBS_VAL$(OUTPUT_SUFFIX))
IMX500_CPPFLAGS = -I$(LIBJPEG) $(_IMX500_PKG_CFLAGS)
IMX500_CPPFLAGS += -I$(CURDIR) -I$(CURDIR)/src -DUSHER_GIT_SHA='"$(USHER_GIT_SHA)"'
IMX500_LIBS = $(_IMX500_PKG_LIBS) -pthread
IMX500_SPAN_HEADER = $(if $(IMX500_SYSROOT),$(IMX500_SYSROOT),$(if $(filter 1,$(IMX500_USE_HOST_LIBS)),,$(IMX500_NATIVE_SDK)))/usr/include/libcamera/libcamera/base/span.h
IMX500_STD_SPAN = $(if $(_IMX500_SPAN_SET$(OUTPUT_SUFFIX)),,$(eval _IMX500_SPAN_VAL$(OUTPUT_SUFFIX) := $$(shell grep -Eq 'using Span.*std::span' '$$(IMX500_SPAN_HEADER)' 2>/dev/null && printf 1))$(eval _IMX500_SPAN_SET$(OUTPUT_SUFFIX) := 1))$(_IMX500_SPAN_VAL$(OUTPUT_SUFFIX))
_IMX500_CXX_VER = $(if $(_IMX500_CXX_VER_SET$(OUTPUT_SUFFIX)),,$(eval _IMX500_CXX_VER_VAL$(OUTPUT_SUFFIX) := $$(shell $$(CXX) -dumpversion 2>/dev/null))$(eval _IMX500_CXX_VER_SET$(OUTPUT_SUFFIX) := 1))$(_IMX500_CXX_VER_VAL$(OUTPUT_SUFFIX))
IMX500_CXXFLAGS = $(OPTFLAGS) $(TARGET_ARCH_FLAGS) -g0 -std=$(if $(IMX500_STD_SPAN),c++20,c++17) $(if $(IMX500_STD_SPAN),-DSENSORAIA_LIBCAMERA_STD_SPAN) -Wall -Wextra -Werror -Wpedantic \
	$(if $(filter 16 16.%,$(_IMX500_CXX_VER)),-fno-devirtualize-speculatively) \
 $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)' -nostdinc++ \
 -isystem '$(IMX500_SYSROOT)/usr/lib/$(IMX500_TRIPLET)/$(IMX500_CROSS_VERSION)/include' \
 -isystem '$(IMX500_SYSROOT)/usr/include/c++/$(IMX500_CROSS_VERSION)' \
 -isystem '$(IMX500_SYSROOT)/usr/include/$(IMX500_TRIPLET)/c++/$(IMX500_CROSS_VERSION)')
IMX500_GCC_LIB = $(if $(IMX500_SYSROOT),$(if $(_IMX500_GCC_LIB_SET$(OUTPUT_SUFFIX)),,$(eval _IMX500_GCC_LIB_VAL$(OUTPUT_SUFFIX) := $$(shell find '$$(IMX500_SYSROOT)/usr/lib/$$(IMX500_TRIPLET)' \
 -name libstdc++.so -printf '%h\n' 2>/dev/null | sort -V | tail -1))$(eval _IMX500_GCC_LIB_SET$(OUTPUT_SUFFIX) := 1))$(_IMX500_GCC_LIB_VAL$(OUTPUT_SUFFIX)))
IMX500_LDFLAGS = $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)' \
 -Xlinker -rpath-link -Xlinker '$(IMX500_SYSROOT)/usr/lib/$(IMX500_TRIPLET)' \
 -Xlinker -rpath-link -Xlinker '$(IMX500_SYSROOT)/lib/$(IMX500_TRIPLET)') $(if $(IMX500_GCC_LIB),-L'$(IMX500_GCC_LIB)')
ifeq ($(IMX500_SYSROOT),)
ifneq ($(IMX500_USE_HOST_LIBS),1)
IMX500_LDFLAGS += -Wl,--disable-new-dtags,-rpath,'$(IMX500_NATIVE_SDK)/usr/lib/$(IMX500_TRIPLET)'
endif
endif

# Shared configuration stamp, source identity, and generated application headers.
CONFIG_STAMP = $(BUILD_DIR)/config.stamp
USHER_GIT_SHA := $(shell git -C '$(CURDIR)' rev-parse --short=12 HEAD 2>/dev/null || printf unknown)
VERSION_STATE ?= $(CURDIR)/build_versions.ini
VERSION_LOCK = $(CURDIR)/.build/build_versions.lock
VERSION_TOOL = $(CURDIR)/build-support/next_build_version.sh
USHER_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/usher.h
SODA_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/soda.h
LWIRCAM_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/lwircam.h
EARCAM_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/earcam.h
PANGEA_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/pangea.h
CAMERA_LENS_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/camera_lens.h
LWIR_CALIBRATE_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/lwir_calibrate.h
DIPLACAM_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/diplacam.h
EAR_HEATMAP_REPLAY_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/ear_heatmap_replay.h
EAR_HEATMAP_OVERLAY_VERSION_HEADER = $(abspath $(BUILD_DIR))/versions/ear_heatmap_overlay.h

# Common USHER and application language/link policy.
CPPFLAGS = -I$(CURDIR) -I$(CURDIR)/src -I$(CURDIR)/ext -I$(LIBJPEG) \
	-D_POSIX_C_SOURCE=200809L -D_XOPEN_SOURCE=700 -DNO_VERBOSE_TAG_DATA -DNO_VERBOSE_TAG_STRINGS \
	-DUSHER_GIT_SHA='"$(USHER_GIT_SHA)"' -DIMAGE_EXIF_ATOMIC_REPLACE=$(EXIF_ATOMIC_REPLACE)
CFLAGS = $(OPTFLAGS) $(TARGET_ARCH_FLAGS) -g0 -std=c11 -Wall -Wextra -Werror -Wpedantic -Wno-expansion-to-defined \
	$(foreach name,$(USHER_STRING_OPTIONS),-DUSHER_$(name)='"$(USHER_$(name))"') \
	-Dnear_horizon_case_black=$(NEAR_HORIZON_CASE_BLACK) \
	-Dnear_horizon_roll_deg=$(NEAR_HORIZON_ROLL_DEG) \
	-Dnear_horizon_pitch_deg=$(NEAR_HORIZON_PITCH_DEG) \
	-Dnear_horizon_combined_tilt_deg=$(NEAR_HORIZON_COMBINED_TILT_DEG)
CXXFLAGS = $(OPTFLAGS) $(TARGET_ARCH_FLAGS) -g0 -Wall -Wextra -Werror -Wno-expansion-to-defined
VENDOR_CFLAGS = $(OPTFLAGS) $(TARGET_ARCH_FLAGS) -g0 -w
DEPFLAGS = -MMD -MP
LDFLAGS += -static
LDLIBS = -lpthread -lm