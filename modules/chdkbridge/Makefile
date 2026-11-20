# Copyright 2026 OpenUAS
# SPDX-License-Identifier: Apache-2.0
#
# Dual-mode Makefile: works both standalone (e.g. as an independent repository)
# and integrated inside Sensoraia (auto-detected via SENSORAIA_ROOT).

.DEFAULT_GOAL := all
CHDKBRIDGE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SENSORAIA_ROOT ?= $(if $(wildcard $(CHDKBRIDGE_DIR)/../../build-support/make/profiles.mk),$(abspath $(CHDKBRIDGE_DIR)/../..))

CC ?= cc
ROOT ?= $(if $(SENSORAIA_ROOT),$(SENSORAIA_ROOT),$(CHDKBRIDGE_DIR))
BUILD_ROOT ?= $(if $(SENSORAIA_ROOT),$(SENSORAIA_ROOT)/.build/chdkbridge,$(CHDKBRIDGE_DIR)/.build)
BIN_STAGE_DIR ?= $(ROOT)/bin
STAGE_BIN ?= 1
VERSION_BUMP ?= 0

CROSS_COMPILE ?= aarch64-linux-gnu-
STRIP ?= strip
ARM64_STRIP ?= $(CROSS_COMPILE)strip
STRIPFLAGS ?= --strip-all
ARM64_CC ?= $(CROSS_COMPILE)gcc
ARM64_SYSROOT ?=
ARM64_ARCH_FLAGS ?= -mcpu=cortex-a53

VERSION_TOOL ?= $(if $(SENSORAIA_ROOT),$(SENSORAIA_ROOT)/build-support/next_build_version.sh)
VERSION_STATE ?= $(if $(SENSORAIA_ROOT),$(SENSORAIA_ROOT)/build_versions.ini)
VERSION_LOCK ?= $(if $(SENSORAIA_ROOT),$(SENSORAIA_ROOT)/.build/build_versions.lock,$(BUILD_ROOT)/build_versions.lock)
HAS_VERSION_TOOL := $(if $(and $(VERSION_TOOL),$(VERSION_STATE),$(wildcard $(VERSION_TOOL)),$(wildcard $(VERSION_STATE))),1)

NATIVE_DIR := $(BUILD_ROOT)/native
ARM64_DIR := $(BUILD_ROOT)/arm64
NATIVE_VERSION_HEADER := $(NATIVE_DIR)/version.h
ARM64_VERSION_HEADER := $(ARM64_DIR)/version.h
NATIVE_FLAGS_STAMP := $(NATIVE_DIR)/config.stamp
ARM64_FLAGS_STAMP := $(ARM64_DIR)/architecture-flags.stamp

GIT_SHA := $(shell git -C "$(CHDKBRIDGE_DIR)" rev-parse --short=12 HEAD 2>/dev/null || git -C "$(ROOT)" rev-parse --short=12 HEAD 2>/dev/null || printf unknown)
VERSION_CPPFLAGS := -DCHDKBRIDGE_GIT_SHA=\"$(GIT_SHA)\"
CPPFLAGS += -I$(CHDKBRIDGE_DIR)
CFLAGS ?= -O2 -g0
CFLAGS += -std=c11 -Wall -Wextra -Werror
DEPFLAGS ?= -MMD -MP
LDLIBS += -ldl -lm

.PHONY: all native native-check arm64 arm64-check both verify verify-native verify-arm64 clean distclean help FORCE
all: native

ifeq ($(STAGE_BIN),1)
ifneq ($(SENSORAIA_ROOT),)
ONNXRUNTIME_STAGE_LIBS := $(BIN_STAGE_DIR)/libonnxruntime.so.1.31.0 $(BIN_STAGE_DIR)/libonnxruntime.so.1.31.0-arm64

$(BIN_STAGE_DIR)/libonnxruntime.so.1.31.0: $(SENSORAIA_ROOT)/ext/onnxruntime-linux-x64/lib/libonnxruntime.so.1.31.0
	@mkdir -p '$(@D)'
	@temporary=$$(mktemp '$@.XXXXXX'); cp -f '$<' "$$temporary" && chmod 0755 "$$temporary" && mv -f "$$temporary" '$@'

$(BIN_STAGE_DIR)/libonnxruntime.so.1.31.0-arm64: $(SENSORAIA_ROOT)/ext/onnxruntime-linux-aarch64/lib/libonnxruntime.so.1.31.0
	@mkdir -p '$(@D)'
	@temporary=$$(mktemp '$@.XXXXXX'); cp -f '$<' "$$temporary" && chmod 0755 "$$temporary" && mv -f "$$temporary" '$@'
endif

native: $(NATIVE_DIR)/chdkbridge $(BIN_STAGE_DIR)/chdkbridge $(ONNXRUNTIME_STAGE_LIBS)
arm64: $(ARM64_DIR)/chdkbridge $(BIN_STAGE_DIR)/chdkbridge-arm64 $(ONNXRUNTIME_STAGE_LIBS)

$(BIN_STAGE_DIR)/chdkbridge: $(NATIVE_DIR)/chdkbridge
	@mkdir -p '$(@D)'
	install -m 0755 '$<' '$@'

$(BIN_STAGE_DIR)/chdkbridge-arm64: $(ARM64_DIR)/chdkbridge
	@mkdir -p '$(@D)'
	install -m 0755 '$<' '$@'
else
native: $(NATIVE_DIR)/chdkbridge
arm64: $(ARM64_DIR)/chdkbridge
endif

both: native arm64

native-check:
	@command -v $(firstword $(CC)) >/dev/null 2>&1 || { \
	  printf '%s\n' 'chdkbridge: C compiler not found.' \
	    'Debian Trixie / Ubuntu: sudo apt-get update && sudo apt-get install build-essential'; exit 1; }
	@printf '#include <linux/usbdevice_fs.h>\n' | \
	  $(CC) $(CPPFLAGS) $(CFLAGS) -fsyntax-only -x c - >/dev/null 2>&1 || { \
	  printf '%s\n' 'chdkbridge: Linux USB kernel headers cannot be compiled with the selected compiler.' \
	    'Debian Trixie / Ubuntu: sudo apt-get install linux-libc-dev'; exit 1; }

arm64-check:
	@command -v $(firstword $(ARM64_CC)) >/dev/null 2>&1 || { \
	  printf '%s\n' 'chdkbridge: ARM64 cross-compiler not found.' \
	    'Debian Trixie / Ubuntu: sudo apt-get update && sudo apt-get install gcc-aarch64-linux-gnu' \
	    'Install the compiler on the development PC, not on MORA.'; exit 1; }
	@printf '#include <linux/usbdevice_fs.h>\n' | \
	  $(ARM64_CC) $(if $(ARM64_SYSROOT),--sysroot=$(ARM64_SYSROOT)) $(CPPFLAGS) $(CFLAGS) -fsyntax-only -x c - >/dev/null 2>&1 || { \
	  printf '%s\n' 'chdkbridge: Linux USB kernel headers cannot be compiled with the ARM64 compiler.' \
	    'Debian Trixie / Ubuntu: sudo apt-get install linux-libc-dev-arm64-cross gcc-aarch64-linux-gnu'; exit 1; }

$(NATIVE_FLAGS_STAMP): FORCE
	@mkdir -p '$(@D)'
	@temporary='$@.tmp.$$'; \
	  printf '%s\n' 'CC=$(CC)' 'CPPFLAGS=$(CPPFLAGS) $(VERSION_CPPFLAGS)' 'CFLAGS=$(CFLAGS)' 'LDFLAGS=$(LDFLAGS)' 'LDLIBS=$(LDLIBS)' > "$$temporary"; \
	  cmp -s "$$temporary" '$@' || mv -f "$$temporary" '$@'; rm -f "$$temporary"

$(ARM64_FLAGS_STAMP): FORCE
	@mkdir -p '$(@D)'
	@temporary='$@.tmp.$$'; \
	  printf '%s\n' 'CC=$(ARM64_CC)' 'SYSROOT=$(ARM64_SYSROOT)' 'CFLAGS=$(CFLAGS) $(ARM64_ARCH_FLAGS)' 'LDFLAGS=$(LDFLAGS)' 'LDLIBS=$(LDLIBS)' > "$$temporary"; \
	  cmp -s "$$temporary" '$@' || mv -f "$$temporary" '$@'; rm -f "$$temporary"

ifeq ($(HAS_VERSION_TOOL),1)
$(NATIVE_VERSION_HEADER) $(ARM64_VERSION_HEADER): $(VERSION_STATE) $(VERSION_TOOL)
	@VERSION_BUMP=0 bash '$(VERSION_TOOL)' '$(VERSION_STATE)' '$(VERSION_LOCK)' chdkbridge CHDKBRIDGE_VERSION '$@' -- true >/dev/null
else
$(NATIVE_VERSION_HEADER) $(ARM64_VERSION_HEADER): version.h
	@mkdir -p '$(@D)'
	@temporary='$@.tmp.$$'; \
	  printf '%s\n' '#ifndef CHDKBRIDGE_VERSION_GENERATED_H' '#define CHDKBRIDGE_VERSION_GENERATED_H' \
	    $(if $(CHDKBRIDGE_VERSION),'#define CHDKBRIDGE_VERSION "$(CHDKBRIDGE_VERSION)"') '#endif' > "$$temporary"; \
	  cmp -s "$$temporary" '$@' || mv -f "$$temporary" '$@'; rm -f "$$temporary"
endif

$(NATIVE_DIR)/chdkbridge.o: chdkbridge.c usbfs.h version.h $(NATIVE_VERSION_HEADER) $(NATIVE_FLAGS_STAMP) | native-check
	@mkdir -p '$(@D)'
	$(CC) $(CPPFLAGS) $(VERSION_CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -include '$(NATIVE_VERSION_HEADER)' -c $< -o $@

$(ARM64_DIR)/chdkbridge.o: chdkbridge.c usbfs.h version.h $(ARM64_VERSION_HEADER) $(ARM64_FLAGS_STAMP) | arm64-check
	@mkdir -p '$(@D)'
	$(ARM64_CC) $(if $(ARM64_SYSROOT),--sysroot=$(ARM64_SYSROOT)) $(CPPFLAGS) $(VERSION_CPPFLAGS) $(CFLAGS) $(ARM64_ARCH_FLAGS) $(DEPFLAGS) -include '$(ARM64_VERSION_HEADER)' -c $< -o $@

ifeq ($(and $(HAS_VERSION_TOOL),$(filter 1,$(VERSION_BUMP))),1)
$(NATIVE_DIR)/chdkbridge: chdkbridge.c usbfs.h version.h Makefile $(VERSION_TOOL) $(NATIVE_FLAGS_STAMP) FORCE | native-check
	@mkdir -p '$(@D)'
	VERSION_BUMP=1 bash '$(VERSION_TOOL)' '$(VERSION_STATE)' '$(VERSION_LOCK)' chdkbridge CHDKBRIDGE_VERSION '$(NATIVE_VERSION_HEADER)' -- \
	  $(CC) $(CPPFLAGS) $(VERSION_CPPFLAGS) $(CFLAGS) $(LDFLAGS) -include '$(NATIVE_VERSION_HEADER)' -o $@ chdkbridge.c $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@

$(ARM64_DIR)/chdkbridge: chdkbridge.c usbfs.h version.h Makefile $(VERSION_TOOL) $(ARM64_FLAGS_STAMP) FORCE | arm64-check
	@mkdir -p '$(@D)'
	VERSION_BUMP=1 bash '$(VERSION_TOOL)' '$(VERSION_STATE)' '$(VERSION_LOCK)' chdkbridge CHDKBRIDGE_VERSION '$(ARM64_VERSION_HEADER)' -- \
	  $(ARM64_CC) $(if $(ARM64_SYSROOT),--sysroot=$(ARM64_SYSROOT)) $(CPPFLAGS) $(VERSION_CPPFLAGS) $(CFLAGS) $(ARM64_ARCH_FLAGS) -include '$(ARM64_VERSION_HEADER)' -o $@ chdkbridge.c $(LDFLAGS) $(LDLIBS)
	$(ARM64_STRIP) $(STRIPFLAGS) $@
else
$(NATIVE_DIR)/chdkbridge: $(NATIVE_DIR)/chdkbridge.o
	$(CC) $(LDFLAGS) $< -o $@ $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@

$(ARM64_DIR)/chdkbridge: $(ARM64_DIR)/chdkbridge.o
	$(ARM64_CC) $(if $(ARM64_SYSROOT),--sysroot=$(ARM64_SYSROOT)) $(LDFLAGS) $< -o $@ $(LDLIBS)
	$(ARM64_STRIP) $(STRIPFLAGS) $@
endif

verify verify-native: native
	@LC_ALL=C readelf -h '$(NATIVE_DIR)/chdkbridge' | grep -Eq 'Machine:.*(Advanced Micro Devices X86-64|AArch64|ARM)'
	@file '$(NATIVE_DIR)/chdkbridge' | grep -Eq ', stripped(,|$$)'
	@! readelf -SW '$(NATIVE_DIR)/chdkbridge' | grep -Eq '\.(debug[[:alnum:]_]*|zdebug[[:alnum:]_]*)[[:space:]]'

verify-arm64: arm64
	@LC_ALL=C readelf -h '$(ARM64_DIR)/chdkbridge' | grep -Eq 'Machine:.*AArch64'
	@file '$(ARM64_DIR)/chdkbridge' | grep -Eq ', stripped(,|$$)'
	@! readelf -SW '$(ARM64_DIR)/chdkbridge' | grep -Eq '\.(debug[[:alnum:]_]*|zdebug[[:alnum:]_]*)[[:space:]]'

clean distclean:
	rm -f chdkbridge chdkbridge-arm64 '$(BIN_STAGE_DIR)/chdkbridge' '$(BIN_STAGE_DIR)/chdkbridge-arm64'
	$(if $(SENSORAIA_ROOT),,rm -rf '$(BIN_STAGE_DIR)')
	rm -rf '$(BUILD_ROOT)'

help:
	@printf '%s\n' \
	  'CHDKbridge ($(if $(SENSORAIA_ROOT),integrated in $(SENSORAIA_ROOT),standalone mode))' \
	  '  make [all|native]    Build native executable' \
	  '  make arm64           Cross-build ARM64 executable' \
	  '  make both            Build both native and ARM64 executables' \
	  '  make verify[-arm64]  Build and verify ELF architecture and stripping' \
	  '  make clean           Remove CHDKbridge build objects and binaries'

FORCE:

-include $(NATIVE_DIR)/chdkbridge.d $(ARM64_DIR)/chdkbridge.d
