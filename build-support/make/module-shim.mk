# Copyright 2026 OpenUAS
# SPDX-License-Identifier: Apache-2.0
#
# Universal module-directory Makefile shim.
# Include from `modules/<name>/Makefile` after setting `MODULE_TARGET`.
# Optional: `MODULE_EXTRA_TARGETS` for module-specific subcommands forwarded as
# `$(MODULE_TARGET)-<subcommand>` to the repository root.

ifeq ($(strip $(MODULE_TARGET)),)
$(error MODULE_TARGET must be set before including module-shim.mk)
endif

.DEFAULT_GOAL := all
SENSORAIA_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
MODULE_EXTRA_TARGETS ?=

.PHONY: all native arm64 both clean distclean verify verify-native verify-arm64 help $(MODULE_EXTRA_TARGETS)

all native:
	$(MAKE) --no-print-directory -C '$(SENSORAIA_ROOT)' $(MODULE_TARGET)

arm64:
	$(MAKE) --no-print-directory -C '$(SENSORAIA_ROOT)' $(MODULE_TARGET)-arm64

both:
	$(MAKE) --no-print-directory -C '$(SENSORAIA_ROOT)' $(MODULE_TARGET) $(MODULE_TARGET)-arm64

clean distclean:
	$(MAKE) --no-print-directory -C '$(SENSORAIA_ROOT)' $(MODULE_TARGET)-clean

verify verify-native:
	$(MAKE) --no-print-directory -C '$(SENSORAIA_ROOT)' $(MODULE_TARGET)-verify-native

verify-arm64:
	$(MAKE) --no-print-directory -C '$(SENSORAIA_ROOT)' $(MODULE_TARGET)-verify-arm64

$(MODULE_EXTRA_TARGETS):
	$(MAKE) --no-print-directory -C '$(SENSORAIA_ROOT)' $(MODULE_TARGET)-$@

help:
	@printf '%s\n' \
	  'Module: $(MODULE_TARGET) (builds into $(SENSORAIA_ROOT)/.build and $(SENSORAIA_ROOT)/bin)' \
	  '  make [all|native]    Build native executable' \
	  '  make arm64           Cross-build ARM64 executable' \
	  '  make both            Build both native and ARM64 executables' \
	  '  make verify[-arm64]  Build and verify ELF architecture and stripping' \
	  '  make clean           Remove only $(MODULE_TARGET) build objects and binaries' \
	  $(foreach extra,$(MODULE_EXTRA_TARGETS),'  make $(extra)  Run $(MODULE_TARGET)-$(extra)')
