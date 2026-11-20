# Copyright 2026 OpenUAS
# SPDX-License-Identifier: Apache-2.0
#
# Shared GNU Make build rules: atomic configuration stamps, version headers,
# ELF binary verification, and uniform module target generation.

# $(call sensoraia-write-stamp,VAR1 VAR2 ...)
# Atomically updates $@ with `VAR=$(VAR)` lines only when values change.
# Uses GNU Make $(file) guarded by MAKE_DRY_RUN so arbitrary quotes in flags
# are preserved verbatim and `make -n` never touches the filesystem.
define sensoraia-write-stamp
@$(if $(MAKE_DRY_RUN),,$(shell mkdir -p '$(@D)')$(file >$@.$(_SENSORAIA_MAKE_PID).tmp,)$(foreach _var,$(1),$(file >>$@.$(_SENSORAIA_MAKE_PID).tmp,$(_var)=$($(_var)))))if cmp -s '$@.$(_SENSORAIA_MAKE_PID).tmp' '$@'; then rm -f '$@.$(_SENSORAIA_MAKE_PID).tmp'; else mv -f '$@.$(_SENSORAIA_MAKE_PID).tmp' '$@'; fi
endef

# $(call sensoraia-verify-elf,BINARY,ARCH_REGEX)
# Validates that an output executable exists, matches the target architecture,
# is stripped, and retains no debug sections.
define sensoraia-verify-elf
@test -x '$(1)' || { printf 'Missing executable: %s\n' '$(1)' >&2; exit 1; }
@LC_ALL=C readelf -h '$(1)' | grep -Eq 'Machine:.*($(2))' || { \
  printf 'Unexpected ELF architecture for %s:\n' '$(1)' >&2; \
  LC_ALL=C readelf -h '$(1)' | grep 'Machine:' >&2 || true; exit 1; }
@file '$(1)' | grep -Eq ', stripped(,|$$)' || { printf '%s is not stripped.\n' '$(1)' >&2; exit 1; }
@! readelf -SW '$(1)' | grep -Eq '\.(debug[[:alnum:]_]*|zdebug[[:alnum:]_]*)[[:space:]]' || { \
  printf '%s retains debug sections.\n' '$(1)' >&2; exit 1; }
endef

SENSORAIA_NATIVE_ELF_MACHINE := Advanced Micro Devices X86-64|AArch64|ARM
SENSORAIA_ARM64_ELF_MACHINE := AArch64

# $(eval $(call sensoraia-version-header-rule,HEADER_PATH,APP_SECTION,VERSION_MACRO))
# Materializes a version header directly from build_versions.ini without spawning
# a recursive sub-make during ordinary (VERSION_BUMP=0) builds.
define sensoraia-version-header-rule
$(1): $$(VERSION_STATE) $$(VERSION_TOOL)
	@VERSION_BUMP=0 bash '$$(VERSION_TOOL)' '$$(VERSION_STATE)' '$$(VERSION_LOCK)' $(2) $(3) '$$@' -- true >/dev/null
endef

# $(eval $(call sensoraia-module-verify-rules,TARGET_NAME,NATIVE_BIN,ARM64_BIN))
# Registers <target>-verify-native and <target>-verify-arm64 targets uniformly.
define sensoraia-module-verify-rules
.PHONY: $(1)-verify-native $(1)-verify-arm64
$(1)-verify-native: $(1)
	$$(call sensoraia-verify-elf,$(2),$$(SENSORAIA_NATIVE_ELF_MACHINE))

$(1)-verify-arm64: $(1)-arm64
	$$(call sensoraia-verify-elf,$(3),$$(SENSORAIA_ARM64_ELF_MACHINE))
endef
