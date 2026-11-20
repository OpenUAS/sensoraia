# Thin root adapter. CHDKbridge owns its standalone-capable native/ARM64 build rules.
.PHONY: chdkbridge chdkbridge-arm64 chdkbridge-clean chdkbridge-verify-native chdkbridge-verify-arm64
chdkbridge: $(CHDKBRIDGE_OUTPUT)

$(CHDKBRIDGE_OUTPUT): modules/chdkbridge/chdkbridge.c modules/chdkbridge/usbfs.h modules/chdkbridge/version.h modules/chdkbridge/Makefile $(VERSION_STATE) $(VERSION_BUMP_FORCE) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(MAKE) -C modules/chdkbridge $(if $(OUTPUT_SUFFIX),arm64,native) \
	  SENSORAIA_ROOT='$(CURDIR)' STAGE_BIN=0 \
	  CROSS_COMPILE='$(CROSS_COMPILE)' \
	  VERSION_BUMP='$(VERSION_BUMP)' \
	  ARM64_SYSROOT='$(if $(wildcard $(IMX500_ARM64_SYSROOT)/usr/include/linux/usbdevice_fs.h),$(IMX500_ARM64_SYSROOT))' \
	  $(if $(OUTPUT_SUFFIX),ARM64_ARCH_FLAGS='$(TARGET_ARCH_FLAGS)') \
	  CFLAGS='$(OPTFLAGS) -g0 -std=c11 -Wall -Wextra -Werror' \
	  LDFLAGS='$(filter-out -static,$(LDFLAGS))'
	install -m 0755 .build/chdkbridge/$(if $(OUTPUT_SUFFIX),arm64,native)/chdkbridge '$(CHDKBRIDGE_OUTPUT)'

chdkbridge-arm64:
	$(MAKE) chdkbridge OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64

chdkbridge-clean:
	$(MAKE) -C modules/chdkbridge clean SENSORAIA_ROOT='$(CURDIR)'

$(eval $(call sensoraia-module-verify-rules,chdkbridge,bin/chdkbridge,bin/chdkbridge-arm64))