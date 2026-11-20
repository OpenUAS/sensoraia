# LWIRcam variables, source-preparation, capture, verification, and clean rules.
LWIR_BUILD = $(abspath $(BUILD_DIR))/lwir
LWIR_SYSROOT = $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)')
LWIR_ARCH_FLAGS = $(if $(filter -arm64,$(OUTPUT_SUFFIX)),$(TARGET_ARCH_FLAGS) -mno-outline-atomics)
LWIR_SANITIZE ?= 0
LWIR_ENABLE_ANALYSIS ?= 0
LWIR_ENABLE_MOCK ?= 0
ifneq ($(filter-out ijg,$(LWIR_JPEG)),)
$(error LWIR_JPEG is no longer selectable; all applications use the bundled IJG JPEG codec)
endif
ifneq ($(filter-out 0 1,$(LWIR_ENABLE_ANALYSIS) $(LWIR_ENABLE_MOCK)),)
$(error LWIR_ENABLE_ANALYSIS and LWIR_ENABLE_MOCK must be 0 or 1)
endif
CPPFLAGS += -DLWIR_ENABLE_ANALYSIS=$(LWIR_ENABLE_ANALYSIS) -DLWIR_ENABLE_MOCK=$(LWIR_ENABLE_MOCK) -Imodules/lwir_cam
LWIR_FEATURE_FLAGS = -DLWIR_ENABLE_ANALYSIS=$(LWIR_ENABLE_ANALYSIS) -DLWIR_ENABLE_MOCK=$(LWIR_ENABLE_MOCK)
LWIR_SAN_FLAGS =
LWIR_SAN_LINK =
ifeq ($(LWIR_SANITIZE),1)
LWIR_SAN_FLAGS = -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie -no-pie
LWIR_SAN_LINK = -no-pie -Wl,-l:libasan.so.8 -Wl,-l:libubsan.so.1
endif
LWIR_INCLUDES = -I $(LIBJPEG) -I src -I modules/lwir_cam
LWIR_JPEG_DEPS = $(patsubst $(LIBJPEG)/%.c,$(LWIR_BUILD)/jpeg/%.o,$(LIBJPEG_CFILES))
LWIR_JPEG_LINK = $(LWIR_JPEG_DEPS)
LWIR_VENDOR_OBJECTS = $(LWIR_JPEG_DEPS)
LWIR_VENDOR_LINK = $(filter-out $(LWIR_JPEG_DEPS),$(LWIR_VENDOR_OBJECTS)) $(LWIR_JPEG_LINK)
LWIR_SSSE3_OBJECT = $(if $(filter x86_64%,$(IMX500_TRIPLET)),$(LWIR_BUILD)/lwir_ssse3.o)
LWIR_ANALYSIS_OBJECTS = $(addprefix $(LWIR_BUILD)/,lwir_geolocation.o lwir_detection.o lwir_analysis.o lwir_inference.o inference_jpeg.o) \
	$(filter-out $(BUILD_DIR)/inference/sha256fst.o $(BUILD_DIR)/inference/inference_settings.o,$(INFERENCE_OBJECTS))
LWIR_MOCK_OBJECTS = $(LWIR_BUILD)/lwir_mock.o
LWIR_DEVICE_OBJECT = $(LWIR_BUILD)/lwir_device_v4l2.o
LWIR_OBJECTS = $(addprefix $(LWIR_BUILD)/,lwir_frame.o lwir_capture.o lwir_render.o lwir_profile.o lwir_state.o) $(LWIR_DEVICE_OBJECT) $(LWIR_SSSE3_OBJECT) \
	$(if $(filter 1,$(LWIR_ENABLE_ANALYSIS)),$(LWIR_ANALYSIS_OBJECTS)) $(if $(filter 1,$(LWIR_ENABLE_MOCK)),$(LWIR_MOCK_OBJECTS))
LWIR_SUPPORT = $(addprefix $(LWIR_BUILD)/,exifite.o sha256fst.o inference_settings.o lwir_container.o lwir_image.o)
LWIR_LIBRARY_OBJECTS = $(filter-out $(LWIR_BUILD)/lwir_capture.o $(LWIR_DEVICE_OBJECT),$(LWIR_OBJECTS)) $(LWIR_SUPPORT)

$(eval $(call sensoraia-version-header-rule,$(LWIRCAM_VERSION_HEADER),lwircam,LWIRCAM_APP_VERSION))

.PHONY: lwircam lwircam-deps lwircam-build lwircam-arm64 lwircam-clean
lwircam-deps:
	$(MAKE) '$(LWIR_BUILD)/configuration.stamp'

ifeq ($(VERSION_BUMP),1)
lwircam:
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) lwircam LWIRCAM_APP_VERSION $(LWIRCAM_VERSION_HEADER) -- \
	  $(MAKE) lwircam-build LWIR_VERSION_HEADER='$(LWIRCAM_VERSION_HEADER)'
else
lwircam: lwircam-build
endif

lwircam-build: $(LWIR_OUTPUT)

lwircam-arm64:
	@test -f '$(IMX500_ARM64_SYSROOT)/usr/include/stdio.h' || { echo 'Prepare the Trixie SDK with make imx500-sdk first.' >&2; exit 1; }
	$(MAKE) lwircam OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)'

lwircam-clean:
	rm -rf .build/lwir .build/arm64/lwir bin/lwircam bin/lwircam-arm64

$(eval $(call sensoraia-module-verify-rules,lwircam,bin/lwircam,bin/lwircam-arm64))

$(LWIR_BUILD)/lwir_%.o: modules/lwir_cam/lwir_%.c modules/lwir_cam/lwir_frame.h $(LWIR_BUILD)/configuration.stamp Makefile
	@mkdir -p $(@D)
	$(CC) $(LWIR_INCLUDES) $(CPPFLAGS) $(CFLAGS) -fno-tree-loop-distribute-patterns $(LWIR_SAN_FLAGS) $(LWIR_SYSROOT) $(LWIR_ARCH_FLAGS) -D_POSIX_C_SOURCE=200809L \
	  $(LWIR_INCLUDES) -pthread -MMD -MP -c $< -o $@

$(LWIR_BUILD)/lwir_image.o $(LWIR_BUILD)/lwir_mock.o: $(LWIR_JPEG_DEPS)

$(LWIR_BUILD)/lwir_ssse3.o: modules/lwir_cam/lwir_ssse3.c modules/lwir_cam/lwir_render.h modules/lwir_cam/lwir_frame.h $(LWIR_BUILD)/configuration.stamp Makefile
	@mkdir -p $(@D)
	$(CC) $(LWIR_INCLUDES) $(CPPFLAGS) $(CFLAGS) -fno-tree-loop-distribute-patterns $(LWIR_SAN_FLAGS) $(LWIR_SYSROOT) -mssse3 -D_POSIX_C_SOURCE=200809L \
	  $(LWIR_INCLUDES) -MMD -MP -c $< -o $@

$(LWIR_BUILD)/lwir_capture.o: modules/lwir_cam/lwir_capture.c modules/lwir_cam/lwir_device.h modules/lwir_cam/lwir_image.h modules/lwir_cam/lwir_features.h $(LWIRCAM_VERSION_HEADER) modules/lwir_cam/version.h $(LWIR_BUILD)/configuration.stamp Makefile
	@mkdir -p $(@D)
	$(CC) $(LWIR_INCLUDES) $(CPPFLAGS) $(CFLAGS) -fno-tree-loop-distribute-patterns $(LWIR_SAN_FLAGS) $(LWIR_SYSROOT) $(LWIR_ARCH_FLAGS) -D_POSIX_C_SOURCE=200809L \
	  -include '$(LWIRCAM_VERSION_HEADER)' -DLWIRCAM_GIT_SHA='"$(USHER_GIT_SHA)"' \
	  $(LWIR_INCLUDES) -pthread -MMD -MP -c modules/lwir_cam/lwir_capture.c -o $@

LWIR_STAMP_CFLAGS = $(OPTFLAGS) $(LWIR_SAN_FLAGS) $(LWIR_SYSROOT) $(LWIR_ARCH_FLAGS) $(LWIR_FEATURE_FLAGS) -DIMAGE_EXIF_ATOMIC_REPLACE=$(EXIF_ATOMIC_REPLACE)
$(LWIR_BUILD)/configuration.stamp: FORCE
	$(call sensoraia-write-stamp,CC LWIR_STAMP_CFLAGS)

$(LWIR_BUILD)/jpeg/%.o: $(LIBJPEG)/%.c $(LWIR_BUILD)/configuration.stamp
	@mkdir -p $(@D)
	$(CC) $(OPTFLAGS) $(LWIR_SAN_FLAGS) $(if $(filter 1,$(LWIR_SANITIZE)),-fno-sanitize=shift-base) $(LWIR_SYSROOT) $(LWIR_ARCH_FLAGS) -I $(LIBJPEG) -MMD -MP -c $< -o $@

$(LWIR_BUILD)/exifite.o $(LWIR_BUILD)/sha256fst.o $(LWIR_BUILD)/inference_settings.o: $(LWIR_BUILD)/%.o: src/%.c $(LWIR_BUILD)/configuration.stamp
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LWIR_SAN_FLAGS) $(LWIR_SYSROOT) $(LWIR_ARCH_FLAGS) -D_POSIX_C_SOURCE=200809L -I src -MMD -MP -c $< -o $@

$(LWIR_BUILD)/inference_jpeg.o: src/inference_jpeg.c $(LWIR_BUILD)/configuration.stamp $(LWIR_JPEG_DEPS)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LWIR_SAN_FLAGS) $(LWIR_SYSROOT) $(LWIR_ARCH_FLAGS) $(LWIR_INCLUDES) -MMD -MP -c $< -o $@

$(LWIR_OUTPUT): $(LWIR_OBJECTS) $(LWIR_SUPPORT) $(LWIR_VENDOR_OBJECTS) $(VERSION_BUMP_FORCE) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CC) $(LWIR_SYSROOT) -nodefaultlibs -Wl,-Map,$(LWIR_BUILD)/link.map \
	  $(LWIR_OBJECTS) $(LWIR_SUPPORT) $(LWIR_VENDOR_LINK) $(LWIR_SAN_LINK) -pthread \
	  -Wl,-l:libc.so.6 -Wl,-l:libm.so.6 -Wl,-l:libgcc_s.so.1 \
	  $(if $(filter -arm64,$(OUTPUT_SUFFIX)),-l:ld-linux-aarch64.so.1) -o $@
	@if grep -Eq '^LOAD .*\.a$$|\.a\(' '$(LWIR_BUILD)/link.map'; then \
	  echo 'LWIR link unexpectedly used an archive library' >&2; rm -f '$@'; exit 1; fi
	@if grep -Eq 'libusb|libuvc|^LOAD .*/(usb|uvc)/' '$(LWIR_BUILD)/link.map'; then \
	  echo 'LWIR link unexpectedly included libusb/libuvc' >&2; rm -f '$@'; exit 1; fi
	$(if $(filter 1,$(LWIR_SANITIZE)),:,$(STRIP) $(STRIPFLAGS) '$@')

-include $(LWIR_OBJECTS:.o=.d) $(LWIR_SUPPORT:.o=.d) $(patsubst %.o,%.d,$(filter %.o,$(LWIR_VENDOR_OBJECTS)))