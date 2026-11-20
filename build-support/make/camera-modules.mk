# Independent capture products that share the root toolchain and SDK profiles.
AICAM_OUTPUT = $(BIN_DIR)/aicam$(OUTPUT_SUFFIX)
AICAM_VERSION_HEADER = $(abspath $(IMX500_BUILD_DIR))/aicam-version.h
SPOTTERCAM_OUTPUT = $(BIN_DIR)/spottercam$(OUTPUT_SUFFIX)
SPOTTERCAM_VERSION_HEADER = $(abspath $(IMX500_BUILD_DIR))/version.h
LOOMCAM_OUTPUT = $(BIN_DIR)/loomcam$(OUTPUT_SUFFIX)
IMX500_FRAME_TIMING_OBJECT = $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_BUILD_DIR)/usher_frame_timing.o)
IMX500_CAMERA_OBJECT = $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_BUILD_DIR)/imx500_camera.o)
IMX500_INFERENCE_CAPTURE_OBJECT = $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_BUILD_DIR)/inference_capture.o)
loomcam: $(LOOMCAM_OUTPUT)
LOOMCAM_PHOTO_OBJECT = $(IMX500_BUILD_DIR)/loomcam-photo.o
LOOM_FEATURE_OBJECT = $(IMX500_BUILD_DIR)/loom-features.o
LOOM_RETURN_OBJECT = $(IMX500_BUILD_DIR)/loom-return.o
LOOM_ONNX_FEATURE_OBJECT = $(IMX500_BUILD_DIR)/loom-onnx-features.o
LOOM_CAPTURE_OBJECT = $(IMX500_BUILD_DIR)/loom_capture.o
LOOM_IMX500_CAMERA_OBJECT = $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_BUILD_DIR)/loom_imx500_camera.o)
LOOM_INFERENCE_CAPTURE_OBJECT = $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_BUILD_DIR)/loom_inference_capture.o)
LOOMCAM_VERSION_HEADER = $(abspath $(IMX500_BUILD_DIR))/loomcam-version.h
LOOM_STATIC_SDK ?= $(CURDIR)/.build/loom-static$(OUTPUT_SUFFIX)/install
LOOMCAM_CPPFLAGS = -I$(LOOM_STATIC_SDK)/include/libcamera -I$(LIBJPEG) -I$(CURDIR) -DUSHER_GIT_SHA='"$(USHER_GIT_SHA)"'
LOOMCAM_LDFLAGS = $(IMX500_LDFLAGS) $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)' -L'$(IMX500_SYSROOT)/usr/lib/$(IMX500_TRIPLET)') $(if $(IMX500_GCC_LIB),-L'$(IMX500_GCC_LIB)')
LOOMCAM_LIBS = -Wl,--start-group $(LOOM_STATIC_SDK)/lib/libcamera.a $(LOOM_STATIC_SDK)/lib/libcamera-base.a $(LOOM_STATIC_SDK)/lib/libyaml.a -Wl,--end-group -pthread -ldl -latomic

EAR_BUILD = $(if $(OUTPUT_SUFFIX),$(abspath $(BUILD_DIR))/ear_cam,$(abspath $(BUILD_DIR))/ear_cam/native)
EAR_SOURCES = modules/ear_cam/earcam.c modules/ear_cam/ear_inference.c
EAR_OBJECTS = $(patsubst modules/ear_cam/%.c,$(EAR_BUILD)/%.o,$(EAR_SOURCES))
EAR_CONFIG_STAMP = $(EAR_BUILD)/config.stamp
EAR_PKG_CONFIG ?= pkg-config
EAR_PKG_ENV = $(if $(OUTPUT_SUFFIX),env PKG_CONFIG_PATH= PKG_CONFIG_SYSROOT_DIR='$(ARM64_SYSROOT)' PKG_CONFIG_LIBDIR='$(ARM64_PKG_CONFIG_LIBDIR)')
_EAR_ALSA_CFLAGS_NATIVE = $(if $(_EAR_ALSA_CFLAGS_SET),,$(eval _EAR_ALSA_CFLAGS_VAL := $$(shell $$(EAR_PKG_CONFIG) --cflags alsa 2>/dev/null))$(eval _EAR_ALSA_CFLAGS_SET := 1))$(_EAR_ALSA_CFLAGS_VAL)
EAR_ALSA_CPPFLAGS = $(if $(OUTPUT_SUFFIX),-isystem '$(ARM64_SYSROOT)/usr/include',$(_EAR_ALSA_CFLAGS_NATIVE))
EAR_ALSA_LDLIBS = $(if $(_EAR_ALSA_LIBS_SET$(OUTPUT_SUFFIX)),,$(eval _EAR_ALSA_LIBS_VAL$(OUTPUT_SUFFIX) := $$(shell $$(EAR_PKG_ENV) $$(EAR_PKG_CONFIG) --libs alsa 2>/dev/null))$(eval _EAR_ALSA_LIBS_SET$(OUTPUT_SUFFIX) := 1))$(_EAR_ALSA_LIBS_VAL$(OUTPUT_SUFFIX))
EAR_SYSROOT_FLAGS = $(if $(OUTPUT_SUFFIX),--sysroot='$(ARM64_SYSROOT)')
EAR_CPPFLAGS = $(EAR_ALSA_CPPFLAGS) -D_POSIX_C_SOURCE=200809L -DEARCAM_GIT_SHA='"$(USHER_GIT_SHA)"' -include '$(EARCAM_VERSION_HEADER)'
EAR_CFLAGS = $(OPTFLAGS) $(TARGET_ARCH_FLAGS) $(EAR_SYSROOT_FLAGS) -g0 -std=c11 -Wall -Wextra -Wpedantic -Werror
EAR_LDFLAGS = $(filter-out -static,$(LDFLAGS)) $(EAR_SYSROOT_FLAGS)
EAR_LDLIBS = $(EAR_ALSA_LDLIBS) -lm -pthread -ldl

$(eval $(call sensoraia-version-header-rule,$(EARCAM_VERSION_HEADER),earcam,EARCAM_VERSION))
$(eval $(call sensoraia-version-header-rule,$(AICAM_VERSION_HEADER),aicam,AICAM_VERSION))
$(eval $(call sensoraia-version-header-rule,$(SPOTTERCAM_VERSION_HEADER),spottercam,SPOTTERCAM_VERSION))
$(SPOTTERCAM_VERSION_HEADER): | $(if $(filter 1,$(CAMERA_CAPTURE)),imx500-deps)
$(eval $(call sensoraia-version-header-rule,$(LOOMCAM_VERSION_HEADER),loomcam,LOOMCAM_VERSION))

.PHONY: earcam earcam-deps earcam-build earcam-arm64 earcam-clean earcam-configure-pcm2902
ifeq ($(VERSION_BUMP),1)
earcam: $(INFERENCE_OBJECTS) | $(BIN_DIR)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) earcam EARCAM_VERSION $(EARCAM_VERSION_HEADER) -- \
	  $(MAKE) earcam-build TARGET='$(abspath $(EAR_OUTPUT))'
else
earcam: earcam-build
endif

earcam-deps:
	@$(EAR_PKG_ENV) $(EAR_PKG_CONFIG) --exists alsa || { \
	  echo 'ALSA development files not found by $(EAR_PKG_CONFIG).' >&2; \
	  echo 'Install pkg-config and libasound2-dev (or prepare the ARM64 SDK / set ARM64_SYSROOT) for this target.' >&2; \
	  exit 1; \
	}

earcam-build: earcam-deps $(EAR_OUTPUT)

earcam-arm64:
	@test -f '$(ARM64_SYSROOT)/usr/include/alsa/asoundlib.h' || test -f /usr/include/alsa/asoundlib.h || { echo 'Prepare the Trixie SDK with make imx500-sdk (or set ARM64_SYSROOT) first.' >&2; exit 1; }
	$(MAKE) earcam OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)'

$(EAR_OUTPUT): $(EAR_OBJECTS) $(INFERENCE_OBJECTS) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	@mkdir -p $(@D)
	TARGET='$(abspath $@)' $(CC) $(EAR_LDFLAGS) $(EAR_OBJECTS) $(INFERENCE_OBJECTS) -o $@ $(EAR_LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@

$(EAR_BUILD)/%.o: modules/ear_cam/%.c modules/ear_cam/ear_inference.h modules/ear_cam/version.h $(EARCAM_VERSION_HEADER) $(EAR_CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(EAR_CPPFLAGS) $(EAR_CFLAGS) $(DEPFLAGS) -MF $@.d -c $< -o $@

$(EAR_CONFIG_STAMP): FORCE
	$(call sensoraia-write-stamp,OUTPUT_SUFFIX CC OPTFLAGS EAR_CPPFLAGS EAR_CFLAGS EAR_LDFLAGS EAR_LDLIBS)

earcam-configure-pcm2902:
	amixer -c Device set 'Auto Gain Control' off
	amixer -c Device set Mic 14 cap

earcam-clean:
	rm -rf .build/ear_cam .build/arm64/ear_cam bin/earcam bin/earcam-arm64

$(eval $(call sensoraia-module-verify-rules,earcam,bin/earcam,bin/earcam-arm64))

-include $(addsuffix .d,$(EAR_OBJECTS))

chdkcam: $(USHER_OUTPUT)
	@printf '%s\n' '$@ backend is compiled into $(USHER_OUTPUT); its camera driver is external.'

.PHONY: aicam aicam-arm64 aicam-clean spottercam spottercam-arm64 spottercam-clean loomcam-clean imx500-deps imx500-native-sdk
ifeq ($(CAMERA_CAPTURE),1)
aicam: $(USHER_OUTPUT) imx500-deps $(AICAM_OUTPUT)
else
aicam:
	@echo 'AIcam photo capture requires CAMERA_CAPTURE=1 and a prepared libcamera SDK.' >&2; exit 1
endif

aicam-arm64:
	$(MAKE) aicam OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)' \
	  CXX='$(CROSS_COMPILE)g++' \
	  OPTFLAGS='$(OPTFLAGS) -g0'

aicam-clean:
	rm -f .build/imx500/aicam.o .build/imx500/aicam.d .build/imx500-arm64/aicam.o .build/imx500-arm64/aicam.d bin/aicam bin/aicam-arm64

$(eval $(call sensoraia-module-verify-rules,aicam,bin/aicam,bin/aicam-arm64))

imx500-native-sdk:
	CC='$(CC)' CXX='$(CXX)' BUILD_JOBS='$(BUILD_JOBS)' \
	  bash build-support/prepare_imx500_native_sdk.sh '$(IMX500_NATIVE_SDK)' '$(IMX500_NATIVE_SOURCE)'

spottercam: imx500-deps $(SPOTTERCAM_OUTPUT)

spottercam-arm64:
	$(MAKE) spottercam OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)' \
	  CXX='$(CROSS_COMPILE)g++' \
	  OPTFLAGS='$(OPTFLAGS) -g0'

spottercam-clean:
	rm -f .build/imx500/spotter_targets.o .build/imx500/spotter_targets.d .build/imx500/spotter_cam.d \
	  .build/imx500-arm64/spotter_targets.o .build/imx500-arm64/spotter_targets.d .build/imx500-arm64/spotter_cam.d \
	  bin/spottercam bin/spottercam-arm64

$(eval $(call sensoraia-module-verify-rules,spottercam,bin/spottercam,bin/spottercam-arm64))

imx500-deps:
ifeq ($(IMX500_SYSROOT),)
ifneq ($(IMX500_USE_HOST_LIBS),1)
	@test -f '$(IMX500_NATIVE_SDK)/usher-native-sdk.sha256' && \
	  test "$$(cat '$(IMX500_NATIVE_SDK)/usher-native-sdk-triplet')" = '$(IMX500_TRIPLET)' || { \
	  echo 'Prepare a private native SDK with make imx500-native-sdk (or explicitly opt in to IMX500_USE_HOST_LIBS=1).' >&2; exit 1; }
	@cd '$(IMX500_NATIVE_SDK)' && sha256sum --check --status usher-native-sdk.sha256
	@cd '$(IMX500_NATIVE_SDK)' && find . -type l -printf '%P %l\n' | sort | cmp -s - usher-native-sdk-links || { \
	  echo 'Private native SDK symlinks changed; do not fall back to host libraries.' >&2; exit 1; }
endif
endif
	@$(IMX500_PKG_ENV) $(IMX500_PKG_CONFIG) --exists libcamera || { \
	  echo 'Prepare the camera SDK with make all, or select a matching IMX500_SYSROOT.' >&2; exit 1; }

$(IMX500_BUILD_DIR):
	@mkdir -p $@

_IMX500_NATIVE_SDK_SHA = $(if $(_IMX500_NSDK_SHA_SET),,$(eval _IMX500_NSDK_SHA_VAL := $$(shell sha256sum '$$(IMX500_NATIVE_SDK)/usher-native-sdk.sha256' 2>/dev/null))$(eval _IMX500_NSDK_SHA_SET := 1))$(_IMX500_NSDK_SHA_VAL)

$(IMX500_BUILD_DIR)/config.stamp: FORCE | $(IMX500_BUILD_DIR)
	$(call sensoraia-write-stamp,CC CXX CAMERA_CAPTURE IMX500_CPPFLAGS IMX500_CXXFLAGS IMX500_LDFLAGS IMX500_LIBS LOOMCAM_CPPFLAGS LOOMCAM_LDFLAGS LOOMCAM_LIBS _IMX500_NATIVE_SDK_SHA)

$(IMX500_BUILD_DIR)/usher_frame_timing.o: src/usher_frame_timing.c src/usher_frame_timing.h src/boot_id.h src/sensor_capture.h src/exifite.h $(IMX500_BUILD_DIR)/config.stamp
	$(CC) $(CPPFLAGS) $(CFLAGS) $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)') $(DEPFLAGS) -c $< -o $@

$(IMX500_BUILD_DIR)/imx500_camera.o: src/imx500_camera.cpp src/imx500_camera.hpp src/usher_frame_timing.h src/sensor_capture.h src/exifite.h $(IMX500_BUILD_DIR)/config.stamp | imx500-deps
	$(CXX) $(IMX500_CPPFLAGS) $(IMX500_CXXFLAGS) $(DEPFLAGS) -c $< -o $@

$(IMX500_BUILD_DIR)/inference_capture.o: src/inference_capture.cpp src/inference_capture.hpp src/imx500_camera.hpp $(IMX500_BUILD_DIR)/config.stamp | imx500-deps
	$(CXX) $(IMX500_CPPFLAGS) $(IMX500_CXXFLAGS) -Isrc $(DEPFLAGS) -c $< -o $@

$(IMX500_BUILD_DIR)/aicam.o: modules/ai_cam/aicam.cpp src/version.h $(AICAM_VERSION_HEADER) $(IMX500_BUILD_DIR)/config.stamp | imx500-deps
	$(CXX) $(IMX500_CPPFLAGS) $(IMX500_CXXFLAGS) -include $(AICAM_VERSION_HEADER) $(DEPFLAGS) -c $< -o $@

ifeq ($(VERSION_BUMP),1)
$(AICAM_OUTPUT): modules/ai_cam/aicam.cpp src/version.h $(IMX500_CAMERA_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) $(IMX500_BUILD_DIR)/config.stamp $(VERSION_TOOL) FORCE | imx500-deps $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) aicam AICAM_VERSION $(AICAM_VERSION_HEADER) -- \
	  $(CXX) $(IMX500_CPPFLAGS) $(IMX500_CXXFLAGS) -include $(AICAM_VERSION_HEADER) $< $(IMX500_CAMERA_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(IMX500_LDFLAGS) $(IMX500_LIBS) -lm
	$(STRIP) $(STRIPFLAGS) $@
else
$(AICAM_OUTPUT): $(IMX500_BUILD_DIR)/aicam.o $(IMX500_CAMERA_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) | imx500-deps $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CXX) $(IMX500_BUILD_DIR)/aicam.o $(IMX500_CAMERA_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(IMX500_LDFLAGS) $(IMX500_LIBS) -lm
	$(STRIP) $(STRIPFLAGS) $@
endif

$(IMX500_BUILD_DIR)/spotter_targets.o: modules/spotter_cam/spotter_targets.c modules/spotter_cam/spotter_targets.h | $(IMX500_BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -std=c11 $(DEPFLAGS) -c $< -o $@

ifeq ($(VERSION_BUMP),1)
$(SPOTTERCAM_OUTPUT): modules/spotter_cam/spotter_cam.cpp modules/spotter_cam/spotter_targets.h $(IMX500_BUILD_DIR)/spotter_targets.o $(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o build-support/make/camera-modules.mk src/version.h src/usher_frame_timing.h $(IMX500_CAMERA_OBJECT) $(IMX500_INFERENCE_CAPTURE_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) $(LIBJPEG_OBJECTS) $(IMX500_BUILD_DIR)/config.stamp $(VERSION_TOOL) FORCE | $(if $(filter 1,$(CAMERA_CAPTURE)),imx500-deps) $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) spottercam SPOTTERCAM_VERSION $(SPOTTERCAM_VERSION_HEADER) -- \
	  $(CXX) $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_CPPFLAGS) $(IMX500_CXXFLAGS),$(CXXFLAGS) -DSENSORAIA_INFERENCE_ONLY) -Isrc -include $(SPOTTERCAM_VERSION_HEADER) $(DEPFLAGS) -MF $(IMX500_BUILD_DIR)/spotter_cam.d $< $(IMX500_BUILD_DIR)/spotter_targets.o $(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_CAMERA_OBJECT) $(IMX500_INFERENCE_CAPTURE_OBJECT) $(IMX500_FRAME_TIMING_OBJECT)) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_LDFLAGS) $(IMX500_LIBS),$(filter-out -static,$(LDFLAGS))) -pthread -ldl -lm
	$(STRIP) $(STRIPFLAGS) $@
else
$(SPOTTERCAM_OUTPUT): modules/spotter_cam/spotter_cam.cpp modules/spotter_cam/spotter_targets.h $(SPOTTERCAM_VERSION_HEADER) $(IMX500_BUILD_DIR)/spotter_targets.o $(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o build-support/make/camera-modules.mk src/version.h src/usher_frame_timing.h $(IMX500_CAMERA_OBJECT) $(IMX500_INFERENCE_CAPTURE_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) $(LIBJPEG_OBJECTS) $(IMX500_BUILD_DIR)/config.stamp | $(if $(filter 1,$(CAMERA_CAPTURE)),imx500-deps) $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CXX) $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_CPPFLAGS) $(IMX500_CXXFLAGS),$(CXXFLAGS) -DSENSORAIA_INFERENCE_ONLY) -Isrc -include $(SPOTTERCAM_VERSION_HEADER) $(DEPFLAGS) -MF $(IMX500_BUILD_DIR)/spotter_cam.d $< $(IMX500_BUILD_DIR)/spotter_targets.o $(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_CAMERA_OBJECT) $(IMX500_INFERENCE_CAPTURE_OBJECT) $(IMX500_FRAME_TIMING_OBJECT)) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(if $(filter 1,$(CAMERA_CAPTURE)),$(IMX500_LDFLAGS) $(IMX500_LIBS),$(filter-out -static,$(LDFLAGS))) -pthread -ldl -lm
	$(STRIP) $(STRIPFLAGS) $@
endif

.PHONY: loom-static-deps
loom-static-deps: $(LOOM_STATIC_SDK)/loom-static.sha256

$(LOOM_STATIC_SDK)/loom-static.sha256: build-support/build_loom_static_sdk.sh FORCE
	CC='$(CC)' CXX='$(CXX)' BUILD_JOBS='$(BUILD_JOBS)' LOOM_STATIC_SYSROOT='$(IMX500_SYSROOT)' \
	  LOOM_STATIC_ARCH_FLAGS='$(TARGET_ARCH_FLAGS)' \
	  bash build-support/build_loom_static_sdk.sh '$(LOOM_STATIC_SDK)'

ifeq ($(OUTPUT_SUFFIX),)
.PHONY: loomcam-arm64
loomcam-arm64:
	$(MAKE) loomcam OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)' \
	  CC='$(CROSS_COMPILE)gcc' CXX='$(CROSS_COMPILE)g++' OPTFLAGS='$(OPTFLAGS) -g0'
endif

loomcam-clean:
	rm -f .build/imx500/loom* .build/imx500-arm64/loom* bin/loomcam bin/loomcam-arm64

$(eval $(call sensoraia-module-verify-rules,loomcam,bin/loomcam,bin/loomcam-arm64))

$(LOOMCAM_PHOTO_OBJECT): modules/loom_cam/loomcam.c modules/loom_cam/loom_return.h modules/loom_cam/loom_features.h src/camera_backend.h src/usher_protocol.h src/inference_settings.h src/inference_model.h src/exifite.h $(IMX500_BUILD_DIR)/config.stamp
	$(CC) -I$(CURDIR)/src -D_POSIX_C_SOURCE=200809L -DLOOMCAM_EMBEDDED -std=c11 -Wall -Wextra -Werror -Wpedantic $(OPTFLAGS) $(TARGET_ARCH_FLAGS) -g0 \
	  $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)') $(DEPFLAGS) -c $< -o $@

$(LOOM_FEATURE_OBJECT): modules/loom_cam/loom_features.c modules/loom_cam/loom_features.h src/imx500_tensor.h $(IMX500_BUILD_DIR)/config.stamp
	$(CC) -I$(CURDIR)/src -D_POSIX_C_SOURCE=200809L -std=c11 -Wall -Wextra -Werror -Wpedantic $(OPTFLAGS) $(TARGET_ARCH_FLAGS) -g0 \
	  $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)') $(DEPFLAGS) -c $< -o $@

$(LOOM_ONNX_FEATURE_OBJECT): modules/loom_cam/loom_onnx_features.c modules/loom_cam/loom_features.h src/inference_model.h $(IMX500_BUILD_DIR)/config.stamp
	$(CC) $(CPPFLAGS) $(CFLAGS) $(TARGET_ARCH_FLAGS) $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)') -Isrc $(DEPFLAGS) -c $< -o $@

$(LOOM_RETURN_OBJECT): modules/loom_cam/loom_return.c modules/loom_cam/loom_return.h src/bounded_ini.h $(IMX500_BUILD_DIR)/config.stamp build-support/make/camera-modules.mk
	$(CC) $(CPPFLAGS) $(CFLAGS) $(TARGET_ARCH_FLAGS) $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)') -std=c11 $(DEPFLAGS) -c $< -o $@

$(LOOM_IMX500_CAMERA_OBJECT): src/imx500_camera.cpp src/imx500_camera.hpp src/usher_frame_timing.h $(LOOM_STATIC_SDK)/loom-static.sha256 $(IMX500_BUILD_DIR)/config.stamp
	$(CXX) $(LOOMCAM_CPPFLAGS) $(IMX500_CXXFLAGS) -Isrc $(DEPFLAGS) -c $< -o $@

$(LOOM_INFERENCE_CAPTURE_OBJECT): src/inference_capture.cpp src/inference_capture.hpp src/imx500_camera.hpp $(LOOM_STATIC_SDK)/loom-static.sha256 $(IMX500_BUILD_DIR)/config.stamp
	$(CXX) $(LOOMCAM_CPPFLAGS) $(IMX500_CXXFLAGS) -Isrc $(DEPFLAGS) -c $< -o $@

$(LOOM_CAPTURE_OBJECT): modules/loom_cam/loom_capture.cpp modules/loom_cam/loom_features.h modules/loom_cam/loom_return.h src/version.h $(LOOMCAM_VERSION_HEADER) $(if $(filter 1,$(CAMERA_CAPTURE)),$(LOOM_STATIC_SDK)/loom-static.sha256) $(IMX500_BUILD_DIR)/config.stamp
	$(CXX) $(if $(filter 1,$(CAMERA_CAPTURE)),$(LOOMCAM_CPPFLAGS) $(IMX500_CXXFLAGS),$(CXXFLAGS) -DSENSORAIA_INFERENCE_ONLY) -Isrc -include $(LOOMCAM_VERSION_HEADER) $(DEPFLAGS) -c $< -o $@

ifeq ($(VERSION_BUMP),1)
$(LOOMCAM_OUTPUT): modules/loom_cam/loom_capture.cpp build-support/make/camera-modules.mk $(LOOM_IMX500_CAMERA_OBJECT) $(LOOM_INFERENCE_CAPTURE_OBJECT) modules/loom_cam/loom_features.h modules/loom_cam/loom_return.h $(LOOMCAM_PHOTO_OBJECT) $(LOOM_FEATURE_OBJECT) $(LOOM_ONNX_FEATURE_OBJECT) $(LOOM_RETURN_OBJECT) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) src/version.h src/usher_frame_timing.h $(IMX500_FRAME_TIMING_OBJECT) $(IMX500_BUILD_DIR)/config.stamp $(VERSION_TOOL) $(LIBJPEG_OBJECTS) $(if $(filter 1,$(CAMERA_CAPTURE)),$(LOOM_STATIC_SDK)/loom-static.sha256) FORCE | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) loomcam LOOMCAM_VERSION $(LOOMCAM_VERSION_HEADER) -- \
	  $(CXX) $(if $(filter 1,$(CAMERA_CAPTURE)),$(LOOMCAM_CPPFLAGS) $(IMX500_CXXFLAGS),$(CXXFLAGS) -DSENSORAIA_INFERENCE_ONLY) -Isrc -include $(LOOMCAM_VERSION_HEADER) \
	$< $(LOOM_IMX500_CAMERA_OBJECT) $(LOOM_INFERENCE_CAPTURE_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(LOOMCAM_PHOTO_OBJECT) $(LOOM_FEATURE_OBJECT) $(LOOM_ONNX_FEATURE_OBJECT) $(LOOM_RETURN_OBJECT) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(if $(filter 1,$(CAMERA_CAPTURE)),$(LOOMCAM_LDFLAGS) $(LOOMCAM_LIBS),$(filter-out -static,$(LDFLAGS))) -pthread -ldl -lm
	$(STRIP) $(STRIPFLAGS) $@
	@readelf -lW $@ | grep -q INTERP
	@! readelf -d $@ | grep -Eq 'NEEDED.*libcamera'
else
$(LOOMCAM_OUTPUT): $(LOOM_CAPTURE_OBJECT) $(LOOM_IMX500_CAMERA_OBJECT) $(LOOM_INFERENCE_CAPTURE_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(LOOMCAM_PHOTO_OBJECT) $(LOOM_FEATURE_OBJECT) $(LOOM_ONNX_FEATURE_OBJECT) $(LOOM_RETURN_OBJECT) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) $(LIBJPEG_OBJECTS) $(if $(filter 1,$(CAMERA_CAPTURE)),$(LOOM_STATIC_SDK)/loom-static.sha256) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CXX) $(LOOM_CAPTURE_OBJECT) $(LOOM_IMX500_CAMERA_OBJECT) $(LOOM_INFERENCE_CAPTURE_OBJECT) $(IMX500_FRAME_TIMING_OBJECT) $(LOOMCAM_PHOTO_OBJECT) $(LOOM_FEATURE_OBJECT) $(LOOM_ONNX_FEATURE_OBJECT) $(LOOM_RETURN_OBJECT) $(EXIF_OBJECTS) $(INFERENCE_IMAGE_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(if $(filter 1,$(CAMERA_CAPTURE)),$(LOOMCAM_LDFLAGS) $(LOOMCAM_LIBS),$(filter-out -static,$(LDFLAGS))) -pthread -ldl -lm
	$(STRIP) $(STRIPFLAGS) $@
	@readelf -lW $@ | grep -q INTERP
	@! readelf -d $@ | grep -Eq 'NEEDED.*libcamera'
endif

-include $(IMX500_BUILD_DIR)/usher_frame_timing.d $(IMX500_BUILD_DIR)/imx500_camera.d $(IMX500_BUILD_DIR)/inference_capture.d $(IMX500_BUILD_DIR)/aicam.d $(IMX500_BUILD_DIR)/spotter_targets.d $(IMX500_BUILD_DIR)/spotter_cam.d $(LOOMCAM_PHOTO_OBJECT:.o=.d) $(LOOM_FEATURE_OBJECT:.o=.d) $(LOOM_ONNX_FEATURE_OBJECT:.o=.d) $(LOOM_RETURN_OBJECT:.o=.d) $(LOOM_CAPTURE_OBJECT:.o=.d) $(IMX500_BUILD_DIR)/loom_imx500_camera.d $(IMX500_BUILD_DIR)/loom_inference_capture.d