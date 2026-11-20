# Share implementation objects, not per-model sessions or an ambient host runtime.
ONNXRUNTIME_ROOT ?= $(abspath ext/onnxruntime-linux-$(if $(OUTPUT_SUFFIX),aarch64,x64))
ONNXRUNTIME_LIBRARY ?= $(ONNXRUNTIME_ROOT)/lib/libonnxruntime.so.1.31.0
ONNXRUNTIME_X64_SOURCE = ext/onnxruntime-linux-x64/lib/libonnxruntime.so.1.31.0
ONNXRUNTIME_ARM64_SOURCE = ext/onnxruntime-linux-aarch64/lib/libonnxruntime.so.1.31.0
ONNXRUNTIME_BIN_X64 = $(BIN_DIR)/libonnxruntime.so.1.31.0
ONNXRUNTIME_BIN_ARM64 = $(BIN_DIR)/libonnxruntime.so.1.31.0-arm64
ONNXRUNTIME_BIN_LIBRARIES = $(ONNXRUNTIME_BIN_X64) $(ONNXRUNTIME_BIN_ARM64)
INFERENCE_SOURCES = src/onnx_runtime.c src/onnx_session.c src/inference_settings.c src/inference_model.c src/inference_image.c src/sha256fst.c
INFERENCE_OBJECTS = $(patsubst src/%.c,$(BUILD_DIR)/inference/%.o,$(INFERENCE_SOURCES))
INFERENCE_CPPFLAGS = -Isrc -isystem $(ONNXRUNTIME_ROOT)/include
INFERENCE_IMAGE_OBJECTS = $(INFERENCE_OBJECTS) $(BUILD_DIR)/inference/inference_jpeg.o $(BUILD_DIR)/inference/inference_worker.o

.PHONY: onnxruntime-bin
onnxruntime-bin: $(ONNXRUNTIME_BIN_LIBRARIES)

$(ONNXRUNTIME_BIN_X64): $(ONNXRUNTIME_X64_SOURCE) | $(BIN_DIR)
	@temporary=$$(mktemp '$@.XXXXXX'); cp -f '$<' "$$temporary" && chmod 0755 "$$temporary" && mv -f "$$temporary" '$@'

$(ONNXRUNTIME_BIN_ARM64): $(ONNXRUNTIME_ARM64_SOURCE) | $(BIN_DIR)
	@temporary=$$(mktemp '$@.XXXXXX'); cp -f '$<' "$$temporary" && chmod 0755 "$$temporary" && mv -f "$$temporary" '$@'

$(BUILD_DIR)/inference/%.o: src/%.c $(wildcard src/onnx_*.h src/inference_*.h) src/bounded_ini.h src/jpeg_marker_guard.h build-support/make/onnxruntime.mk
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -fno-tree-loop-distribute-patterns $(TARGET_ARCH_FLAGS) $(if $(IMX500_SYSROOT),--sysroot='$(IMX500_SYSROOT)') \
	  $(INFERENCE_CPPFLAGS) -D_POSIX_C_SOURCE=200809L -MMD -MP -c $< -o $@

-include $(INFERENCE_OBJECTS:.o=.d)