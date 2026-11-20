# Shared USHER objects and the core service executables.

$(eval $(call sensoraia-version-header-rule,$(USHER_VERSION_HEADER),usher,USHER_VERSION))
$(eval $(call sensoraia-version-header-rule,$(SODA_VERSION_HEADER),soda,SODA_VERSION))
$(eval $(call sensoraia-version-header-rule,$(EAR_HEATMAP_REPLAY_VERSION_HEADER),ear_heatmap_replay,EAR_HEATMAP_REPLAY_VERSION))
$(eval $(call sensoraia-version-header-rule,$(EAR_HEATMAP_OVERLAY_VERSION_HEADER),ear_heatmap_overlay,EAR_HEATMAP_OVERLAY_VERSION))

$(BUILD_DIR)/project/usher.o: src/usher.c $(HFILES) $(USHER_VERSION_HEADER) $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -include $(USHER_VERSION_HEADER) $(DEPFLAGS) -c $< -o $@

ifeq ($(VERSION_BUMP),1)
$(USHER_OUTPUT): src/usher.c $(PROJECT_OBJECTS) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) $(VERSION_TOOL) FORCE | chdkbridge $(LENS_PROFILES) $(ONNXRUNTIME_BIN_LIBRARIES)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) usher USHER_VERSION $(USHER_VERSION_HEADER) -- \
	  $(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -include $(USHER_VERSION_HEADER) src/usher.c \
	  $(PROJECT_OBJECTS) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@
else
$(USHER_OUTPUT): src/usher.c $(BUILD_DIR)/project/usher.o $(PROJECT_OBJECTS) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) | chdkbridge $(LENS_PROFILES) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CC) $(LDFLAGS) $(BUILD_DIR)/project/usher.o $(PROJECT_OBJECTS) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@
endif

$(BUILD_DIR)/project/%.o: src/%.c $(HFILES) $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@

$(BUILD_DIR)/project/loom_return.o: modules/loom_cam/loom_return.c modules/loom_cam/loom_return.h src/bounded_ini.h $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@

$(BUILD_DIR)/project/spotter_targets.o: modules/spotter_cam/spotter_targets.c modules/spotter_cam/spotter_targets.h $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@

$(addprefix $(BUILD_DIR)/project/,lwir_container.o lwir_frame.o lwir_render.o): $(BUILD_DIR)/project/%.o: modules/lwir_cam/%.c $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -DLWIR_DISABLE_SSSE3 $(DEPFLAGS) -c $< -o $@

$(EAR_CORE_OBJECTS): $(BUILD_DIR)/project/%.o: modules/ear_cam/%.c modules/ear_cam/earcam_core.h modules/ear_cam/ear_heatmap.h $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@

$(EAR_CORE_ARCHIVE): $(EAR_CORE_OBJECTS)
	$(AR) rcs $@ $^

$(BUILD_DIR)/libjpeg/%.o: $(LIBJPEG)/%.c $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(VENDOR_CFLAGS) $(DEPFLAGS) -c $< -o $@

$(BUILD_DIR):
	@mkdir -p $@

$(CONFIG_STAMP): FORCE | $(BUILD_DIR)
	$(call sensoraia-write-stamp,CC CXX TARGET_ARCH_FLAGS OPTFLAGS CPPFLAGS CFLAGS CXXFLAGS VENDOR_CFLAGS LDFLAGS LDLIBS)


$(BUILD_DIR)/project/soda.o: src/soda.cpp src/soda_object_postprocess.h src/version.h src/path_utils.h src/jpeg_marker_guard.h $(SODA_VERSION_HEADER) $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -include $(SODA_VERSION_HEADER) $(DEPFLAGS) -c $< -o $@

$(BUILD_DIR)/project/soda_object_postprocess.o: src/soda_object_postprocess.cpp src/soda_object_postprocess.h $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@

ifeq ($(VERSION_BUMP),1)
$(SODA_OUTPUT): src/soda.cpp src/soda_object_postprocess.cpp src/soda_object_postprocess.h src/version.h src/path_utils.h src/jpeg_marker_guard.h \
		$(BUILD_DIR)/project/image_lens.o $(LENS_OUTPUT) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) $(CONFIG_STAMP) $(VERSION_TOOL) FORCE | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) soda SODA_VERSION $(SODA_VERSION_HEADER) -- \
	  $(CXX) $(CPPFLAGS) $(LDFLAGS) $(CXXFLAGS) -include $(SODA_VERSION_HEADER) \
	  src/soda.cpp src/soda_object_postprocess.cpp $(BUILD_DIR)/project/image_lens.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@
else
$(SODA_OUTPUT): $(BUILD_DIR)/project/soda.o $(BUILD_DIR)/project/soda_object_postprocess.o \
		$(BUILD_DIR)/project/image_lens.o $(LENS_OUTPUT) $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CXX) $(LDFLAGS) $(BUILD_DIR)/project/soda.o $(BUILD_DIR)/project/soda_object_postprocess.o $(BUILD_DIR)/project/image_lens.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) -o $@ $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@
endif

# Ground-side: re-render an earlogs/*.csv session into the heatmap JPEG + .geo sidecar.
.PHONY: ear_heatmap_replay ear-heatmap-replay-build
ifeq ($(VERSION_BUMP),1)
ear_heatmap_replay: $(EAR_CORE_ARCHIVE) $(LIBJPEG_OBJECTS)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) ear_heatmap_replay EAR_HEATMAP_REPLAY_VERSION $(EAR_HEATMAP_REPLAY_VERSION_HEADER) -- \
	  $(MAKE) ear-heatmap-replay-build
else
ear_heatmap_replay: ear-heatmap-replay-build
endif

ear-heatmap-replay-build: $(REPLAY_OUTPUT)

$(BUILD_DIR)/project/ear_heatmap_replay.o: src/ear_heatmap_replay.c src/version.h $(EAR_HEATMAP_REPLAY_VERSION_HEADER) $(CONFIG_STAMP) $(VERSION_BUMP_FORCE)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -include '$(EAR_HEATMAP_REPLAY_VERSION_HEADER)' -MMD -MP -c $< -o $@

$(REPLAY_OUTPUT): $(BUILD_DIR)/project/ear_heatmap_replay.o $(EAR_CORE_ARCHIVE) $(LIBJPEG_OBJECTS) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@

# Ground-side host utility; consumes replay/live artifacts and is not packaged for MORA.
.PHONY: ear_heatmap_overlay ear-heatmap-overlay-build
ifeq ($(VERSION_BUMP),1)
ear_heatmap_overlay: $(EAR_CORE_ARCHIVE) $(LIBJPEG_OBJECTS)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) ear_heatmap_overlay EAR_HEATMAP_OVERLAY_VERSION $(EAR_HEATMAP_OVERLAY_VERSION_HEADER) -- \
	  $(MAKE) ear-heatmap-overlay-build
else
ear_heatmap_overlay: ear-heatmap-overlay-build
endif

ear-heatmap-overlay-build: $(EAR_HEATMAP_OVERLAY_OUTPUT)

$(BUILD_DIR)/project/ear_heatmap_overlay.o: src/ear_heatmap_overlay.c src/version.h modules/ear_cam/ear_heatmap.h $(EAR_HEATMAP_OVERLAY_VERSION_HEADER) $(CONFIG_STAMP) $(VERSION_BUMP_FORCE)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -include '$(EAR_HEATMAP_OVERLAY_VERSION_HEADER)' -MMD -MP -c $< -o $@

$(EAR_HEATMAP_OVERLAY_OUTPUT): $(BUILD_DIR)/project/ear_heatmap_overlay.o $(EAR_CORE_ARCHIVE) $(LIBJPEG_OBJECTS) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)
	$(STRIP) $(STRIPFLAGS) $@

-include $(BUILD_DIR)/project/usher.d $(BUILD_DIR)/project/soda.d $(BUILD_DIR)/project/soda_object_postprocess.d $(BUILD_DIR)/project/ear_heatmap_replay.d $(BUILD_DIR)/project/ear_heatmap_overlay.d
