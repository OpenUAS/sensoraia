# Camera Lens and PANGEA share the source-built static OpenCV profile.
.PHONY: camera-lens camera-lens-native-deps pangea-native pangea-native-cpu pangea-clean
LENS_OPENCV_ROOT = $(abspath $(BUILD_DIR))/lens-opencv/install
LENS_OPENCV_BUILD = $(LENS_OPENCV_ROOT)/../build-cat-ext
LENS_OPENCV_STAMP = $(LENS_OPENCV_ROOT)/../source-config.stamp
override LENS_OPENCV_LIBRARIES = $(addprefix $(LENS_OPENCV_ROOT)/lib/libopencv_,stitching.a calib3d.a features2d.a flann.a imgproc.a core.a)
LWIR_CALIBRATE_OPENCV_HAL_LIBRARIES = $(foreach name,tegra_hal kleidicv kleidicv_thread kleidicv_hal,$(wildcard $(LENS_OPENCV_ROOT)/lib/opencv4/3rdparty/lib$(name).a))
LWIR_CALIBRATE_OPENCV_LIBRARIES = $(addprefix $(LENS_OPENCV_ROOT)/lib/libopencv_,calib3d.a features2d.a flann.a imgproc.a core.a) \
	$(LWIR_CALIBRATE_OPENCV_HAL_LIBRARIES) $(LENS_OPENCV_ROOT)/lib/opencv4/3rdparty/libzlib.a
LWIR_CALIBRATE_JPEG_OBJECTS = $(LIBJPEG_OBJECTS)
LWIR_CALIBRATE_CXXFLAGS = $(CXXFLAGS) -std=c++17
LENS_BUILD_JOBS ?= $(BUILD_JOBS)
LENS_OPENCV_SANITIZER_FLAGS = $(if $(findstring -fsanitize=thread,$(TARGET_ARCH_FLAGS)),-DCV_THREAD_SANITIZER=1)
PANGEA_SURVEY_DIR = modules/pangea
PANGEA_BUILD = $(BUILD_DIR)/pangea
PANGEA_SOURCES = pangea.cpp survey.cpp ortho_map.cpp
PANGEA_OBJECTS = $(addprefix $(PANGEA_BUILD)/,$(PANGEA_SOURCES:.cpp=.o))
PANGEA_OPTFLAGS ?= -O3 -flto=$(BUILD_JOBS) -fno-fast-math -ffp-contract=off
PANGEA_ARCH_FLAGS ?= $(if $(OUTPUT_SUFFIX),-mcpu=cortex-a53)
PANGEA_CXXFLAGS = $(filter-out -O%,$(CXXFLAGS)) $(PANGEA_OPTFLAGS) $(PANGEA_ARCH_FLAGS)
PANGEA_CONFIG_STAMP = $(BUILD_DIR)/pangea-config.stamp
override LENS_LIBS = $(LENS_OPENCV_LIBRARIES) \
	$(shell test ! -f '$(LENS_OPENCV_BUILD)/install_manifest.txt' || sed -n '\|/lib/opencv4/3rdparty/.*\.a$$|p' '$(LENS_OPENCV_BUILD)/install_manifest.txt')

$(eval $(call sensoraia-version-header-rule,$(PANGEA_VERSION_HEADER),pangea,PANGEA_VERSION))
$(eval $(call sensoraia-version-header-rule,$(CAMERA_LENS_VERSION_HEADER),camera_lens,CAMERA_LENS_VERSION))
$(eval $(call sensoraia-version-header-rule,$(LWIR_CALIBRATE_VERSION_HEADER),lwir_calibrate,LWIR_CALIBRATE_VERSION))

$(PANGEA_BUILD)/%.o: $(PANGEA_SURVEY_DIR)/%.cpp src/version.h $(PANGEA_VERSION_HEADER) $(LENS_OPENCV_ROOT)/lib/libopencv_stitching.a $(PANGEA_CONFIG_STAMP) $(CONFIG_STAMP) | opencv-source-check
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(PANGEA_CXXFLAGS) -include $(PANGEA_VERSION_HEADER) -std=c++17 \
	  -I$(LENS_OPENCV_ROOT)/include/opencv4 -I$(PANGEA_SURVEY_DIR) $(DEPFLAGS) -c $< -o $@

ifeq ($(VERSION_BUMP),1)
$(PANGEA_OUTPUT): $(PANGEA_SURVEY_DIR)/pangea.cpp src/version.h src/opencv_json.hpp src/opencv_parallel.hpp $(PANGEA_SURVEY_DIR)/survey.cpp $(PANGEA_SURVEY_DIR)/ortho_map.cpp \
		$(wildcard $(PANGEA_SURVEY_DIR)/*.h) \
		$(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o $(LENS_OPENCV_ROOT)/lib/libopencv_stitching.a \
		$(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) $(PANGEA_CONFIG_STAMP) $(CONFIG_STAMP) $(VERSION_TOOL) FORCE | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES) opencv-source-check
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) pangea PANGEA_VERSION $(PANGEA_VERSION_HEADER) -- \
	  $(CXX) $(CPPFLAGS) $(PANGEA_CXXFLAGS) \
	  -include $(PANGEA_VERSION_HEADER) -std=c++17 -I$(LENS_OPENCV_ROOT)/include/opencv4 \
	  -I$(PANGEA_SURVEY_DIR) \
	  $(LDFLAGS) $(PANGEA_SURVEY_DIR)/pangea.cpp $(PANGEA_SURVEY_DIR)/survey.cpp $(PANGEA_SURVEY_DIR)/ortho_map.cpp \
	  $(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o $(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) \
	  -Wl,--gc-sections -Wl,--start-group $(LENS_LIBS) -Wl,--end-group -o $@ $(LDLIBS) -ldl -lrt
	$(STRIP) $(STRIPFLAGS) $@
else
$(PANGEA_OUTPUT): $(PANGEA_OBJECTS) $(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o \
		$(LENS_OPENCV_ROOT)/lib/libopencv_stitching.a $(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES) opencv-source-check
	$(CXX) $(PANGEA_CXXFLAGS) $(LDFLAGS) $(PANGEA_OBJECTS) \
	  $(BUILD_DIR)/project/camera_profile.o $(BUILD_DIR)/project/coverage_scheduler.o $(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) \
	  -Wl,--gc-sections -Wl,--start-group $(LENS_LIBS) -Wl,--end-group -o $@ $(LDLIBS) -ldl -lrt
	$(STRIP) $(STRIPFLAGS) $@
endif

$(PANGEA_CONFIG_STAMP): FORCE
	$(call sensoraia-write-stamp,CXX PANGEA_CXXFLAGS)

pangea-clean:
	rm -rf .build/pangea .build/pangea-config.stamp .build/arm64/pangea .build/arm64/pangea-config.stamp bin/pangea bin/pangea-arm64

$(eval $(call sensoraia-module-verify-rules,pangea,bin/pangea,bin/pangea-arm64))

camera-lens-native-deps: $(LENS_OPENCV_ROOT)/lib/libopencv_calib3d.a

.PHONY: lwir_calibrate lwir-calibrate-arm64 lwir-calibrate-build
ifeq ($(VERSION_BUMP),1)
lwir_calibrate: $(LENS_OPENCV_LIBRARIES) $(LWIR_CALIBRATE_JPEG_OBJECTS) $(CONFIG_STAMP)
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) lwir_calibrate LWIR_CALIBRATE_VERSION $(LWIR_CALIBRATE_VERSION_HEADER) -- \
	  $(MAKE) lwir-calibrate-build LWIR_CALIBRATE_VERSION_HEADER='$(LWIR_CALIBRATE_VERSION_HEADER)'
else
lwir_calibrate: $(LENS_OPENCV_LIBRARIES) $(LWIR_CALIBRATE_JPEG_OBJECTS) $(CONFIG_STAMP) lwir-calibrate-build
endif

lwir-calibrate-arm64:
	$(MAKE) lwir_calibrate OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64

lwir-calibrate-build: $(LWIR_CALIBRATE_OUTPUT)

$(BUILD_DIR)/lwir_calibrate.o: src/lwir_calibrate.cpp src/opencv_parallel.hpp src/jpeg_marker_guard.h src/version.h $(LWIR_CALIBRATE_VERSION_HEADER) \
		$(LENS_OPENCV_ROOT)/lib/libopencv_calib3d.a $(CONFIG_STAMP) $(VERSION_BUMP_FORCE) | opencv-source-check
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(LWIR_CALIBRATE_CXXFLAGS) -include $(LWIR_CALIBRATE_VERSION_HEADER) \
	  -I$(LENS_OPENCV_ROOT)/include/opencv4 $(DEPFLAGS) -c $< -o $@

$(LWIR_CALIBRATE_OUTPUT): $(BUILD_DIR)/lwir_calibrate.o $(LENS_OPENCV_ROOT)/lib/libopencv_calib3d.a $(LWIR_CALIBRATE_JPEG_OBJECTS) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES) opencv-source-check
	$(CXX) $(LWIR_CALIBRATE_CXXFLAGS) $(LDFLAGS) $(BUILD_DIR)/lwir_calibrate.o $(LWIR_CALIBRATE_JPEG_OBJECTS) \
	  -Wl,--gc-sections -Wl,--start-group $(LWIR_CALIBRATE_OPENCV_LIBRARIES) -Wl,--end-group \
	  -o $@ $(LDLIBS) -ldl -lrt
	$(STRIP) $(STRIPFLAGS) $@

LENS_OPTFLAGS ?= -O3 -fno-fast-math -ffp-contract=off
LENS_CXXFLAGS = $(filter-out -O%,$(CXXFLAGS)) $(LENS_OPTFLAGS)
LENS_CONFIG_STAMP = $(BUILD_DIR)/lens-config.stamp

$(BUILD_DIR)/camera_lens.o: src/camera_lens.cpp src/version.h src/opencv_json.hpp src/opencv_parallel.hpp $(IMAGE_EXIF_HEADERS) \
		$(CAMERA_LENS_VERSION_HEADER) $(LENS_OPENCV_ROOT)/lib/libopencv_calib3d.a $(LENS_CONFIG_STAMP) $(CONFIG_STAMP) | opencv-source-check
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(LENS_CXXFLAGS) -include $(CAMERA_LENS_VERSION_HEADER) -std=c++17 \
	  -I$(LENS_OPENCV_ROOT)/include/opencv4 $(DEPFLAGS) -c $< -o $@

ifeq ($(VERSION_BUMP),1)
$(LENS_OUTPUT): src/camera_lens.cpp src/version.h src/opencv_json.hpp src/opencv_parallel.hpp $(IMAGE_EXIF_HEADERS) $(LENS_OPENCV_ROOT)/lib/libopencv_calib3d.a \
		$(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) $(LENS_CONFIG_STAMP) $(CONFIG_STAMP) $(VERSION_TOOL) FORCE | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES) opencv-source-check
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) camera_lens CAMERA_LENS_VERSION $(CAMERA_LENS_VERSION_HEADER) -- \
	  $(CXX) $(CPPFLAGS) $(LENS_CXXFLAGS) -include $(CAMERA_LENS_VERSION_HEADER) -std=c++17 -I$(LENS_OPENCV_ROOT)/include/opencv4 \
	  $(LDFLAGS) src/camera_lens.cpp $(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) \
	  -Wl,--gc-sections -Wl,--start-group $(LENS_LIBS) -Wl,--end-group -o $@ $(LDLIBS) -ldl -lrt
	$(STRIP) $(STRIPFLAGS) $@
else
$(LENS_OUTPUT): $(BUILD_DIR)/camera_lens.o $(LENS_OPENCV_ROOT)/lib/libopencv_calib3d.a \
		$(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES) opencv-source-check
	$(CXX) $(LENS_CXXFLAGS) $(LDFLAGS) $(BUILD_DIR)/camera_lens.o $(BUILD_DIR)/lens-sha256fst.o $(EXIF_OBJECTS) $(LIBJPEG_OBJECTS) \
	  -Wl,--gc-sections -Wl,--start-group $(LENS_LIBS) -Wl,--end-group -o $@ $(LDLIBS) -ldl -lrt
	$(STRIP) $(STRIPFLAGS) $@
endif

$(LENS_CONFIG_STAMP): FORCE
	$(call sensoraia-write-stamp,CXX LENS_CXXFLAGS)

$(BUILD_DIR)/lens-sha256fst.o: src/sha256fst.c src/sha256fst.h $(CONFIG_STAMP)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@

$(LENS_OPENCV_STAMP): FORCE
	@mkdir -p '$(@D)'
	@bash -e -o pipefail -c ' \
	  temporary=$$(mktemp "$$1.tmp.XXXXXX"); \
	  trap '\''rm -f -- "$$temporary"'\'' EXIT; \
	  { printf "%s\n" \
	      "SOURCE=$$2" "CC=$$3" "CXX=$$4" "TARGET=$$5" \
	      "TARGET_ARCH_FLAGS=$$6" "WITH_PTHREADS_PF=OFF"; \
	    test -f "$$2/CMakeLists.txt"; \
	    (cd -- "$$2" && LC_ALL=C find . \
	      \( -name .git -o -name .hg -o -name .svn -o -name .cache \) -prune \
	      -o -type f -printf "%P\0" | LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum); \
	  } > "$$temporary"; \
	  cmp -s -- "$$temporary" "$$1" || mv -f -- "$$temporary" "$$1"' \
	  lens_opencv_stamp '$@' '$(OPENCV_SOURCE)' '$(CC)' '$(CXX)' '$(OUTPUT_SUFFIX)' '$(TARGET_ARCH_FLAGS)'

.PHONY: opencv-source-check
opencv-source-check: $(LENS_OPENCV_LIBRARIES)
	bash build-support/check_opencv_sources.sh '$(LENS_OPENCV_BUILD)/CMakeCache.txt' '$(LENS_OPENCV_ROOT)'

$(LENS_OPENCV_LIBRARIES) &: Makefile build-support/make/vision.mk $(LENS_OPENCV_STAMP)
	set -e; exec 9>'$(LENS_OPENCV_ROOT)/../build.lock'; flock 9; \
	for configure_pass in 1 2; do \
	cmake -S '$(OPENCV_SOURCE)' -B '$(LENS_OPENCV_BUILD)' \
	  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX='$(LENS_OPENCV_ROOT)' \
	  -DCMAKE_DISABLE_FIND_PACKAGE_Git=ON -DOPENCV_VCSVERSION=unknown \
	  -DOPENCV_EXTRA_MODULES_PATH= \
	  $(if $(OUTPUT_SUFFIX),-DWITH_KLEIDICV=OFF) \
	  -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_C_COMPILER=$(CC) -DCMAKE_CXX_COMPILER=$(CXX) \
	  $(if $(OUTPUT_SUFFIX),-DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64) \
	  -DBUILD_LIST=core,imgproc,calib3d,stitching -DBUILD_opencv_imgcodecs=OFF -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTS=OFF \
	  -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF \
	  -DBUILD_JAVA=OFF -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_OPENCL=OFF \
	  -DWITH_LAPACK=OFF -DWITH_EIGEN=OFF -DWITH_TBB=OFF -DWITH_PTHREADS_PF=OFF -DWITH_GTK=OFF \
	  -DWITH_QT=OFF -DWITH_FFMPEG=OFF -DWITH_V4L=OFF -DWITH_PNG=OFF -DWITH_JPEG=OFF -DWITH_TIFF=OFF \
	  -DWITH_WEBP=OFF -DWITH_AVIF=OFF -DWITH_OPENJPEG=OFF -DWITH_OPENEXR=OFF -DWITH_JASPER=OFF \
	  -DBUILD_JPEG=OFF -DBUILD_PNG=OFF -DBUILD_TIFF=OFF -DBUILD_WEBP=OFF -DBUILD_OPENEXR=OFF \
	  -DBUILD_ZLIB=ON -DCPU_DISPATCH= \
	  -DCMAKE_C_FLAGS_RELEASE='-O2 -DNDEBUG $(TARGET_ARCH_FLAGS) $(LENS_OPENCV_SANITIZER_FLAGS)' \
	  -DCMAKE_CXX_FLAGS_RELEASE='-O2 -DNDEBUG $(TARGET_ARCH_FLAGS) $(LENS_OPENCV_SANITIZER_FLAGS)' \
	  -DPARALLEL_ENABLE_PLUGINS=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF || exit $$?; \
	done; \
	cmake --build '$(LENS_OPENCV_BUILD)'; \
	cmake --install '$(LENS_OPENCV_BUILD)'; \
	bash build-support/check_opencv_sources.sh '$(LENS_OPENCV_BUILD)/CMakeCache.txt' '$(LENS_OPENCV_ROOT)'; \
	touch $(LENS_OPENCV_LIBRARIES)

camera-lens: $(LENS_OUTPUT) $(LENS_PROFILES)

pangea-native: PANGEA_ARCH_FLAGS = -march=native -mtune=native
pangea-native: pangea
pangea-native-cpu: pangea-native

-include $(PANGEA_OBJECTS:.o=.d) $(BUILD_DIR)/lwir_calibrate.d $(BUILD_DIR)/camera_lens.d $(BUILD_DIR)/lens-sha256fst.d