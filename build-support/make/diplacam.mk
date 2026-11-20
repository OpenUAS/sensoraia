# Diplacam CMake targets and architecture-specific runtime selection.
DIPLA_BUILD = $(abspath $(BUILD_DIR))/diplacam
DIPLA_MATH_BACKEND ?= aigenlib
DIPLA_AIGENLIB_BUILD = $(DIPLA_BUILD)-aigenlib
DIPLA_RUNTIME = $(ONNXRUNTIME_ROOT)
DIPLA_RUNTIME_LIBRARY = $(ONNXRUNTIME_LIBRARY)
DIPLA_FIELD ?= $(CURDIR)/modules/dipla_cam/data/map_26_NAIP_grid_counts.csv
DIPLA_WORKER_CONFIG ?= $(CURDIR)/modules/dipla_cam/diplacam-example.ini

_DIPLA_CXX_INCLUDE = $(if $(_DIPLA_CXX_INC_SET$(OUTPUT_SUFFIX)),,$(eval _DIPLA_CXX_INC_VAL$(OUTPUT_SUFFIX) := $$(shell $$(CXX) -print-file-name=include))$(eval _DIPLA_CXX_INC_SET$(OUTPUT_SUFFIX) := 1))$(_DIPLA_CXX_INC_VAL$(OUTPUT_SUFFIX))

$(eval $(call sensoraia-version-header-rule,$(DIPLACAM_VERSION_HEADER),diplacam,DIPLACAM_VERSION))

.PHONY: diplacam-aigenlib diplacam-aigenlib-arm64
.PHONY: diplacam diplacam-build diplacam-arm64 diplacam-clean
.PHONY: diplacam-package diplacam-package-stage diplacam-package-native

diplacam-aigenlib:
	$(if $(MAKE_DRY_RUN),,+)\
	set -eu; \
	mkdir -p '$(dir $(DIPLA_AIGENLIB_BUILD))'; \
	exec 9>'$(DIPLA_AIGENLIB_BUILD).lock'; \
	flock 9; \
	ar_bin=$$(command -v '$(AR)'); \
	ranlib_bin=$$(command -v '$(RANLIB)'); \
	cmake -S modules/dipla_cam/aigenlib -B '$(DIPLA_AIGENLIB_BUILD)' -DCMAKE_BUILD_TYPE=Release \
	  -DCMAKE_CXX_COMPILER='$(CXX)' \
	  -DCMAKE_AR="$$ar_bin" \
	  -DCMAKE_RANLIB="$$ranlib_bin" \
	  -DCMAKE_CXX_FLAGS="$(if $(OUTPUT_SUFFIX),-isystem '$(_DIPLA_CXX_INCLUDE)' $(IMX500_CXXFLAGS))" \
	  -DCMAKE_EXE_LINKER_FLAGS="$(if $(OUTPUT_SUFFIX),-L'$(IMX500_SYSROOT)/usr/lib/$(IMX500_TRIPLET)' -L'$(IMX500_SYSROOT)/lib/$(IMX500_TRIPLET)' $(IMX500_LDFLAGS))"; \
	cmake --build '$(DIPLA_AIGENLIB_BUILD)'

diplacam-aigenlib-arm64:
	@test -f '$(IMX500_ARM64_SYSROOT)/usr/include/stdio.h' || { echo 'Prepare the Trixie SDK with make imx500-sdk first.' >&2; exit 1; }
	$(MAKE) diplacam-aigenlib OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)' OPTFLAGS='$(ARM64_OPTFLAGS)'

ifeq ($(VERSION_BUMP),1)
diplacam:
	VERSION_BUMP='$(VERSION_BUMP)' bash $(VERSION_TOOL) $(VERSION_STATE) $(VERSION_LOCK) diplacam DIPLACAM_VERSION $(DIPLACAM_VERSION_HEADER) -- \
	  $(MAKE) diplacam-build
else
diplacam: diplacam-build
endif

DIPLA_OUTPUT = $(BIN_DIR)/diplacam$(OUTPUT_SUFFIX)
DIPLA_CONFIG_STAMP = $(DIPLA_BUILD)/make-config.stamp
DIPLA_SOURCES = $(wildcard modules/dipla_cam/*.cpp modules/dipla_cam/*.hpp modules/dipla_cam/*.h modules/dipla_cam/CMakeLists.txt \
	modules/dipla_cam/aigenlib/*.cpp modules/dipla_cam/aigenlib/*.hpp modules/dipla_cam/aigenlib/CMakeLists.txt \
	modules/dipla_cam/c-cmaes/src/*.c modules/dipla_cam/c-cmaes/src/*.h) \
	$(addprefix src/,onnx_runtime.c onnx_runtime.h onnx_session.c onnx_session.h inference_settings.c inference_settings.h \
	inference_model.c inference_model.h inference_image.c inference_image.h sha256fst.c sha256fst.h jsonbourne.c jsonbourne.h bounded_ini.h version.h)

$(DIPLA_CONFIG_STAMP): FORCE
	$(call sensoraia-write-stamp,DIPLA_MATH_BACKEND DIPLA_RUNTIME DIPLA_RUNTIME_LIBRARY DIPLACAM_VERSION_HEADER USHER_GIT_SHA IMX500_SYSROOT CC CXX AR RANLIB TARGET_ARCH_FLAGS IMX500_CXXFLAGS IMX500_LDFLAGS)

diplacam-build: $(DIPLA_OUTPUT)

$(DIPLA_OUTPUT): $(DIPLA_SOURCES) $(DIPLACAM_VERSION_HEADER) $(DIPLA_CONFIG_STAMP) $(VERSION_BUMP_FORCE) | $(BIN_DIR) $(ONNXRUNTIME_BIN_LIBRARIES)
	$(if $(MAKE_DRY_RUN),,+)\
	set -eu; \
	mkdir -p '$(dir $(DIPLA_BUILD))'; \
	exec 9>'$(DIPLA_BUILD).lock'; \
	flock 9; \
	ar_bin=$$(command -v '$(AR)'); \
	ranlib_bin=$$(command -v '$(RANLIB)'); \
	if test ! -f '$(DIPLA_BUILD)/CMakeCache.txt' \
	    || test modules/dipla_cam/CMakeLists.txt -nt '$(DIPLA_BUILD)/CMakeCache.txt' \
	    || test modules/dipla_cam/aigenlib/CMakeLists.txt -nt '$(DIPLA_BUILD)/CMakeCache.txt' \
	    || test '$(DIPLA_CONFIG_STAMP)' -nt '$(DIPLA_BUILD)/CMakeCache.txt'; then \
	  cmake -S modules/dipla_cam -B '$(DIPLA_BUILD)' -DCMAKE_BUILD_TYPE=Release \
	    -DDIPLA_MATH_BACKEND='$(DIPLA_MATH_BACKEND)' \
	    -DDIPLACAM_VERSION_HEADER='$(DIPLACAM_VERSION_HEADER)' -DDIPLACAM_GIT_SHA='$(USHER_GIT_SHA)' \
	    -DONNXRUNTIME_ROOT='$(DIPLA_RUNTIME)' \
	    -DONNXRUNTIME_LIBRARY='$(DIPLA_RUNTIME_LIBRARY)' \
	    -DCMAKE_C_COMPILER='$(CC)' \
	    -DCMAKE_C_FLAGS="$(if $(OUTPUT_SUFFIX),--sysroot='$(IMX500_SYSROOT)' $(TARGET_ARCH_FLAGS))" \
	    -DCMAKE_CXX_COMPILER='$(CXX)' \
	    -DCMAKE_CXX_FLAGS="$(if $(OUTPUT_SUFFIX),-isystem '$(_DIPLA_CXX_INCLUDE)' $(IMX500_CXXFLAGS))" \
	    -DCMAKE_AR="$$ar_bin" \
	    -DCMAKE_RANLIB="$$ranlib_bin" \
	    -DCMAKE_EXE_LINKER_FLAGS="$(if $(OUTPUT_SUFFIX),-L'$(IMX500_SYSROOT)/usr/lib/$(IMX500_TRIPLET)' -L'$(IMX500_SYSROOT)/lib/$(IMX500_TRIPLET)' $(IMX500_LDFLAGS))"; \
	fi; \
	for archive in '$(DIPLA_BUILD)/libdipla_core.a' '$(DIPLA_BUILD)/libdipla_cmaes.a' '$(DIPLA_BUILD)/aigenlib/libdipla_aigenlib.a'; do \
	  test -f "$$archive" || continue; \
	  if ! '$(NM)' -s "$$archive" 2>/dev/null | awk '\
	    /^Archive index:$$/ { in_idx = 1; has_idx = 1; next } \
	    in_idx && NF == 0 { in_idx = 0; next } \
	    in_idx && NF >= 3 && $$(NF-1) == "in" { indexed[$$1 SUBSEP $$NF] = 1; next } \
	    !in_idx && /^[^[:space:]].*:$$/ { obj = substr($$0, 1, length($$0) - 1); next } \
	    !in_idx && NF >= 3 && $$2 ~ /^[ABCDGRSTVWu]$$/ { \
	      defined_count++; \
	      if (!(($$3 SUBSEP obj) in indexed)) { bad = 1; exit 1 } \
	    } \
	    END { if (bad || (defined_count > 0 && !has_idx)) exit 1 }'; then \
	    printf 'Rebuilding corrupted static archive: %s\n' "$$archive" >&2; \
	    rm -f "$$archive"; \
	    find '$(DIPLA_BUILD)' -name '*.o' -type f | while IFS= read -r obj_file; do \
	      '$(NM)' "$$obj_file" >/dev/null 2>&1 || rm -f "$$obj_file"; \
	    done; \
	  fi; \
	done; \
	cmake --build '$(DIPLA_BUILD)'; \
	temporary=$$(mktemp '$(DIPLA_OUTPUT).XXXXXX'); \
	trap 'rm -f "$$temporary"' EXIT; \
	install -m 0755 '$(DIPLA_BUILD)/diplacam' "$$temporary"; \
	'$(STRIP)' $(STRIPFLAGS) "$$temporary"; \
	mv -f "$$temporary" '$(DIPLA_OUTPUT)'; \
	trap - EXIT

diplacam-arm64:
	@test -f '$(IMX500_ARM64_SYSROOT)/usr/include/stdio.h' || { echo 'Prepare the Trixie SDK with make imx500-sdk first.' >&2; exit 1; }
	$(MAKE) diplacam OUTPUT_SUFFIX=-arm64 BUILD_DIR=.build/arm64 \
	  IMX500_SYSROOT='$(IMX500_ARM64_SYSROOT)' OPTFLAGS='$(ARM64_OPTFLAGS)'

diplacam-clean:
	rm -rf .build/diplacam .build/diplacam.lock .build/diplacam-aigenlib .build/diplacam-aigenlib.lock \
	  .build/arm64/diplacam .build/arm64/diplacam.lock .build/arm64/diplacam-aigenlib .build/arm64/diplacam-aigenlib.lock \
	  bin/diplacam bin/diplacam-arm64

$(eval $(call sensoraia-module-verify-rules,diplacam,bin/diplacam,bin/diplacam-arm64))

diplacam-package-stage: private OUTPUT_SUFFIX = -arm64
diplacam-package-stage:
	CROSS_COMPILE='$(CROSS_COMPILE)' DIPLA_RUNTIME_LIBRARY='$(DIPLA_RUNTIME_LIBRARY)' \
	  DIPLA_WORKER_CONFIG='$(DIPLA_WORKER_CONFIG)' \
	  bash build-support/package_diplacam.sh aarch64 '$(DIPLA_FIELD)'

diplacam-package: diplacam-arm64
	$(MAKE) diplacam-package-stage

diplacam-package-native: diplacam
	DIPLA_RUNTIME_LIBRARY='$(DIPLA_RUNTIME_LIBRARY)' DIPLA_WORKER_CONFIG='$(DIPLA_WORKER_CONFIG)' bash build-support/package_diplacam.sh x64 '$(DIPLA_FIELD)'