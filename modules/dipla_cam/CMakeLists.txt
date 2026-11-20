# Build the native planner executable for the architecture selected by the
# root Makefile; never infer target ABI from the build host.
cmake_minimum_required(VERSION 3.20)
project(diplacam LANGUAGES C CXX)
set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> crs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> crs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DIPLA_MATH_BACKEND "aigenlib" CACHE STRING "Numerical backend: aigenlib or eigen")
set_property(CACHE DIPLA_MATH_BACKEND PROPERTY STRINGS aigenlib eigen)
add_library(dipla_math INTERFACE)
if(DIPLA_MATH_BACKEND STREQUAL "aigenlib")
  add_subdirectory(aigenlib)
  target_link_libraries(dipla_math INTERFACE dipla_aigenlib)
  target_compile_definitions(dipla_math INTERFACE DIPLA_USE_EIGEN=0)
elseif(DIPLA_MATH_BACKEND STREQUAL "eigen")
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
  target_link_libraries(dipla_math INTERFACE Eigen3::Eigen)
  target_compile_definitions(dipla_math INTERFACE DIPLA_USE_EIGEN=1)
else()
  message(FATAL_ERROR "DIPLA_MATH_BACKEND must be aigenlib or eigen")
endif()
message(STATUS "Diplacam numerical backend: ${DIPLA_MATH_BACKEND}")
find_package(Threads REQUIRED)
set(DIPLA_JSON_READER_DEPTH_LIMIT 16)
# Headers and runtime binary are separate inputs: ARM64 may use a suffixed
# runtime staged by the root build while headers remain under the target SDK.
set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so.1.31.0"
  CACHE FILEPATH "Target architecture's ONNX Runtime shared library")
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
  message(FATAL_ERROR "Set ONNXRUNTIME_LIBRARY to the target architecture's runtime and ONNXRUNTIME_ROOT to its headers")
endif()
configure_file("${ONNXRUNTIME_LIBRARY}"
  "${CMAKE_CURRENT_BINARY_DIR}/libonnxruntime.so.1.31.0" COPYONLY)
add_library(dipla_cmaes STATIC c-cmaes/src/cmaes.c
  c-cmaes/src/boundary_transformation.c)
target_include_directories(dipla_cmaes SYSTEM PUBLIC c-cmaes/src)
add_library(dipla_core STATIC dipla_planner.cpp dipla_gp_belief_planner.cpp cmaes_optimizer.cpp
  ../../src/onnx_runtime.c ../../src/onnx_session.c ../../src/inference_settings.c
  ../../src/inference_model.c ../../src/inference_image.c ../../src/sha256fst.c
  ../../src/jsonbourne.c)
set_source_files_properties(../../src/jsonbourne.c PROPERTIES
  COMPILE_OPTIONS "-std=c11;-Wpedantic;-Wshadow")
target_compile_definitions(dipla_core PRIVATE _POSIX_C_SOURCE=200809L)
target_compile_options(dipla_core PRIVATE -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
target_compile_definitions(dipla_core PRIVATE JSONBOURNE_READER_DEPTH_LIMIT=${DIPLA_JSON_READER_DEPTH_LIMIT})
target_include_directories(dipla_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(dipla_core SYSTEM PRIVATE
  "${ONNXRUNTIME_ROOT}/include")
target_include_directories(dipla_core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../src")
target_link_libraries(dipla_core PUBLIC dipla_math Threads::Threads ${CMAKE_DL_LIBS})
target_link_libraries(dipla_core PRIVATE dipla_cmaes)
add_executable(diplacam diplacam.cpp dipla_worker.cpp)
set(DIPLACAM_VERSION_HEADER "" CACHE FILEPATH "Generated application version header")
set(DIPLACAM_GIT_SHA "unknown" CACHE STRING "Build Git revision")
if(DIPLACAM_VERSION_HEADER)
  target_compile_options(diplacam PRIVATE -include "${DIPLACAM_VERSION_HEADER}")
endif()
target_compile_definitions(diplacam PRIVATE USHER_GIT_SHA="${DIPLACAM_GIT_SHA}")
target_compile_options(diplacam PRIVATE -Wall -Wextra -Werror)
target_link_options(diplacam PRIVATE -Wl,--gc-sections)
target_link_libraries(diplacam PRIVATE dipla_core)
set_target_properties(diplacam PROPERTIES SKIP_BUILD_RPATH TRUE)
