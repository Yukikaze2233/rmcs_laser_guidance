cmake_minimum_required(VERSION 3.16)
project(rmcs_laser_guidance)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME "Enable ONNX Runtime model backend support." OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to an ONNX Runtime installation root")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)
find_package(yaml-cpp REQUIRED)

include(GNUInstallDirs)

set(RMCS_LASER_GUIDANCE_USE_AMENT ${ament_cmake_FOUND})

if(RMCS_LASER_GUIDANCE_USE_AMENT)
  message(STATUS "Configuring rmcs_laser_guidance with ament_cmake")
else()
  message(STATUS "Configuring rmcs_laser_guidance in standalone CMake mode")
endif()

if(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
  find_path(
    RMCS_LASER_GUIDANCE_ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime/core/session/onnxruntime_cxx_api.h
    HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT
    PATH_SUFFIXES include
  )
  find_library(
    RMCS_LASER_GUIDANCE_ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT
    PATH_SUFFIXES lib lib64
  )

  if(NOT RMCS_LASER_GUIDANCE_ONNXRUNTIME_INCLUDE_DIR OR NOT RMCS_LASER_GUIDANCE_ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR
      "RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON requires ONNX Runtime. "
      "Set ONNXRUNTIME_ROOT or export ONNXRUNTIME_ROOT to point at the installation root.")
  endif()

  add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
  set_target_properties(
    onnxruntime::onnxruntime
    PROPERTIES
      IMPORTED_LOCATION "${RMCS_LASER_GUIDANCE_ONNXRUNTIME_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${RMCS_LASER_GUIDANCE_ONNXRUNTIME_INCLUDE_DIR}"
  )

  message(STATUS "ONNX Runtime model backend enabled")
endif()

set(default_config_path "${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml")
set(default_sample_replay_path "${CMAKE_CURRENT_SOURCE_DIR}/test_data/sample_images")
set(default_video_session_root "${CMAKE_CURRENT_SOURCE_DIR}/videos")

add_library(
  ${PROJECT_NAME}_core
  SHARED
  src/config.cpp
  src/debug_renderer.cpp
  src/detector.cpp
  src/frame_format.cpp
  src/model_adapter.cpp
  src/model_infer.cpp
  src/model_runtime.cpp
  src/pipeline.cpp
  src/red_target_refiner.cpp
  src/replay.cpp
  src/training_data.cpp
  src/v4l2_capture.cpp
)

target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${OpenCV_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}_core PUBLIC yaml-cpp::yaml-cpp ${OpenCV_LIBS})

if(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
  target_link_libraries(${PROJECT_NAME}_core PRIVATE onnxruntime::onnxruntime)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=1)
endif()

function(rmcs_laser_guidance_add_default_paths target_name)
  target_compile_definitions(
    ${target_name}
    PRIVATE
      RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH="${default_config_path}"
      RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH="${default_sample_replay_path}"
      RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT="${default_video_session_root}"
  )
endfunction()

function(rmcs_laser_guidance_add_example target_name source_path)
  add_executable(${target_name} ${source_path})
  rmcs_laser_guidance_add_default_paths(${target_name})
  target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${target_name} PRIVATE ${PROJECT_NAME}_core ${OpenCV_LIBS})
endfunction()

function(rmcs_laser_guidance_add_test target_name source_path)
  add_executable(${target_name} ${source_path})
  rmcs_laser_guidance_add_default_paths(${target_name})
  target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${target_name} PRIVATE ${PROJECT_NAME}_core ${OpenCV_LIBS})
  add_test(NAME ${target_name} COMMAND ${target_name})
endfunction()

enable_testing()

rmcs_laser_guidance_add_example(example_offline_smoke examples/offline_smoke.cpp)
rmcs_laser_guidance_add_example(example_v4l2_preview examples/v4l2_preview.cpp)
rmcs_laser_guidance_add_example(example_v4l2_capture examples/v4l2_capture.cpp)
rmcs_laser_guidance_add_example(
  example_v4l2_record_session
  examples/v4l2_record_session.cpp
)
rmcs_laser_guidance_add_example(
  example_export_training_frames
  examples/export_training_frames.cpp
)
rmcs_laser_guidance_add_example(example_replay_preview examples/replay_preview.cpp)
rmcs_laser_guidance_add_example(
  example_detector_benchmark
  examples/detector_benchmark.cpp
)
rmcs_laser_guidance_add_example(example_model_infer examples/model_infer.cpp)
rmcs_laser_guidance_add_example(
  example_transcode_recorded_session
  examples/transcode_recorded_session.cpp
)

rmcs_laser_guidance_add_test(config_test tests/config_test.cpp)
rmcs_laser_guidance_add_test(pipeline_test tests/pipeline_test.cpp)
rmcs_laser_guidance_add_test(frame_format_test tests/frame_format_test.cpp)
rmcs_laser_guidance_add_test(detector_test tests/detector_test.cpp)
rmcs_laser_guidance_add_test(debug_renderer_test tests/debug_renderer_test.cpp)
rmcs_laser_guidance_add_test(sample_images_test tests/sample_images_test.cpp)
rmcs_laser_guidance_add_test(model_infer_test tests/model_infer_test.cpp)
rmcs_laser_guidance_add_test(red_target_refiner_test tests/red_target_refiner_test.cpp)
rmcs_laser_guidance_add_test(training_data_test tests/training_data_test.cpp)

install(
  TARGETS
    ${PROJECT_NAME}_core
    example_offline_smoke
    example_v4l2_preview
    example_v4l2_capture
    example_v4l2_record_session
    example_export_training_frames
    example_replay_preview
    example_detector_benchmark
    example_model_infer
    example_transcode_recorded_session
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY test_data
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md AGENTS.md plan.md
  DESTINATION share/${PROJECT_NAME}
)

if(RMCS_LASER_GUIDANCE_USE_AMENT)
  ament_export_include_directories(${CMAKE_INSTALL_INCLUDEDIR})
  ament_export_dependencies(OpenCV yaml-cpp)
  ament_package()
endif()
