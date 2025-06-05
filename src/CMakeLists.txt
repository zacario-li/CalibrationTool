# Define the executable
add_executable(calibration_tool_app
    main.cpp
    app.cpp
    # app.h # Header files are typically not listed in add_executable
    ui/main_window.cpp
    ui/tabs/tab_single_cam.cpp
    ui/tabs/tab_stereo_cam.cpp
    ui/tabs/tab_hand_eye.cpp
    ui/tabs/tab_stereo_disparity.cpp
    core/common/file_utils.cpp
    core/calib/transformation_utils.cpp
    core/calib/calib_board.cpp
    core/calib/hand_eye_calibration.cpp
    core/detectors/custom_checkerboard_detector.cpp
    core/depth_processing/sgbm_processor.cpp
    core/common/storage_service.cpp
    ui/components/custom_events.cpp
    ui/components/image_panel.cpp
    ui/components/details_image_panel.cpp
    ui/dialogs/stereo_file_loader_dialog.cpp
    # ...
    # core/calib/calibration_algorithms.cpp
    # ...
)

# Include directories
target_include_directories(calibration_tool_app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}      # For headers in src/ (e.g. app.h, ui/main_window.h)
                                     # and allows src/core/common for core/common/file_utils.h
    "${CMAKE_SOURCE_DIR}/src"        # To find core/common/file_utils.h as <core/common/file_utils.h>
    ${wxWidgets_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${SQLite3_INCLUDE_DIRS}
    # ${Eigen3_INCLUDE_DIRS} # If found as package
    # Add include dirs for spdlog if needed (often handled by their own CMake targets if added as subdir)
    "${CMAKE_SOURCE_DIR}/third_party/eigen" # Example if Eigen is just headers in third_party
    "${CMAKE_SOURCE_DIR}/third_party/spdlog/include" # Example for spdlog headers
    "${CMAKE_SOURCE_DIR}/third_party/nlohmann_json/single_include" # For nlohmann_json
)

# Link libraries
target_link_libraries(calibration_tool_app PRIVATE
    ${wxWidgets_LIBRARIES}
    ${OpenCV_LIBRARIES}
    # VTK libraries are often linked using target_link_libraries(my_target PRIVATE VTK::RenderingCore VTK::IOImage ... etc.)
    # For simplicity here, using the variable, but component-based linking is better.
    ${VTK_LIBRARIES}
    ${Open3D_LIBRARIES} # Or Open3D::Open3D if it provides such a target
    ${SQLite3_LIBRARIES}
    # Eigen3::Eigen # If found as package and provides a target
    # spdlog::spdlog # If found as package or built from third_party
)

# If VTK components are used, link them specifically:
# Example:
# target_link_libraries(calibration_tool_app PRIVATE
#    VTK::CommonCore
#    VTK::RenderingOpenGL2
#    VTK::InteractionStyle
#    VTK::IOImage # if vtkPNGReader etc. are used
#    # ... other VTK modules
# )

# The core/common utilities are header-only for now, so no separate library to link.
# If they had .cpp files, we would create a library in src/core/common/CMakeLists.txt
# and link it: target_link_libraries(calibration_tool_app PRIVATE core_common)

# Add definitions from wxWidgets (e.g., for precompiled headers, resource compilation)
# This is often handled by include(${wxWidgets_USE_FILE}) in the root CMakeLists.txt
# If not, you might need: target_compile_definitions(calibration_tool_app PRIVATE ${wxWidgets_DEFINITIONS})
