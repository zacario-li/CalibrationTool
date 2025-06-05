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
    # ui/components/image_panel.cpp
    # ...
    # core/calib/calibration_algorithms.cpp
    # ...
)

# Include directories
target_include_directories(calibration_tool_app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}      # For headers in src/
    ${wxWidgets_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${SQLite3_INCLUDE_DIRS}
    # ${Eigen3_INCLUDE_DIRS} # If found as package
    # Add include dirs for spdlog, nlohmann_json if needed (often handled by their own CMake targets if added as subdir)
    "${CMAKE_SOURCE_DIR}/third_party/eigen" # Example if Eigen is just headers in third_party
    "${CMAKE_SOURCE_DIR}/third_party/spdlog/include" # Example for spdlog headers
    "${CMAKE_SOURCE_DIR}/third_party/nlohmann_json/include" # Example for nlohmann_json headers
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
