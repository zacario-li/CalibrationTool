cmake_minimum_required(VERSION 3.16)
project(CalibrationToolCpp VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- Find Packages ---
# wxWidgets
find_package(wxWidgets COMPONENTS core base REQUIRED)
include(${wxWidgets_USE_FILE})
# Add wxWidgets definitions, e.g., for precompiled headers or resource compilation
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${wxWidgets_CXX_FLAGS}") # Handled by wxWidgets_USE_FILE typically

# OpenCV
find_package(OpenCV REQUIRED)

# VTK
find_package(VTK REQUIRED)

# Open3D
# Assuming Open3D is installed in a way CMake can find it
# May require Open3D_DIR to be set if not in standard paths
find_package(Open3D REQUIRED)

# Eigen (Option 1: Find as a package if installed system-wide)
# find_package(Eigen3 REQUIRED)
# if(Eigen3_FOUND)
#   include_directories(${Eigen3_INCLUDE_DIRS})
# endif()
# (Option 2: Add as subdirectory - preferred if bundled in third_party)
# add_subdirectory(third_party/eigen) # Assuming Eigen is in third_party/eigen

# spdlog (Option 1: Find as a package)
# find_package(spdlog REQUIRED)
# (Option 2: Add as subdirectory)
# add_subdirectory(third_party/spdlog)

# nlohmann_json (Typically header-only, added via third_party)
# add_subdirectory(third_party/nlohmann_json) # if it has a CMakeLists.txt

# SQLite3
find_package(SQLite3 REQUIRED)

# --- Subdirectories ---
add_subdirectory(src)
# add_subdirectory(third_party) # If third_party has its own CMakeLists.txt to build/install its libs

# --- Resources ---
# Example for handling resources, actual implementation might vary
# file(GLOB_RECURSE RESOURCE_FILES "resources/*")
# add_custom_target(CopyResources ALL
#    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/resources $<TARGET_FILE_DIR:calibration_tool_app>/resources
#    DEPENDS calibration_tool_app # Assuming your executable is named calibration_tool_app
# )

# --- Testing (Optional for initial setup) ---
# enable_testing()
# add_subdirectory(tests)

message(STATUS "wxWidgets_INCLUDE_DIRS: ${wxWidgets_INCLUDE_DIRS}")
message(STATUS "wxWidgets_LIBRARIES: ${wxWidgets_LIBRARIES}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
message(STATUS "VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")
message(STATUS "VTK_LIBRARIES: ${VTK_LIBRARIES}") # This might be complex, VTK::xxx targets are preferred
message(STATUS "Open3D_INCLUDE_DIRS: ${Open3D_INCLUDE_DIRS}")
message(STATUS "Open3D_LIBRARIES: ${Open3D_LIBRARIES}")
message(STATUS "SQLite3_INCLUDE_DIRS: ${SQLite3_INCLUDE_DIRS}")
message(STATUS "SQLite3_LIBRARIES: ${SQLite3_LIBRARIES}")
