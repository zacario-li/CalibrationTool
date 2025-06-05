# This file can be used to add subdirectories for third-party libraries
# that are built using CMake or need specific CMake handling.

# Example for Eigen (if it's a subdirectory with its own CMakeLists.txt)
# add_subdirectory(eigen)

# Example for spdlog (if it's a subdirectory with its own CMakeLists.txt)
# add_subdirectory(spdlog)
# if(TARGET spdlog)
#   # Can set properties or aliases here if needed
# endif()

# For header-only libraries like nlohmann/json, you might just add their
# include directory in the main src/CMakeLists.txt or here if you create
# an INTERFACE library for them.
# add_library(nlohmann_json INTERFACE)
# target_include_directories(nlohmann_json INTERFACE nlohmann_json/include)

message(STATUS "Processing third_party/CMakeLists.txt (if it exists)")
