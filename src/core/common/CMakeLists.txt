# Placeholder for core/common utilities
#
# In a real project, if app_errors.h and profiler.h remain header-only,
# this CMakeLists.txt might primarily be used for defining an INTERFACE library
# to make these headers easily consumable by other CMake targets, e.g.:
#
# add_library(core_common INTERFACE)
# target_include_directories(core_common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
#
# If .cpp files were added (e.g., if to_string or parts of timed_call were not inline/template),
# it would look more like:
#
# add_library(core_common STATIC
#     app_errors.cpp
#     # profiler.cpp # if timed_call implementation details were in a .cpp
# )
# target_include_directories(core_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#
# For this specific task, as they are header-only, this file serves as a structural placeholder.
