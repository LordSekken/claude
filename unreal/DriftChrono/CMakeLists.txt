# DriftChrono: Project Chrono vehicle physics as a DLL with a C interface
# (include/drift_chrono.h) for DriftGame and test tools.
#   cmake -S unreal/DriftChrono -B build_dc -DChrono_DIR=<chrono build>/cmake
#   cmake --build build_dc --config Release
cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)
project(drift_chrono CXX)

find_package(Chrono COMPONENTS Vehicle CONFIG REQUIRED)

add_library(DriftChrono SHARED src/drift_chrono.cpp include/drift_chrono.h)
target_include_directories(DriftChrono PUBLIC include)
target_compile_definitions(DriftChrono PRIVATE DRIFT_CHRONO_EXPORTS)
if(MSVC)
  set_target_properties(DriftChrono PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()
target_link_libraries(DriftChrono PRIVATE ${CHRONO_TARGETS})

# Smoke test: flat ground, drive, steer, hit a wall.
add_executable(dc_smoke tests/dc_smoke.cpp)
target_link_libraries(dc_smoke PRIVATE DriftChrono)
if(MSVC)
  set_target_properties(dc_smoke PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()
add_CHRONO_DLLS_copy_command()
