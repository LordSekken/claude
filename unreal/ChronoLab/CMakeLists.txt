# ChronoLab: Project Chrono vehicle experiments (see chrono_lab.cpp).
# Configure with -DChrono_DIR=<chrono build or install>/cmake
cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)
project(chrono_lab CXX)

find_package(Chrono COMPONENTS Vehicle CONFIG REQUIRED)

add_executable(chrono_lab chrono_lab.cpp)
if(MSVC)
  set_target_properties(chrono_lab PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()
target_link_libraries(chrono_lab PRIVATE ${CHRONO_TARGETS})
add_CHRONO_DLLS_copy_command()
