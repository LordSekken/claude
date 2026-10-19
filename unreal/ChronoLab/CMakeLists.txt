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

# Suspension kinematics/compliance analysis (camber/toe curves, caster/KPI/trail, roll centre,
# motion ratios, wheel/ARB rates, steering ratio and Ackermann) of a converted car.
add_executable(kinematics kinematics.cpp)
if(MSVC)
  set_target_properties(kinematics PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()
target_link_libraries(kinematics PRIVATE ${CHRONO_TARGETS})

# Single-tyre Pac02 checks (curves vs slip/load/camber, combined slip, rig sign checks) with pass/fail targets:
#   tyre_test <ChronoCars root> <car id> [AC car data dir] [--no-rig]
add_executable(tyre_test tyre_test.cpp)
if(MSVC)
  set_target_properties(tyre_test PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()
target_link_libraries(tyre_test PRIVATE ${CHRONO_TARGETS})
