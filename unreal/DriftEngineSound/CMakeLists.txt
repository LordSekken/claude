# DriftEngineSound: AngeTheGreat's engine-sim (MIT, https://github.com/ange-yaghi/engine-sim)
# as a headless engine-sound DLL with a C interface (include/drift_engine_sound.h).
#
# Builds only the simulation core + synthesizer + .mr scripting (Piranha); no delta-studio GUI,
# no Boost (std::filesystem shim in third_party/boost_shim), no googletest.
#
# Needs: an engine-sim checkout with the simple-2d-constraint-solver and piranha submodules
# (git submodule update --init dependencies/submodules/simple-2d-constraint-solver
#  dependencies/submodules/piranha) and flex/bison (winflexbison: win_flex/win_bison).
#
#   cmake -S unreal/DriftEngineSound -B C:/Dev/engine_sim_build -DENGINE_SIM_DIR=C:/Dev/engine-sim
#         -DFLEX_EXECUTABLE=.../win_flex.exe -DBISON_EXECUTABLE=.../win_bison.exe
#   cmake --build C:/Dev/engine_sim_build --config Release
#   C:/Dev/engine_sim_build/Release/des_render.exe <engines dir> <engine-sim dir> <out dir>
cmake_minimum_required(VERSION 3.18)
project(drift_engine_sound C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ENGINE_SIM_DIR "C:/Dev/engine-sim" CACHE PATH "engine-sim checkout (with submodules)")
set(ES ${ENGINE_SIM_DIR})
set(SUB ${ES}/dependencies/submodules)

if(MSVC)
  add_compile_options(/W1 /MP /bigobj /fp:fast)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX _USE_MATH_DEFINES)
endif()

# ---------------------------------------------------------------- simple-2d-constraint-solver
file(GLOB SCS_SOURCES ${SUB}/simple-2d-constraint-solver/src/*.cpp)
add_library(es_scs STATIC ${SCS_SOURCES})
set_target_properties(es_scs PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------- piranha (flex/bison front end)
find_package(FLEX REQUIRED)
find_package(BISON REQUIRED)
set(PIR ${SUB}/piranha)
set(PIR_GEN ${CMAKE_CURRENT_BINARY_DIR}/piranha_gen)
file(MAKE_DIRECTORY ${PIR_GEN})
flex_target(pir_lexer ${PIR}/flex-bison/scanner.l ${PIR_GEN}/scanner.auto.cpp
            DEFINES_FILE ${PIR_GEN}/scanner.auto.h COMPILE_FLAGS "--wincompat")
bison_target(pir_parser ${PIR}/flex-bison/specification.y ${PIR_GEN}/parser.auto.cpp
             DEFINES_FILE ${PIR_GEN}/parser.auto.h)
add_flex_bison_dependency(pir_lexer pir_parser)

file(GLOB PIR_SOURCES ${PIR}/src/*.cpp)
add_library(es_piranha STATIC ${PIR_SOURCES} ${BISON_pir_parser_OUTPUTS} ${FLEX_pir_lexer_OUTPUTS})
# FlexLexer.h must match the flex that generated the scanner: take it from the flex install.
get_filename_component(FLEX_BIN_DIR ${FLEX_EXECUTABLE} DIRECTORY)
find_path(FLEXLEXER_INCLUDE FlexLexer.h
  HINTS ${FLEX_BIN_DIR} ${FLEX_BIN_DIR}/../../flex/src ${FLEX_BIN_DIR}/../include
  NO_DEFAULT_PATH)
if(NOT FLEXLEXER_INCLUDE)
  set(FLEXLEXER_INCLUDE ${PIR}/dependencies/libraries/flex/include)
endif()
target_include_directories(es_piranha PUBLIC
  ${PIR_GEN} ${PIR}/include ${PIR}/src ${FLEXLEXER_INCLUDE}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost_shim)

# ---------------------------------------------------------------- engine-sim core
set(ES_CORE
  audio_buffer camshaft crankshaft combustion_chamber connecting_rod convolution_filter
  cylinder_bank cylinder_head delay_filter derivative_filter direct_throttle_linkage dynamometer
  engine exhaust_system feedback_comb_filter filter fuel function gas_system gaussian_filter
  governor ignition_module impulse_response intake jitter_filter leveling_filter low_pass_filter
  part piston piston_engine_simulator simulator standard_valvetrain starter_motor throttle
  transmission utilities valvetrain vehicle vehicle_drag_constraint vtec_valvetrain)
set(ES_CORE_SOURCES)
foreach(f ${ES_CORE})
  list(APPEND ES_CORE_SOURCES ${ES}/src/${f}.cpp)
endforeach()
# synthesizer.cpp includes delta.h (the GUI engine) without using it: compile a copy without it.
# The copy lives in the build dir; its "../include/x.h" includes resolve through ${ES}/src.
file(READ ${ES}/src/synthesizer.cpp SYNTH_SRC)
string(REPLACE "#include \"../include/delta.h\"" "" SYNTH_SRC "${SYNTH_SRC}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/es_patched/synthesizer.cpp "${SYNTH_SRC}")
list(APPEND ES_CORE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/es_patched/synthesizer.cpp)

file(GLOB ES_SCRIPT_SOURCES ${ES}/scripting/src/*.cpp)
add_library(es_core STATIC ${ES_CORE_SOURCES} ${ES_SCRIPT_SOURCES})
target_include_directories(es_core PUBLIC ${ES}/src ${ES}/include ${SUB}
  ${SUB}/simple-2d-constraint-solver/include)
target_compile_definitions(es_core PUBLIC ATG_ENGINE_SIM_PIRANHA_ENABLED)
target_link_libraries(es_core PUBLIC es_scs es_piranha)

# ---------------------------------------------------------------- the DLL
add_library(DriftEngineSound SHARED src/drift_engine_sound.cpp include/drift_engine_sound.h)
target_include_directories(DriftEngineSound PUBLIC include)
target_compile_definitions(DriftEngineSound PRIVATE DES_EXPORTS)
target_link_libraries(DriftEngineSound PRIVATE es_core)

# ---------------------------------------------------------------- WAV render / test tool
add_executable(des_render tools/des_render.cpp)
target_link_libraries(des_render PRIVATE DriftEngineSound)
