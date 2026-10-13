# DriftCore: engine-independent drift car physics. Unreal compiles the same
# sources through the DriftGame module; this file builds them standalone for
# the unit tests (and CI).
cmake_minimum_required(VERSION 3.16)
project(DriftCore CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(DriftCore STATIC
	src/CarSpec.cpp
	src/Json.cpp
	src/Tire.cpp
	src/Vehicle.cpp
)
target_include_directories(DriftCore PUBLIC include)
if(MSVC)
	target_compile_options(DriftCore PRIVATE /W4 /GR-)
else()
	# Unreal builds without exceptions/RTTI by default - keep the core honest.
	target_compile_options(DriftCore PRIVATE -Wall -Wextra -Wshadow -fno-exceptions -fno-rtti)
endif()

enable_testing()
add_executable(drift_tests tests/test_main.cpp)
target_link_libraries(drift_tests PRIVATE DriftCore)
target_compile_definitions(drift_tests PRIVATE DRIFT_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
add_test(NAME drift_tests COMMAND drift_tests)
