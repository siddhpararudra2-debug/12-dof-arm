cmake_minimum_required(VERSION 3.16)
project(aether12 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(aether12_core
  src/robot_model.cpp
  src/kinematics.cpp
  src/trajectory.cpp
  src/safety.cpp
  src/hardware.cpp
  src/collision.cpp
)
target_include_directories(aether12_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(aether12_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

add_executable(aether12_demo examples/aether12_demo.cpp)
target_link_libraries(aether12_demo PRIVATE aether12_core)
target_compile_options(aether12_demo PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(aether12_tests tests/test_core.cpp)
target_link_libraries(aether12_tests PRIVATE aether12_core)
target_compile_options(aether12_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME aether12_core_tests COMMAND aether12_tests)

install(TARGETS aether12_core aether12_demo aether12_tests)
install(DIRECTORY include/ DESTINATION include)
