cmake_minimum_required(VERSION 3.25)

project(smu_ebs_sil LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
  message(FATAL_ERROR
    "This assignment has one supported build configuration: Release")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

find_package(Threads REQUIRED)

add_executable(sil_tests
  given/ebs.cpp
  given/smu.cpp
  src/sil.cpp
  tests/sil_tests.cpp
)

target_include_directories(sil_tests PRIVATE
  given
  src
)

target_compile_options(sil_tests PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wshadow
  -Wconversion
  -Wsign-conversion
)

target_link_libraries(sil_tests PRIVATE Threads::Threads)

add_test(NAME sil_tests COMMAND sil_tests)
set_tests_properties(sil_tests PROPERTIES TIMEOUT 30)
