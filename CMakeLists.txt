cmake_minimum_required (VERSION 3.14)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("Manipulator")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(COPPELIASIM_ROOT_DIR $ENV{COPPELIASIM_ROOT_DIR})
if(NOT COPPELIASIM_ROOT_DIR)
    message(FATAL_ERROR "COPPELIASIM_ROOT_DIR environment variable is not found. Please set it manually to match the path to your installed simulator.")
endif()
set(COPPELIASIM_INCLUDE_DIR "${COPPELIASIM_ROOT_DIR}/programming/include")

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(deps/eigen)

add_executable(${PROJECT_NAME} "src/main.cpp" "include/Simulator.h" "src/Simulator.cpp")

target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR} ${COPPELIASIM_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} eigen)