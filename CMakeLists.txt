cmake_minimum_required (VERSION 3.14)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("Manipulator")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#set(COPPELIASIM_ROOT_DIR $ENV{COPPELIASIM_ROOT_DIR}) Sets path with windows backward slashes and it breaks everything!!!
file(TO_CMAKE_PATH $ENV{COPPELIASIM_ROOT_DIR} COPPELIASIM_ROOT_DIR)
if(NOT COPPELIASIM_ROOT_DIR)
    message(FATAL_ERROR "COPPELIASIM_ROOT_DIR environment variable is not found. Please set it manually to match the path to your installed simulator.")
endif()
set(COPPELIASIM_INCLUDE_DIR "${COPPELIASIM_ROOT_DIR}/programming/include")

set(QT_DLLS
    "${COPPELIASIM_ROOT_DIR}/platforms/qdirect2d.dll"
    "${COPPELIASIM_ROOT_DIR}/platforms/qminimal.dll"
    "${COPPELIASIM_ROOT_DIR}/platforms/qoffscreen.dll"
    "${COPPELIASIM_ROOT_DIR}/platforms/qwindows.dll"
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(deps/eigen)

add_executable(${PROJECT_NAME} "src/main.cpp" "include/Simulator.h" "src/Simulator.cpp"  "src/Simulation.cpp"
"${COPPELIASIM_INCLUDE_DIR}/simLib/simLib.h" "${COPPELIASIM_INCLUDE_DIR}/simLib/simLib.cpp")

target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR} ${COPPELIASIM_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} eigen)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms)
foreach (DLL ${QT_DLLS})
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL} $<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms
    )
endforeach()

file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" RES_PATH)
#message(${RES_PATH})
add_compile_definitions(RESOURCES_PATH="${RES_PATH}")