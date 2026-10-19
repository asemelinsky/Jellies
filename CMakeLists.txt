cmake_minimum_required(VERSION 3.16)

# Static MSVC runtime for raylib and the game alike (one .exe, no redistributables)
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

project(Jelly LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# raylib 5.5, built as a static library. To build offline, pass
# -DFETCHCONTENT_SOURCE_DIR_RAYLIB=/path/to/raylib-5.5
include(FetchContent)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(OPENGL_VERSION "3.3" CACHE STRING "" FORCE)
FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(raylib)

add_executable(jelly WIN32
    src/main.cpp
    src/shapes.cpp
    src/physics.cpp
    src/render.cpp
    src/audio.cpp
    src/splash.cpp
    src/eyes.cpp
    src/ui.cpp)
target_link_libraries(jelly PRIVATE raylib)

if(WIN32)
    # Application icon + version info (MSVC rc.exe and MinGW windres)
    enable_language(RC)
    target_sources(jelly PRIVATE assets/jellies.rc)
    target_include_directories(jelly PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/assets)
    set_source_files_properties(assets/jellies.rc PROPERTIES OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/icon.ico)
endif()

if(MSVC)
    # C++-only flags: keep them away from the resource compiler
    target_compile_options(jelly PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:/W3;/O2;/fp:fast;/utf-8>")
    # WIN32 subsystem (no console window) while keeping a regular main()
    target_link_options(jelly PRIVATE /ENTRY:mainCRTStartup)
else()
    target_compile_options(jelly PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-O2>")
    if(WIN32)
        target_link_options(jelly PRIVATE -static -static-libgcc -static-libstdc++ -mwindows)
    else()
        target_link_options(jelly PRIVATE -static-libgcc -static-libstdc++)
    endif()
endif()

if(WIN32)
    set_target_properties(jelly PROPERTIES OUTPUT_NAME "Jelly")
endif()
