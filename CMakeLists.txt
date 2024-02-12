# oal: Obstacle Avoidance Library

# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
cmake_policy(SET CMP0057 NEW)  # Set policy CMP0057 to NEW to support IN_LIST.


# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()


# Define library. Only source files here!
project(oal VERSION 0.1 LANGUAGES CXX)

option(BUILD_TESTS "Compile tests" ON)

add_library(oal SHARED
        src/path_planner.cpp
        src/helper_functions.cpp
        src/node.cpp
        src/obstacle.cpp
        )

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(std_msgs REQUIRED)
# include_directories(${std_msgs_INCLUDE_DIRS})

# set_target_properties(oal PROPERTIES LINKER_LANGUAGE CXX)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(oal PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/oal>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(oal
        PUBLIC
        cxx_auto_type
        cxx_range_for
        cxx_variadic_templates
        )

# Depend on a library that we defined in the top-level file
#target_link_libraries(lib
#    boost
#    MyOtherLibrary)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS oal EXPORT oalConfig
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/oal DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT oalConfig DESTINATION share/oal/cmake)

# This makes the project importable from the build directory
export(TARGETS oal FILE oalConfig.cmake)


if (BUILD_TESTS)
    # Every library has unit tests, of course
    add_executable(oal_test test/test.cc)

    target_link_libraries(oal_test oal)

    add_test(oal_test oal_test)
endif (BUILD_TESTS)
