# Any project's top most CMakeLists.txt must start by specifying a minimum CMake version
# using the cmake_minimum_required() command. This establishes policy settings and ensures
# that the following CMake functions are run with a compatible version of CMake.
cmake_minimum_required(VERSION 3.20.0)

# Set the name of the project, and stores it in
# the variable PROJECT_NAME. When called from the top-level
# CMakeLists.txt also stores the project name in the
# variable CMAKE_PROJECT_NAME.
project(rfm69)

# Add the path to FindWiringPi.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

# Find WiringPi
find_package(WiringPi REQUIRED)

# Include directories
include_directories(${WIRINGPI_INCLUDE_DIRS})

# Add an executable target called <app> to be built from the source files listed in the command invocation.
# Add the entry point <main.c> here for readability
add_executable(app main.c)

# Link libraries
target_link_libraries(app ${WIRINGPI_LIBRARIES})

# Link GCC Math library
target_link_libraries(app m)

# Add souce files to the executable target <app>
target_sources(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/rfm69.c
)



