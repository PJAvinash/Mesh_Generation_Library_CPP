cmake_minimum_required(VERSION 3.1)
project(MESH_GEN)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find CGAL package
find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})

# Find VTK package
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# Add executable
add_executable(mesh_generation src/main.cpp)

# Link CGAL and VTK libraries
target_link_libraries(mesh_generation ${CGAL_LIBRARIES})