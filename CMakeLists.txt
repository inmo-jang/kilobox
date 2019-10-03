cmake_minimum_required(VERSION 2.6)

project(kilobox)

if(UNIX)
	set(kilobox_INSTALL_BY_DEFAULT ON)
else(UNIX)
	set(kilobox_INSTALL_BY_DEFAULT OFF)
endif(UNIX)
option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)

# Use C++11
set(CMAKE_CXX_FLAGS "-std=c++11")

set(kilobox_VERSION 1.0.0)
set(BOX2D_VERSION 2.3.0)
set(LIB_INSTALL_DIR lib${LIB_SUFFIX})
set(BOX2D_SOURCE_DIR src)

add_subdirectory(src/kilobox)
add_subdirectory(src/Box2D/Box2D)
add_subdirectory(src/Box2D/Testbed)
add_subdirectory(src/Box2D/glui)

find_package(OpenGL REQUIRED)
if (APPLE)
else(APPLE)
find_package(X11 REQUIRED)
add_subdirectory(src/Box2D/freeglut)
endif(APPLE)




