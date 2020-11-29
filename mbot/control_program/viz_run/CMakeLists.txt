cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gtk+3
# ref: https://gist.github.com/fracek/3323924
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(GTK3 REQUIRED gtk+-3.0)
INCLUDE_DIRECTORIES(${GTK3_INCLUDE_DIRS})
LINK_DIRECTORIES(${GTK3_LIBRARY_DIRS})
ADD_DEFINITIONS(${GTK3_CFLAGS_OTHER})

# Xlib
find_package(X11 REQUIRED)
link_libraries(${X11_LIBRARIES})
include_directories(${X11_INCLUDE_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


pkg_check_modules(LIBSERIAL REQUIRED libserial)
include_directories("/usr/include/libserial")
link_directories("/usr/include/libserial")


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
add_executable(main main.cc grid.cc viz.cc)
TARGET_LINK_LIBRARIES(main ${GTK3_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(main ${LIBSERIAL_LIBRARIES})
