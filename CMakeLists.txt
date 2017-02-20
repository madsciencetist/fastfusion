cmake_minimum_required(VERSION 2.8)

project(Fastfusion)

set(CMAKE_BUILD_TYPE Release)

#OpenCV
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_calib3d)
find_package(OpenCV REQUIRED)

#Qt4 This is dirty... FIXME
FIND_PACKAGE(Qt4 REQUIRED)
include_directories("/usr/include/qt4/QtXml")
include_directories("/usr/include/qt4")
include_directories("/usr/include/qt4/Qt")
include_directories("/usr/include/qt4/QtCore")
include_directories("/usr/include/qt4/QtXml")
include_directories("/usr/include/qt4/QtGui")
include_directories("/usr/include/qt4/QtOpenGL")
ADD_DEFINITIONS(${QT_DEFINITIONS})

#QGL This is also dirty... FIXME
if(EXISTS "${ROOT}/usr/lib/x86_64-linux-gnu/libQGLViewer.so")
   message(STATUS "Found qglviewer2, linking QGLViewer")
   set(QGLVIEWER QGLViewer)
else()
   message(STATUS "Did not find qglviewer2, linking qglviewer-qt4")
   set(QGLVIEWER QGLViewer-qt4)
endif()

#CMake Flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -frounding-math -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

#AVX2 COMPILING SUPPORT 
execute_process(COMMAND cat /proc/cpuinfo OUTPUT_VARIABLE CPU_INFO)
string(REGEX MATCHALL "avx2" AVX_STRING ${CPU_INFO})
list(LENGTH AVX_STRING AVX_STRING_LENGTH)
if(${AVX_STRING_LENGTH} GREATER 0)
   message(STATUS "Compiling with AVX2 support")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
else()
   message(STATUS "Compiling without AVX2 support")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse4.2 -funroll-loops")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -msse4.2 -funroll-loops")
endif()

#Exec and lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Include
include_directories(${PROJECT_SOURCE_DIR}/include)
#Dirty FIXME
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories("/usr/include/eigen3")

add_subdirectory(src)

