add_subdirectory(camerautils)
add_subdirectory(auxiliary)
#add_subdirectory(auxiliaryGPU)
#add_subdirectory(libelas)
#add_subdirectory(output)
add_subdirectory(fusion)
#add_subdirectory(fusionGPU)
#add_subdirectory(sophus)
#add_subdirectory(filesystem) 


include_directories("/usr/include/x86_64-linux-gnu/qt5/QtXml")
include_directories("/usr/include/x86_64-linux-gnu/qt5")
include_directories("/usr/include/x86_64-linux-gnu/qt5/Qt")
include_directories("/usr/include/x86_64-linux-gnu/qt5/QtCore")
include_directories("/usr/include/x86_64-linux-gnu/qt5/QtXml")
include_directories("/usr/include/x86_64-linux-gnu/qt5/QtGui")
include_directories("/usr/include/x86_64-linux-gnu/qt5/QtOpenGL")
ADD_DEFINITIONS(${QT_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

if(EXISTS "${ROOT}/usr/lib/x86_64-linux-gnu/libQGLViewer.so.2")
message(STATUS "Found qglviewer2, linking QGLViewer")
set(QGLVIEWER "${ROOT}/usr/lib/x86_64-linux-gnu/libQGLViewer.so.2")
else()
message(STATUS "Did not find qglviewer2, linking qglviewer-qt4")
#set(QGLVIEWER QGLViewer-qt4)
endif()


# add_executable(cameracalibration cameracalib_main.cpp)
# target_link_libraries(cameracalibration 
# opencv_core opencv_imgproc opencv_highgui opencv_features2d opencv_calib3d
# )
# 
# add_executable(rectify_known rectify_stereo_known_main.cpp)
# target_link_libraries(rectify_known 
# opencv_core opencv_imgproc opencv_highgui opencv_features2d opencv_calib3d
# )

find_package(catkin COMPONENTS
    rosbag
    cv_bridge
    depth_image_proc
    eigen_conversions
    image_geometry
    message_filters
    pcl_ros
    sensor_msgs
    tf
    tf2_ros
    tf2_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

FIND_PACKAGE(Qt5 COMPONENTS Xml Core Xml Gui OpenGL Widgets)
QT5_WRAP_CPP(onlinefusion_HEADERS_MOC onlinefusionviewer.hpp)

add_executable(onlinefusion
    rosbag_loader.cpp
    onlinefusionviewer_main.cpp
    onlinefusionviewer.cpp
    depth_registration.cpp
    depth_registration.h
    pointcloud_processing.h
    pointcloud_processing.cpp
    ${onlinefusion_HEADERS_MOC}
)
target_link_libraries(onlinefusion
    geometryfusion_mipmap_cpu geometryfusion_aos
    camerautils
    auxiliary
    ${OpenCV_LIBS}
    ${QGLVIEWER}
    Qt5Core Qt5Gui Qt5OpenGL Qt5Widgets
    GL GLU glut GLEW
    boost_thread
    boost_system
    ${catkin_LIBRARIES}
)
