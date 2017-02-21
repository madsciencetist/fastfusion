add_subdirectory(camerautils)
add_subdirectory(auxiliary)
add_subdirectory(fusion)

#This is dirty.. again .hpp in src but did not successfuly move it without breaking Qt FIXME
QT4_WRAP_CPP(onlinefusion_HEADERS_MOC onlinefusionviewer.hpp)

#Binaries
add_executable(onlinefusion
   onlinefusionviewer_main.cpp
   onlinefusionviewer.cpp
   ${onlinefusion_HEADERS_MOC}
   )
target_link_libraries(onlinefusion
   geometryfusion_mipmap_cpu geometryfusion_aos
   camerautils
   auxiliary
   ${OpenCV_LIBS}
   ${QGLVIEWER}
   QtCore QtGui QtOpenGL
   GL GLU glut GLEW
   boost_thread
   boost_system
   )
add_executable(onlinefusion2
   main_test.cpp
   onlinefusionviewer.cpp
   ${onlinefusion_HEADERS_MOC}
   )
target_link_libraries(onlinefusion2
   geometryfusion_mipmap_cpu geometryfusion_aos
   camerautils
   auxiliary
   ${OpenCV_LIBS}
   ${QGLVIEWER}
   QtCore QtGui QtOpenGL
   GL GLU glut GLEW
   boost_thread
   boost_system
   )

#Add lib
add_library(${PROJECT_NAME} SHARED
   onlinefusionviewer.cpp
   ${onlinefusion_HEADERS_MOC}
   )

target_link_libraries(${PROJECT_NAME}
   geometryfusion_mipmap_cpu geometryfusion_aos
   camerautils
   auxiliary
   ${OpenCV_LIBS}
   ${QGLVIEWER}
   QtCore QtGui QtOpenGL
   GL GLU glut GLEW
   boost_thread
   boost_system
   )
