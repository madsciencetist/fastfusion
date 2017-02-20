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
