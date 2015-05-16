cmake_minimum_required(VERSION 2.8)
project( camera )
find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
FIND_PACKAGE ( Threads REQUIRED )
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

file(GLOB helloworld_SRC
    "*.h"
    "*.cpp"
)

add_executable(main ${helloworld_SRC})


target_link_libraries( main ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
