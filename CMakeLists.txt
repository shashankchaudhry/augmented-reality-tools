cmake_minimum_required(VERSION 2.8)
project( camera )
find_package( OpenCV REQUIRED )

file(GLOB helloworld_SRC
    "*.h"
    "*.cpp"
)

add_executable(main ${helloworld_SRC})


target_link_libraries( main ${OpenCV_LIBS} )
