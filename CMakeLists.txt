cmake_minimum_required (VERSION 3.0)

project (scan3d)


find_package(OpenCV REQUIRED core imgproc highgui calib3d)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIB_DIR})
  #message(STATUS "Found OpenCV")
endif()

include_directories(${scan3d_SOURCE_DIR})

add_executable(scan3d leopard.cpp triangulation.cpp unsync.cpp)
target_link_libraries( scan3d ${OpenCV_LIBS} )
install(TARGETS scan3d DESTINATION bin)

add_executable(simple leopard.cpp triangulation.cpp simple.cpp)
target_link_libraries( simple ${OpenCV_LIBS} )
install(TARGETS simple DESTINATION bin)

add_executable(couleur leopard.cpp triangulation.cpp couleur.cpp)
target_link_libraries( couleur ${OpenCV_LIBS} )
install(TARGETS couleur DESTINATION bin)

