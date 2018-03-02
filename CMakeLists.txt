cmake_minimum_required (VERSION 3.0)

project (scan3d CXX)


OPTION(SCAN3D_PARALLEL "Use parallel code" ON)


find_package(OpenCV REQUIRED core imgproc highgui calib3d)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIB_DIR})
  set(SCAN3D_LIBS ${SCAN3D_LIBS} ${OpenCV_LIBS})
endif()


if( SCAN3D_PARALLEL )
    find_package (Threads)
    if( THREADS_FOUND )
        add_definitions(-DUSE_THREADS)
        set(SCAN3D_LIBS ${SCAN3D_LIBS} ${CMAKE_THREAD_LIBS_INIT})
    endif()
endif()


include_directories(${scan3d_SOURCE_DIR})

add_executable(scan3d leopard.cpp triangulation.cpp unsync.cpp)
target_link_libraries( scan3d ${SCAN3D_LIBS} )
install(TARGETS scan3d DESTINATION bin)

add_executable(simple leopard.cpp triangulation.cpp simple.cpp)
target_link_libraries( simple ${SCAN3D_LIBS})
install(TARGETS simple DESTINATION bin)

add_executable(couleur leopard.cpp triangulation.cpp couleur.cpp)
target_link_libraries( couleur ${SCAN3D_LIBS} )
install(TARGETS couleur DESTINATION bin)

