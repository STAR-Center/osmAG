cmake_minimum_required(VERSION 3.2)
project (TOPO_GRAPH_2D)
set (CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(OpenCV 3.0 REQUIRED)
find_package(CGAL COMPONENTS Core Boost)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(Eigen3)

find_package(PCL 1.7 REQUIRED)# COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}) 
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

if ( CGAL_FOUND )

  include( ${CGAL_USE_FILE} )

  include( CGAL_CreateSingleSourceCGALProgram )

else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()


# if ( EIGEN3_FOUND )
#   include( ${EIGEN3_USE_FILE} )
# endif()

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})


#add_subdirectory (dir)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library (topo_graph_2d STATIC  /
  src/VoriConfig.cpp /
  src/VoriGraph.cpp /
  src/TopoGraph.cpp /
  src/RoomDect.cpp /
  src/roomGraph.cpp /
  src/Denoise.cpp /
  src/passageSearch.cpp /
  src/cgal/CgalVoronoi.cpp /
  src/cgal/AlphaShape.cpp /
  src/cgal/AlphaShapeRemoval.cpp /
  src/qt/QImageVoronoi.cpp /
        src/AreaGenerate.cpp /
        src/Area_osm.cpp)


include_directories (${TOPO_GRAPH_2D_SOURCE_DIR}/include)
include_directories(${CGAL_INCLUDE_DIR} ${QT_INCLUDE_DIR})


include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(osm_generate test/test_osm_generation.cpp)
include_directories (${TOPO_GRAPH_2D_SOURCE_DIR}/include)

message(STATUS " cgal: ${CGAL_LIBRARIES} ")

TARGET_LINK_LIBRARIES(osm_generate topo_graph_2d ${OpenCV_LIBS} ${QT_LIBRARIES} ${CGAL_LIBRARIES})
