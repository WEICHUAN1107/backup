cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(testvrp)

set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_definitions(-std=c++11)

find_package(OpenCV 3 REQUIRED)

link_directories(${PROJECT_SOURCE_DIR}/thirdParty/g2o/lib)
SET(G2O_LIBS
	g2o_core
	g2o_csparse_extension
	g2o_ext_freeglut_minimal
	g2o_interface
	g2o_opengl_helper
	g2o_parser
	g2o_solver_csparse
	g2o_solver_dense
	g2o_solver_pcg
	g2o_solver_slam2d_linear
	g2o_solver_structure_only
	g2o_stuff
	g2o_types_data
	g2o_types_icp
	g2o_types_sba
	g2o_types_sclam2d
	g2o_types_sim3
	g2o_types_slam2d
	g2o_types_slam3d
	g2o_types_slam3d_addons
)

include_directories(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/thirdParty/Eigen)

link_directories(${PROJECT_SOURCE_DIR}/lib)
SET(SLAM_LIBS myslam)

add_executable( testvrp src/main.cpp )
target_link_libraries( testvrp ${SLAM_LIBS} ${OpenCV_LIBS} ${G2O_LIBS})
