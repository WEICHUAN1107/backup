cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(testvrp)

set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

add_definitions("-std=c++0x -O3 -DDEBUG") # 优化对运行速度有肉眼可见的影响！ -Ofast 可能有NaN问题 -DNDEBUG -DDEBUG 只影响assert() 不影响CV_Assert()

find_package(OpenCV 3 REQUIRED)

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
link_directories(/usr/local/lib)
SET(G2O_LIBS g2o_ext_freeglut_minimal g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface
    g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_types_slam3d_addons
    g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library( myslam SHARED
    src/util.cpp
    src/frame.cpp
    src/point.cpp
    src/viewer.cpp
    src/framecontrol.cpp
    src/initializer.cpp
    src/pointselector.cpp
    src/pointmatcher.cpp
    src/mapping.cpp
    src/optimizer.cpp
    src/vrprocess.cpp
    src/opticalflow.cpp
    src/subdiv.cpp
    src/manager.cpp
    src/tracking.cpp
	src/ground.cpp
)

target_link_libraries( myslam
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${G2O_LIBS}
)

add_executable( testvrp src/main.cpp )
target_link_libraries( testvrp myslam )


