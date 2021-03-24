cmake_minimum_required(VERSION 3.17)
project(myClionProject)

set(CMAKE_CXX_STANDARD 14)
set( CMAKE_BUILD_TYPE Debug )

# OpenCV
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories( ${OpenCV_LIBRARY_DIRS} )
add_definitions( ${OpenCV_DEFINATIONS} )

# eigen
include_directories( "/usr/include/eigen3/" )

# PCL
find_package( PCL REQUIRED COMPONENT common io visualization features surface keypoints filters )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Pangolin
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )
link_directories( ${Pangolin_LIBRARY_DIRS} )
add_definitions( ${Pangolin_DEFINATIONS} )


add_executable( helloworld src/helloworld.cpp )
target_link_libraries( helloworld ${OpenCV_LIBS} )

add_executable(joinMap src/joinMap.cpp)
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(write_pcd src/write_pcd.cpp)
target_link_libraries( write_pcd ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(visualization_pcd src/visualization_pcd.cpp)
target_link_libraries( visualization_pcd ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(concatenating_two_clouds src/concatenating_two_clouds.cpp)
target_link_libraries( concatenating_two_clouds ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(compute_normal src/compute_normal.cpp)
target_link_libraries( compute_normal ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(matrix_transformations src/matrix_transformations.cpp)
target_link_libraries( matrix_transformations ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(saving_png src/saving_png.cpp)
target_link_libraries( saving_png ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(computing_centroid src/computing_centroid.cpp)
target_link_libraries( computing_centroid ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(upsampling src/upsampling.cpp)
target_link_libraries( upsampling ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(uniform_sampling src/uniform_sampling.cpp)
target_link_libraries( uniform_sampling ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(down_sampling src/down_sampling.cpp)
target_link_libraries( down_sampling ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(normal_estimation src/normal_estimation.cpp)
target_link_libraries( normal_estimation ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )

add_executable(integral_images src/integral_images.cpp)
target_link_libraries( integral_images ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(k-d_tree src/k-d_tree.cpp)
target_link_libraries( k-d_tree ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(octree src/octree.cpp)
target_link_libraries( octree ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(compressing_decompressing src/compressing_decompressing.cpp)
target_link_libraries( compressing_decompressing ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(detecting_changes src/detecting_changes.cpp)
target_link_libraries( detecting_changes ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(passthrough_filter src/passthrough_filter.cpp)
target_link_libraries( passthrough_filter ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(conditional_removal src/conditional_removal.cpp)
target_link_libraries( conditional_removal ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(radius_based_outlier_removal src/radius_based_outlier_removal.cpp)
target_link_libraries( radius_based_outlier_removal ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(statistical_outlier_removal src/statistical_outlier_removal.cpp)
target_link_libraries( statistical_outlier_removal ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(undistortImage src/undistortImage.cpp)
target_link_libraries(undistortImage ${OpenCV_LIBS})

add_executable(undistortImage_opencv src/undistortImage_opencv.cpp)
target_link_libraries(undistortImage_opencv ${OpenCV_LIBS})

add_executable(stereoVision src/stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(sgbm src/sgbm.cpp)
target_link_libraries(sgbm ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(bm src/bm.cpp)
target_link_libraries(bm ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(surface_smoothing src/surface_smoothing.cpp)
target_link_libraries( surface_smoothing ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

add_executable(read_pcd src/read_pcd.cpp)
target_link_libraries( read_pcd ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(read_image src/read_image.cpp)
target_link_libraries( read_image ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(imageBasics src/imageBasics.cpp)
target_link_libraries( imageBasics ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(eigenMatrix src/eigenMatrix.cpp)
target_link_libraries( eigenMatrix ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(visualizeGeometry src/visualizeGeometry.cpp)
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )

