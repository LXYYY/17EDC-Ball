#project name
PROJECT(test)
#head file path
INCLUDE_DIRECTORIES(
	/home/pi/Desktop/New
	)
AUX_SOURCE_DIRECTORY(/home/pi/Desktop/New DIR_SRCS)

SET(TEST 
${DIR_SRCS}
)

link_directories("/usr/local/lib" 
			"/lib/arm-linux-gnueabihf")
add_executable(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} pthread)
target_link_libraries(${PROJECT_NAME} opencv_calib3d)
target_link_libraries(${PROJECT_NAME} opencv_core)
target_link_libraries(${PROJECT_NAME} opencv_features2d)
target_link_libraries(${PROJECT_NAME} opencv_flann)
target_link_libraries(${PROJECT_NAME} opencv_highgui)
target_link_libraries(${PROJECT_NAME} opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} opencv_imgproc)
target_link_libraries(${PROJECT_NAME} opencv_ml)
target_link_libraries(${PROJECT_NAME} opencv_objdetect)
target_link_libraries(${PROJECT_NAME} opencv_photo)
target_link_libraries(${PROJECT_NAME} opencv_shape)
target_link_libraries(${PROJECT_NAME} opencv_stitching)
target_link_libraries(${PROJECT_NAME} opencv_superres)
target_link_libraries(${PROJECT_NAME} opencv_videoio)
target_link_libraries(${PROJECT_NAME} opencv_video)
target_link_libraries(${PROJECT_NAME} opencv_videostab)


