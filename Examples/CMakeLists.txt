# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
        Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/MaskRCNN)

add_executable(mask_rcnn MaskRCNN/mask_rcnn.cc)
target_link_libraries(mask_rcnn ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/DynamicExtractor)
include_directories(DynamicExtractor)
add_executable(test DynamicExtractor/test.cc)
target_link_libraries(test ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/OpticalFlow)
add_executable(optflow OpticalFlow/fback.cc)
target_link_libraries(optflow ${PROJECT_NAME})
