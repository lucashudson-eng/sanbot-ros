cmake_minimum_required(VERSION 3.22.1)
project(orbbec_body)

add_library(orbbec_body SHARED orbbec_body_jni.cpp)

find_library(log-lib log)
find_library(dl-lib dl)

set(JNI_LIBS ${CMAKE_SOURCE_DIR}/../jniLibs/armeabi-v7a)

add_library(OpenNI SHARED IMPORTED)
set_target_properties(OpenNI PROPERTIES IMPORTED_LOCATION ${JNI_LIBS}/libOpenNI.so)

add_library(XnDeviceSensorV2 SHARED IMPORTED)
set_target_properties(XnDeviceSensorV2 PROPERTIES IMPORTED_LOCATION ${JNI_LIBS}/libXnDeviceSensorV2.so)

target_link_libraries(orbbec_body
        OpenNI
        XnDeviceSensorV2
        ${log-lib}
        ${dl-lib})
