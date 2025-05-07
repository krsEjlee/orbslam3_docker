cmake_minimum_required(VERSION 3.10)
project(slam_server)

# 빌드 타입 설정
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# 컴파일러 옵션 설정
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# C++11 이상 사용
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DCOMPILEDWITHC11)

# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
# Set ORB-SLAM3 directory
set(ORB_SLAM3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3)

# Set Thirdparty directory
set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty)

# Restbed settings
set(RESTBED_DIR ${THIRDPARTY_DIR}/restbed)
include_directories(${RESTBED_DIR}/include)
link_directories(${RESTBED_DIR}/build/library)

# Find required packages
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
# Restbed will be included from Thirdparty, don't need to find_package

# Define paths to include
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${THIRDPARTY_DIR}  # 추가된 부분
  ${THIRDPARTY_DIR}/json/include

)

# Collect source files
file(GLOB SOURCES "src/*.cpp")

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  restbed  # 이 부분이 바뀐 부분 (restbed_LIBRARY 대신)
  -lboost_serialization
  -lcrypto
  -pthread
)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Installation
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)