cmake_minimum_required(VERSION 3.10)
project(SLAM_SERVER)

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# OpenCV - 설치된 4.4.0 버전을 찾음
find_package(OpenCV 4.4.0 REQUIRED)
MESSAGE("OPENCV VERSION: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen3 라이브러리 추가
find_package(Eigen3 3.1.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Pangolin (ORB-SLAM3 시각화에 필요)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Restbed 설정
set(RESTBED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/restbed")
include_directories(${RESTBED_DIR}/include)
link_directories(${RESTBED_DIR}/build/library)

# nlohmann/json
# nlohmann/json은 헤더 전용 라이브러리이며, 시스템에 설치되어 있거나 include 디렉토리에 있어야 함
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ORB-SLAM3 설정
set(ORB_SLAM3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3")
include_directories(
    ${PROJECT_SOURCE_DIR}                # 현재 프로젝트 루트
    ${PROJECT_SOURCE_DIR}/include        # 현재 프로젝트 include 디렉토리
    ${ORB_SLAM3_DIR}                     # ORB-SLAM3 루트
    ${ORB_SLAM3_DIR}/include             # ORB-SLAM3 include
    ${ORB_SLAM3_DIR}/include/CameraModels # ORB-SLAM3 카메라 모델
    ${ORB_SLAM3_DIR}/Thirdparty          # Thirdparty 루트
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2    # DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/g2o      # g2o
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus   # Sophus
    ${EIGEN3_INCLUDE_DIR}                # Eigen3
)

# 소스 파일
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})

# 라이브러리 링크
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    restbed
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization  # Boost serialization 라이브러리
    -lcrypto               # Crypto 라이브러리
    -pthread              # 스레드 라이브러리
)

# 컴파일 결과물 출력 디렉토리 설정
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# config 디렉토리 복사 (필요한 경우)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/config)
file(GLOB CONFIG_FILES "${PROJECT_SOURCE_DIR}/config/*")
foreach(CONFIG_FILE ${CONFIG_FILES})
    file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/config)
endforeach()

# 설치 명령 추가
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)