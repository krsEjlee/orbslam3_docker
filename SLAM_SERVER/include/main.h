#ifndef SLAM_SERVER_MAIN_H
#define SLAM_SERVER_MAIN_H

#include <restbed>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <System.h>

// Ping 핸들러 - 서버 연결 확인용 간단한 핸들러
void ping_handler(const std::shared_ptr<restbed::Session>& session);

// 캘리브레이션 핸들러 - 캘리브레이션 파라미터 업데이트
void calibration_handler(const std::shared_ptr<restbed::Session>& session);

// 트래킹 핸들러 - 이미지 데이터 처리 및 카메라 자세 추정
void tracking_handler(const std::shared_ptr<restbed::Session>& session);

// 종료 핸들러 - 서버 종료
void shutdown_handler(const std::shared_ptr<restbed::Session>& session);

#endif // SLAM_SERVER_MAIN_H