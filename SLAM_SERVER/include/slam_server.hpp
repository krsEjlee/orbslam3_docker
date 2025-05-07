#ifndef SLAM_SERVER_HPP
#define SLAM_SERVER_HPP
#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <thread>
#include <restbed>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <ImuTypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
// Remove std::filesystem
#include <sys/stat.h> // For directory creation in C++11
using json = nlohmann::json;
class SlamServer {
public:
    SlamServer(int port, const std::string& vocabPath, const std::string& settingsPath);
    ~SlamServer();
    void start();
    void stop();
private:
    // REST API endpoints
    void handleImage(const std::shared_ptr<restbed::Session>& session);
    void handlePosition(const std::shared_ptr<restbed::Session>& session);
    void handleGetPose(const std::shared_ptr<restbed::Session>& session);
    void handleStatus(const std::shared_ptr<restbed::Session>& session);
    void handleShutdown(const std::shared_ptr<restbed::Session>& session);
    // ORB-SLAM3 related methods
    bool initializeSLAM();
    Sophus::SE3f processCameraFrame(const cv::Mat& image, double timestamp, 
                                    const std::vector<ORB_SLAM3::IMU::Point>& imuData);
    json getCurrentPose();
    
    // Helper methods
    std::vector<ORB_SLAM3::IMU::Point> parseImuData(const std::string& imuHeader);
    void saveTrajectory();
    // Helper for creating directories in C++11
    bool createDirectory(const std::string& path);
    // Server settings (초기화 순서를 생성자 초기화 리스트 순서와 맞춤)
    int port_;
    std::string vocabPath_;
    std::string settingsPath_;
    bool isRunning_;
    std::unique_ptr<restbed::Service> service_;
    // 스레드 안전
    std::mutex poseMutex_;
    
    // ORB-SLAM3 시스템
    std::unique_ptr<ORB_SLAM3::System> slam_;
    
    // 현재 포즈 데이터 (초기화 순서를 생성자 초기화 리스트 순서와 맞춤)
    double lastTimestamp_;
    int frameCount_;
    Sophus::SE3f currentPose_;
    
    // SLAM 상태
    enum Status {
        NOT_INITIALIZED,
        INITIALIZING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };
    Status status_;
};
#endif // SLAM_SERVER_HPP