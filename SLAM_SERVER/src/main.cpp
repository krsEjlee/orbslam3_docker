#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <csignal>
#include <iomanip>
#include <restbed>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <System.h>
#include <ImuTypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>

// 네임스페이스 선언
using namespace std;
using namespace restbed;
using namespace ORB_SLAM3;
using json = nlohmann::json;
namespace fs = std::filesystem;

// 사용자 세션 정보 구조체
struct UserSession {
    string sessionId;
    string shipId;
    string username;
    string slamType;  // "mono" 또는 "mono+imu"
    shared_ptr<System> slamSystem;
    string vocabPath;
    string settingsPath;
    int frameCounter;
    string trajectoryBasePath;
    chrono::time_point<chrono::system_clock> lastActivity;
    
    UserSession(const string& id, const string& ship, const string& user, const string& type, 
                const string& vocab, const string& settings)
        : sessionId(id), shipId(ship), username(user), slamType(type),
          vocabPath(vocab), settingsPath(settings), frameCounter(0) {
          
        trajectoryBasePath = "trajectories/" + shipId + "_" + sessionId;
        lastActivity = chrono::system_clock::now();
        
        // 디렉토리 생성
        fs::create_directories(trajectoryBasePath);
        
        // SLAM 타입에 따라 시스템 생성
        System::eSensor sensorType = System::MONOCULAR;
        if (slamType == "mono+imu") {
            sensorType = System::IMU_MONOCULAR;
        }
        
        // SLAM 시스템 초기화 - Localization 모드로 시작
        slamSystem = make_shared<System>(vocabPath, settingsPath, sensorType, true);
        slamSystem->ActivateLocalizationMode(); // 로컬라이제이션 모드 활성화
    }
    
    void updateActivity() {
        lastActivity = chrono::system_clock::now();
    }
    
    bool isExpired(int timeoutMinutes = 30) {
        auto now = chrono::system_clock::now();
        auto diff = chrono::duration_cast<chrono::minutes>(now - lastActivity).count();
        return diff > timeoutMinutes;
    }
    
    void saveTrajectory(bool isFinal = false) {
        if (!slamSystem) return;
        
        string prefix = isFinal ? "/final" : "/temp_" + to_string(frameCounter);
        string trajFilename = trajectoryBasePath + prefix;
        
        slamSystem->SaveTrajectoryEuRoC(trajFilename);
        slamSystem->SaveKeyFrameTrajectoryEuRoC(trajFilename);
        
        if (isFinal) {
            cout << "최종 궤적 저장: " << trajFilename << endl;
        }
    }
};

// 전역 변수
mutex sessionsMutex;
map<string, shared_ptr<UserSession>> userSessions;
bool server_running = true;
string default_vocab_path = "../ORB_SLAM3/Vocabulary/ORBvoc.txt";
string default_settings_path = "../config/EuRoC.yaml";

// 유틸리티 함수
string generate_session_id() {
    static random_device rd;
    static mt19937 gen(rd());
    static uniform_int_distribution<> dis(0, 15);
    static const char* hex = "0123456789abcdef";
    
    string uuid;
    for (int i = 0; i < 32; ++i) {
        uuid += hex[dis(gen)];
        if (i == 7 || i == 11 || i == 15 || i == 19) {
            uuid += '-';
        }
    }
    return uuid;
}

// 캘리브레이션 파라미터를 YAML 파일로 저장하는 함수
bool saveCalibrationToYaml(const json& calib_params, const string& filename) {
    // EuRoC.yaml 파일을 기본 템플릿으로 사용
    fs::copy_file(default_settings_path, filename, fs::copy_options::overwrite_existing);
    
    // OpenCV FileStorage로 파일 열기
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        cerr << "Failed to open file: " << filename << endl;
        return false;
    }
    
    // 캘리브레이션 파라미터 업데이트
    if (calib_params.contains("fx")) fs << "Camera1.fx" << calib_params["fx"].get<float>();
    if (calib_params.contains("fy")) fs << "Camera1.fy" << calib_params["fy"].get<float>();
    if (calib_params.contains("cx")) fs << "Camera1.cx" << calib_params["cx"].get<float>();
    if (calib_params.contains("cy")) fs << "Camera1.cy" << calib_params["cy"].get<float>();
    if (calib_params.contains("k1")) fs << "Camera1.k1" << calib_params["k1"].get<float>();
    if (calib_params.contains("k2")) fs << "Camera1.k2" << calib_params["k2"].get<float>();
    if (calib_params.contains("p1")) fs << "Camera1.p1" << calib_params["p1"].get<float>();
    if (calib_params.contains("p2")) fs << "Camera1.p2" << calib_params["p2"].get<float>();
    
    if (calib_params.contains("width")) fs << "Camera.width" << calib_params["width"].get<int>();
    if (calib_params.contains("height")) fs << "Camera.height" << calib_params["height"].get<int>();
    
    fs.release();
    return true;
}

// 주기적으로 만료된 세션 정리
void cleanup_expired_sessions() {
    while (server_running) {
        this_thread::sleep_for(chrono::minutes(5));
        
        vector<string> expiredSessions;
        {
            lock_guard<mutex> lock(sessionsMutex);
            for (auto& pair : userSessions) {
                if (pair.second->isExpired()) {
                    expiredSessions.push_back(pair.first);
                }
            }
            
            for (const auto& id : expiredSessions) {
                cout << "만료된 세션 제거: " << id << endl;
                userSessions[id]->saveTrajectory(true);
                userSessions.erase(id);
            }
        }
    }
}

// Ping 핸들러
void ping_handler(const shared_ptr<Session>& session) {
    const auto request = session->get_request();
    string response = "pong";
    
    session->close(OK, response, {
        {"Content-Length", to_string(response.length())},
        {"Content-Type", "text/plain"}
    });
}

// 캘리브레이션 핸들러
void calibration_handler(const shared_ptr<Session>& session) {
    const auto request = session->get_request();
    
    // 요청 본문 길이 가져오기
    size_t content_length = stoul(request->get_header("Content-Length", "0"));
    
    // 요청 본문 가져오기
    session->fetch(content_length, [](const shared_ptr<Session> session, const Bytes& body) {
        string json_str(body.begin(), body.end());
        
        try {
            // JSON 파싱
            json request_data = json::parse(json_str);
            
            // 필수 매개변수 확인
            if (!request_data.contains("ship_id") || 
                !request_data.contains("username") || 
                !request_data.contains("slam_type") ||
                !request_data.contains("calibration_parameters")) {
                
                string error_msg = "Missing required parameters";
                session->close(BAD_REQUEST, error_msg, {
                    {"Content-Length", to_string(error_msg.length())},
                    {"Content-Type", "text/plain"}
                });
                return;
            }
            
            // JSON에서 필요한 데이터 추출
            string ship_id = request_data["ship_id"];
            string username = request_data["username"];
            string slam_type = request_data["slam_type"];
            auto calib_params = request_data["calibration_parameters"];
            
            // SLAM 타입 확인
            if (slam_type != "mono" && slam_type != "mono+imu") {
                string error_msg = "Unsupported SLAM type. Use 'mono' or 'mono+imu'";
                session->close(BAD_REQUEST, error_msg, {
                    {"Content-Length", to_string(error_msg.length())},
                    {"Content-Type", "text/plain"}
                });
                return;
            }
            
            // 세션 ID 생성 또는 기존 세션 사용
            string session_id;
            if (request_data.contains("session_id") && !request_data["session_id"].empty()) {
                session_id = request_data["session_id"];
            } else {
                session_id = generate_session_id();
            }
            
            // 설정 파일 업데이트
            string settings_path = "config/" + ship_id + "_" + session_id + ".yaml";
            
            // 캘리브레이션 파라미터로 YAML 파일 생성
            if (!saveCalibrationToYaml(calib_params, settings_path)) {
                string error_msg = "Failed to save calibration parameters";
                session->close(INTERNAL_SERVER_ERROR, error_msg, {
                    {"Content-Length", to_string(error_msg.length())},
                    {"Content-Type", "text/plain"}
                });
                return;
            }
            
            // 기존 세션이 있으면 종료하고 새로 생성
            {
                lock_guard<mutex> lock(sessionsMutex);
                if (userSessions.find(session_id) != userSessions.end()) {
                    userSessions[session_id]->saveTrajectory(true);
                    userSessions.erase(session_id);
                }
                
                // 새 세션 생성
                userSessions[session_id] = make_shared<UserSession>(
                    session_id, ship_id, username, slam_type, 
                    default_vocab_path, settings_path
                );
            }
            
            // 응답 생성
            json response_data;
            response_data["status"] = "success";
            response_data["message"] = "Calibration complete and SLAM system initialized in localization mode";
            response_data["session_id"] = session_id;
            response_data["ship_id"] = ship_id;
            
            string response = response_data.dump();
            
            session->close(OK, response, {
                {"Content-Length", to_string(response.length())},
                {"Content-Type", "application/json"}
            });
            
        } catch (const exception& e) {
            string error_msg = string("Error processing request: ") + e.what();
            session->close(INTERNAL_SERVER_ERROR, error_msg, {
                {"Content-Length", to_string(error_msg.length())},
                {"Content-Type", "text/plain"}
            });
        }
    });
}

// 트래킹 핸들러
void tracking_handler(const shared_ptr<Session>& session) {
    const auto request = session->get_request();
    
    // 세션 ID 확인
    string session_id = request->get_path_parameter("session_id", "");
    if (session_id.empty()) {
        session_id = request->get_query_parameter("session_id", "");
    }
    
    if (session_id.empty()) {
        string error_msg = "Session ID is required";
        session->close(BAD_REQUEST, error_msg, {
            {"Content-Length", to_string(error_msg.length())},
            {"Content-Type", "text/plain"}
        });
        return;
    }
    
    // 세션 확인
    shared_ptr<UserSession> userSession;
    {
        lock_guard<mutex> lock(sessionsMutex);
        if (userSessions.find(session_id) == userSessions.end()) {
            string error_msg = "Invalid or expired session ID";
            session->close(BAD_REQUEST, error_msg, {
                {"Content-Length", to_string(error_msg.length())},
                {"Content-Type", "text/plain"}
            });
            return;
        }
        userSession = userSessions[session_id];
        userSession->updateActivity();
    }
    
    // 요청 본문 길이 가져오기
    size_t content_length = stoul(request->get_header("Content-Length", "0"));
    
    // 본문 가져오기
    session->fetch(content_length, [userSession](const shared_ptr<Session> session, const Bytes& body) {
        try {
            // OpenCV로 이미지 디코딩
            vector<uchar> img_data(body.begin(), body.end());
            cv::Mat img = cv::imdecode(img_data, cv::IMREAD_COLOR);
            
            if(img.empty()) {
                string error_msg = "Invalid image data";
                session->close(BAD_REQUEST, error_msg, {
                    {"Content-Length", to_string(error_msg.length())},
                    {"Content-Type", "text/plain"}
                });
                return;
            }
            
            // 타임스탬프 생성 (현재 시간)
            double timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            
            // IMU 데이터 
            vector<IMU::Point> imu_measurements;
            
            // IMU 데이터 처리 (IMU 모드이고 IMU 데이터가 있는 경우)
            if (userSession->slamType == "mono+imu") {
                // IMU 데이터가 요청 헤더에 있는지 확인
                string imu_header = session->get_request()->get_header("X-IMU-Data", "");
                
                if (!imu_header.empty()) {
                    try {
                        // 헤더에서 IMU 데이터 파싱
                        json imu_data = json::parse(imu_header);
                        
                        // IMU 데이터를 IMU::Point로 변환
                        if (imu_data.is_array()) {
                            for (const auto& imu_point : imu_data) {
                                if (imu_point.contains("timestamp") && 
                                    imu_point.contains("gyro") && 
                                    imu_point.contains("acc")) {
                                    
                                    double t = imu_point["timestamp"];
                                    vector<float> gyro = imu_point["gyro"];
                                    vector<float> acc = imu_point["acc"];
                                    
                                    if (gyro.size() == 3 && acc.size() == 3) {
                                        IMU::Point point(
                                            acc[0], acc[1], acc[2],
                                            gyro[0], gyro[1], gyro[2],
                                            t
                                        );
                                        imu_measurements.push_back(point);
                                    }
                                }
                            }
                        }
                    } catch (...) {
                        // IMU 데이터 파싱 실패 시 빈 IMU로 진행
                        imu_measurements.clear();
                    }
                }
            }
            
            // 파일명 (이미지 저장용)
            string filename = userSession->trajectoryBasePath + "/frame_" + to_string(timestamp) + ".jpg";
            
            // ORB-SLAM3 트래킹 실행
            Sophus::SE3f pose;
            if (userSession->slamType == "mono") {
                pose = userSession->slamSystem->TrackMonocular(img, timestamp, vector<IMU::Point>(), filename);
            } else if (userSession->slamType == "mono+imu") {
                pose = userSession->slamSystem->TrackMonocular(img, timestamp, imu_measurements, filename);
            }
            
            // 프레임 카운터 증가 및 주기적 궤적 저장
            userSession->frameCounter++;
            if (userSession->frameCounter % 100 == 0) {
                userSession->saveTrajectory();
            }
            
            // SE3f 포즈를 변환 및 회전으로 변환
            Eigen::Vector3f translation = pose.translation();
            Eigen::Matrix3f rotation_matrix = pose.rotationMatrix();
            Eigen::Quaternionf quaternion(rotation_matrix);
            
            // 결과를 JSON으로 반환
            json response_data;
            response_data["status"] = "success";
            response_data["session_id"] = userSession->sessionId;
            response_data["frame_count"] = userSession->frameCounter;
            
            // 변환(translation) 정보
            response_data["translation"] = {
                {"x", translation.x()},
                {"y", translation.y()},
                {"z", translation.z()}
            };
            
            // 회전(quaternion) 정보
            response_data["quaternion"] = {
                {"w", quaternion.w()},
                {"x", quaternion.x()},
                {"y", quaternion.y()},
                {"z", quaternion.z()}
            };
            
            // 트래킹 상태 추가
            response_data["tracking_state"] = userSession->slamSystem->GetTrackingState();
            
            string response = response_data.dump();
            
            session->close(OK, response, {
                {"Content-Length", to_string(response.length())},
                {"Content-Type", "application/json"}
            });
            
        } catch (const exception& e) {
            string error_msg = string("Error processing image: ") + e.what();
            session->close(INTERNAL_SERVER_ERROR, error_msg, {
                {"Content-Length", to_string(error_msg.length())},
                {"Content-Type", "text/plain"}
            });
        }
    });
}

// 세션 종료 핸들러
void session_close_handler(const shared_ptr<Session>& session) {
    const auto request = session->get_request();
    
    // 세션 ID 확인
    string session_id = request->get_path_parameter("session_id", "");
    if (session_id.empty()) {
        session_id = request->get_query_parameter("session_id", "");
    }
    
    if (session_id.empty()) {
        string error_msg = "Session ID is required";
        session->close(BAD_REQUEST, error_msg, {
            {"Content-Length", to_string(error_msg.length())},
            {"Content-Type", "text/plain"}
        });
        return;
    }
    
    // 세션 종료
    bool found = false;
    {
        lock_guard<mutex> lock(sessionsMutex);
        if (userSessions.find(session_id) != userSessions.end()) {
            userSessions[session_id]->saveTrajectory(true);
            userSessions.erase(session_id);
            found = true;
        }
    }
    
    if (found) {
        string response = "Session closed successfully";
        session->close(OK, response, {
            {"Content-Length", to_string(response.length())},
            {"Content-Type", "text/plain"}
        });
    } else {
        string error_msg = "Session not found";
        session->close(NOT_FOUND, error_msg, {
            {"Content-Length", to_string(error_msg.length())},
            {"Content-Type", "text/plain"}
        });
    }
}

// 서버 종료 핸들러
void shutdown_handler(const shared_ptr<Session>& session) {
    const auto request = session->get_request();
    
    // 모든 세션 저장 및 종료
    {
        lock_guard<mutex> lock(sessionsMutex);
        for (auto& pair : userSessions) {
            pair.second->saveTrajectory(true);
        }
        userSessions.clear();
    }
    
    // 응답 생성
    string response = "Server shutting down...";
    
    session->close(OK, response, {
        {"Content-Length", to_string(response.length())},
        {"Content-Type", "text/plain"}
    });
    
    // 다른 스레드에서 서버 종료 실행
    server_running = false;
    thread([&]() {
        this_thread::sleep_for(chrono::seconds(1)); // 응답을 보낼 시간 확보
        std::raise(SIGINT); // 서버 종료 시그널 보내기
    }).detach();
}

// 세션 목록 조회 핸들러
void sessions_list_handler(const shared_ptr<Session>& session) {
    const auto request = session->get_request();
    
    json response_data;
    response_data["status"] = "success";
    json sessions_array = json::array();
    
    {
        lock_guard<mutex> lock(sessionsMutex);
        for (const auto& pair : userSessions) {
            auto& userSession = pair.second;
            
            json session_data;
            session_data["session_id"] = userSession->sessionId;
            session_data["ship_id"] = userSession->shipId;
            session_data["username"] = userSession->username;
            session_data["slam_type"] = userSession->slamType;
            session_data["frame_count"] = userSession->frameCounter;
            
            auto time_point = userSession->lastActivity;
            auto seconds = chrono::time_point_cast<chrono::seconds>(time_point);
            session_data["last_activity"] = seconds.time_since_epoch().count();
            
            sessions_array.push_back(session_data);
        }
    }
    
    response_data["sessions"] = sessions_array;
    response_data["count"] = sessions_array.size();
    
    string response = response_data.dump();
    
    session->close(OK, response, {
        {"Content-Length", to_string(response.length())},
        {"Content-Type", "application/json"}
    });
}

// 메인 함수
int main(int argc, char** argv) {
    cout << "Initializing Multi-User SLAM Localization Server..." << endl;
    
    // 명령행 인자로 경로 변경 가능
    if (argc > 1) default_vocab_path = argv[1];
    if (argc > 2) default_settings_path = argv[2];
    
    cout << "Using vocabulary: " << default_vocab_path << endl;
    cout << "Using default settings: " << default_settings_path << endl;
    
    // 필요한 디렉토리 생성
    fs::create_directories("trajectories");
    fs::create_directories("config");
    
    // 세션 정리 스레드 시작
    thread cleanup_thread(cleanup_expired_sessions);
    cleanup_thread.detach();
    
    try {
        // Restbed 서비스 설정
        auto ping_resource = make_shared<Resource>();
        ping_resource->set_path("/ping");
        ping_resource->set_method_handler("GET", ping_handler);
        
        auto calib_resource = make_shared<Resource>();
        calib_resource->set_path("/calibration");
        calib_resource->set_method_handler("POST", calibration_handler);
        
        auto tracking_resource = make_shared<Resource>();
        tracking_resource->set_path("/tracking");
        tracking_resource->set_method_handler("POST", tracking_handler);
        
        auto session_close_resource = make_shared<Resource>();
        session_close_resource->set_path("/session/close");
        session_close_resource->set_method_handler("POST", session_close_handler);
        
        auto sessions_list_resource = make_shared<Resource>();
        sessions_list_resource->set_path("/sessions");
        sessions_list_resource->set_method_handler("GET", sessions_list_handler);
        
        auto shutdown_resource = make_shared<Resource>();
        shutdown_resource->set_path("/shutdown");
        shutdown_resource->set_method_handler("POST", shutdown_handler);
        
        // restbed의 설정
        auto server_settings = make_shared<restbed::Settings>();
        server_settings->set_port(8080);
        server_settings->set_default_header("Connection", "close");
        server_settings->set_worker_limit(8); // 워커 스레드 수 증가
        
        Service service;
        service.publish(ping_resource);
        service.publish(calib_resource);
        service.publish(tracking_resource);
        service.publish(session_close_resource);
        service.publish(sessions_list_resource);
        service.publish(shutdown_resource);
        
        cout << "Multi-User SLAM Localization Server running on port 8080" << endl;
        cout << "Endpoints:" << endl;
        cout << " - GET  /ping                   : Server health check" << endl;
        cout << " - POST /calibration            : Initialize SLAM system with calibration" << endl;
        cout << " - POST /tracking?session_id=ID : Process image and return camera pose" << endl;
        cout << " - POST /session/close?session_id=ID : Close a specific session" << endl;
        cout << " - GET  /sessions               : List all active sessions" << endl;
        cout << " - POST /shutdown               : Gracefully shutdown the server" << endl;
        
        // 서버 시작
        service.start(server_settings);
        
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return EXIT_FAILURE;
    }
    
    // 모든 세션 저장 및 종료
    {
        lock_guard<mutex> lock(sessionsMutex);
        for (auto& pair : userSessions) {
            pair.second->saveTrajectory(true);
        }
        userSessions.clear();
    }
    
    cout << "SLAM Server terminated." << endl;
    return EXIT_SUCCESS;
}