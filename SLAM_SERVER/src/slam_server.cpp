// src/slam_server.cpp

#include "slam_server.hpp"
// Include for C++11 directory creation
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>

// Constructor
SlamServer::SlamServer(int port, const std::string& vocabPath, const std::string& settingsPath)
    : port_(port), 
      vocabPath_(vocabPath), 
      settingsPath_(settingsPath), 
      isRunning_(false),
      lastTimestamp_(0.0),
      frameCount_(0),
      status_(NOT_INITIALIZED) {
    
    // Create a new service - use direct new instead of make_unique
    service_ = std::unique_ptr<restbed::Service>(new restbed::Service());
}

// Destructor
SlamServer::~SlamServer() {
    stop();
}

// Initialize the SLAM system
bool SlamServer::initializeSLAM() {
    try {
        // Create ORB-SLAM3 system - use direct new instead of make_unique
        slam_ = std::unique_ptr<ORB_SLAM3::System>(new ORB_SLAM3::System(
            vocabPath_,                      // Vocabulary file
            settingsPath_,                   // Settings file
            ORB_SLAM3::System::IMU_MONOCULAR, // Sensor type (Monocular + IMU)
            true                             // Enable visualization
        ));
        
        std::cout << "ORB-SLAM3 system initialized successfully" << std::endl;
        status_ = INITIALIZING;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing ORB-SLAM3: " << e.what() << std::endl;
        return false;
    }
}


// Start the server
void SlamServer::start() {
    if (isRunning_) return;
    
    // Initialize the SLAM system
    if (!initializeSLAM()) {
        std::cerr << "Failed to initialize SLAM system" << std::endl;
        return;
    }
    
    // Create API endpoints
    
    // 1. Image endpoint for processing frames
    auto imageResource = std::make_shared<restbed::Resource>();
    imageResource->set_path("/image");
    imageResource->set_method_handler("POST", [this](const std::shared_ptr<restbed::Session>& session) {
        this->handleImage(session);
    });
    
    // 2. Position endpoint for initializing SLAM with known position
    auto positionResource = std::make_shared<restbed::Resource>();
    positionResource->set_path("/position");
    positionResource->set_method_handler("POST", [this](const std::shared_ptr<restbed::Session>& session) {
        this->handlePosition(session);
    });
    
    // 3. Get pose endpoint
    auto poseResource = std::make_shared<restbed::Resource>();
    poseResource->set_path("/pose");
    poseResource->set_method_handler("GET", [this](const std::shared_ptr<restbed::Session>& session) {
        this->handleGetPose(session);
    });
    
    // 4. Status endpoint
    auto statusResource = std::make_shared<restbed::Resource>();
    statusResource->set_path("/status");
    statusResource->set_method_handler("GET", [this](const std::shared_ptr<restbed::Session>& session) {
        this->handleStatus(session);
    });
    
    // 5. Shutdown endpoint
    auto shutdownResource = std::make_shared<restbed::Resource>();
    shutdownResource->set_path("/shutdown");
    shutdownResource->set_method_handler("POST", [this](const std::shared_ptr<restbed::Session>& session) {
        this->handleShutdown(session);
    });
    
    // Add resources to service
    service_->publish(imageResource);
    service_->publish(positionResource);
    service_->publish(poseResource);
    service_->publish(statusResource);
    service_->publish(shutdownResource);
    
    // Configure service settings
    auto settings = std::make_shared<restbed::Settings>();
    settings->set_port(port_);
    settings->set_default_header("Connection", "close");
    settings->set_worker_limit(4);
    
    std::cout << "Starting SLAM Server on port " << port_ << std::endl;
    std::cout << "Endpoints available:" << std::endl;
    std::cout << " - POST /image        : Process a new camera frame" << std::endl;
    std::cout << " - POST /position     : Initialize with known position" << std::endl;
    std::cout << " - GET  /pose         : Get current camera pose" << std::endl;
    std::cout << " - GET  /status       : Get SLAM system status" << std::endl;
    std::cout << " - POST /shutdown     : Shutdown the server" << std::endl;
    
    // Start the service
    isRunning_ = true;
    service_->start(settings);
}

// Stop the server
void SlamServer::stop() {
    if (!isRunning_) return;
    
    std::cout << "Stopping SLAM Server..." << std::endl;
    
    // Stop the service
    service_->stop();
    
    // Save trajectory and shutdown SLAM
    if (slam_) {
        saveTrajectory();
        slam_->Shutdown();
        slam_.reset();
    }
    
    isRunning_ = false;
    std::cout << "SLAM Server stopped" << std::endl;
}

// Handle image processing
void SlamServer::handleImage(const std::shared_ptr<restbed::Session>& session) {
    const auto request = session->get_request();
    
    // Get content length
    size_t contentLength = std::stoul(request->get_header("Content-Length", "0"));
    
    // Check if we have content
    if (contentLength == 0) {
        std::string errorMsg = "No image data provided";
        session->close(restbed::BAD_REQUEST, errorMsg, {
            {"Content-Length", std::to_string(errorMsg.length())},
            {"Content-Type", "text/plain"}
        });
        return;
    }
    
    // Fetch request body
    session->fetch(contentLength, [this](const std::shared_ptr<restbed::Session> session, const restbed::Bytes& body) {
        const auto request = session->get_request();
        
        try {
            // Convert body to OpenCV image
            std::vector<uchar> imageData(body.begin(), body.end());
            cv::Mat image = cv::imdecode(imageData, cv::IMREAD_COLOR);
            
            if (image.empty()) {
                std::string errorMsg = "Invalid image data";
                session->close(restbed::BAD_REQUEST, errorMsg, {
                    {"Content-Length", std::to_string(errorMsg.length())},
                    {"Content-Type", "text/plain"}
                });
                return;
            }
            
            // Get timestamp from header or use current time
            double timestamp = std::stod(request->get_header("X-Timestamp", "0.0"));
            if (timestamp == 0.0) {
                // Use current time
                timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            }
            
            // Parse IMU data if available
            std::vector<ORB_SLAM3::IMU::Point> imuData;
            std::string imuHeader = request->get_header("X-IMU-Data", "");
            if (!imuHeader.empty()) {
                imuData = parseImuData(imuHeader);
            }
            
            // Process the frame
            // 경고 수정: pose 변수가 설정되었지만 사용되지 않음
            processCameraFrame(image, timestamp, imuData);
            
            // Prepare response
            json response = getCurrentPose();
            
            // Return the response
            std::string responseBody = response.dump();
            session->close(restbed::OK, responseBody, {
                {"Content-Length", std::to_string(responseBody.length())},
                {"Content-Type", "application/json"}
            });
        } catch (const std::exception& e) {
            std::string errorMsg = std::string("Error processing image: ") + e.what();
            session->close(restbed::INTERNAL_SERVER_ERROR, errorMsg, {
                {"Content-Length", std::to_string(errorMsg.length())},
                {"Content-Type", "text/plain"}
            });
        }
    });
}

// Handle position initialization
void SlamServer::handlePosition(const std::shared_ptr<restbed::Session>& session) {
    const auto request = session->get_request();
    
    // Get content length
    size_t contentLength = std::stoul(request->get_header("Content-Length", "0"));
    
    // Check if we have content
    if (contentLength == 0) {
        std::string errorMsg = "No position data provided";
        session->close(restbed::BAD_REQUEST, errorMsg, {
            {"Content-Length", std::to_string(errorMsg.length())},
            {"Content-Type", "text/plain"}
        });
        return;
    }
    
    // Fetch request body
    session->fetch(contentLength, [this](const std::shared_ptr<restbed::Session> session, const restbed::Bytes& body) {
        try {
            // Parse JSON body
            std::string jsonStr(body.begin(), body.end());
            json data = json::parse(jsonStr);
            
            // Validate required fields
            if (!data.contains("position") || !data.contains("orientation") || !data.contains("timestamp")) {
                std::string errorMsg = "Missing required fields (position, orientation, timestamp)";
                session->close(restbed::BAD_REQUEST, errorMsg, {
                    {"Content-Length", std::to_string(errorMsg.length())},
                    {"Content-Type", "text/plain"}
                });
                return;
            }
            
            // Get timestamp
            double timestamp = data["timestamp"];
            
            // Get position and orientation
            auto position = data["position"];
            auto orientation = data["orientation"];
            
            // Convert to Eigen types
            Eigen::Vector3f translationVector(
                position[0].get<float>(),
                position[1].get<float>(),
                position[2].get<float>()
            );
            
            Eigen::Quaternionf quaternion(
                orientation[3].get<float>(), // w
                orientation[0].get<float>(), // x
                orientation[1].get<float>(), // y
                orientation[2].get<float>()  // z
            );
            
            // Create SE3 pose
            Eigen::Matrix3f rotMatrix = quaternion.toRotationMatrix();
            Sophus::SE3f pose(rotMatrix, translationVector);
            
            // Store the pose (this would normally be used to initialize SLAM)
            {
                std::lock_guard<std::mutex> lock(poseMutex_);
                currentPose_ = pose;
                lastTimestamp_ = timestamp;
                
                // In practice, this information would be passed to ORB-SLAM3
                // for initialization, but that's beyond the scope of this example
                
                // Update status
                if (status_ == INITIALIZING) {
                    status_ = TRACKING_GOOD;
                }
            }
            
            // Respond with success
            json response = {
                {"status", "success"},
                {"message", "Position initialized successfully"}
            };
            
            std::string responseBody = response.dump();
            session->close(restbed::OK, responseBody, {
                {"Content-Length", std::to_string(responseBody.length())},
                {"Content-Type", "application/json"}
            });
            
        } catch (const std::exception& e) {
            std::string errorMsg = std::string("Error processing position data: ") + e.what();
            session->close(restbed::INTERNAL_SERVER_ERROR, errorMsg, {
                {"Content-Length", std::to_string(errorMsg.length())},
                {"Content-Type", "text/plain"}
            });
        }
    });
}

// Handle get pose request
void SlamServer::handleGetPose(const std::shared_ptr<restbed::Session>& session) {
    try {
        // Get current pose as JSON
        json response = getCurrentPose();
        
        // Return the response
        std::string responseBody = response.dump();
        session->close(restbed::OK, responseBody, {
            {"Content-Length", std::to_string(responseBody.length())},
            {"Content-Type", "application/json"}
        });
    } catch (const std::exception& e) {
        std::string errorMsg = std::string("Error getting pose: ") + e.what();
        session->close(restbed::INTERNAL_SERVER_ERROR, errorMsg, {
            {"Content-Length", std::to_string(errorMsg.length())},
            {"Content-Type", "text/plain"}
        });
    }
}

// Handle status request
void SlamServer::handleStatus(const std::shared_ptr<restbed::Session>& session) {
    try {
        // Prepare status response
        json response = {
            {"status", "running"},
            {"frame_count", frameCount_},
            {"slam_status", ""}
        };
        
        // Convert enum to string
        switch (status_) {
            case NOT_INITIALIZED:
                response["slam_status"] = "not_initialized";
                break;
            case INITIALIZING:
                response["slam_status"] = "initializing";
                break;
            case TRACKING_GOOD:
                response["slam_status"] = "tracking_good";
                break;
            case TRACKING_BAD:
                response["slam_status"] = "tracking_bad";
                break;
            case LOST:
                response["slam_status"] = "lost";
                break;
        }
        
        // Return the response
        std::string responseBody = response.dump();
        session->close(restbed::OK, responseBody, {
            {"Content-Length", std::to_string(responseBody.length())},
            {"Content-Type", "application/json"}
        });
    } catch (const std::exception& e) {
        std::string errorMsg = std::string("Error getting status: ") + e.what();
        session->close(restbed::INTERNAL_SERVER_ERROR, errorMsg, {
            {"Content-Length", std::to_string(errorMsg.length())},
            {"Content-Type", "text/plain"}
        });
    }
}

// Handle shutdown request
void SlamServer::handleShutdown(const std::shared_ptr<restbed::Session>& session) {
    // Prepare response
    std::string response = "Shutting down SLAM server...";
    
    // Send response
    session->close(restbed::OK, response, {
        {"Content-Length", std::to_string(response.length())},
        {"Content-Type", "text/plain"}
    });
    
    // Stop the server in a separate thread to allow response to be sent
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        this->stop();
    }).detach();
}

// Process camera frame
Sophus::SE3f SlamServer::processCameraFrame(const cv::Mat& image, double timestamp, 
                                           const std::vector<ORB_SLAM3::IMU::Point>& imuData) {
    // Process frame with ORB-SLAM3
    Sophus::SE3f pose = slam_->TrackMonocular(image, timestamp, imuData);
    
    // Update frame counter
    frameCount_++;
    
    // Save every 100 frames
    if (frameCount_ % 100 == 0) {
        saveTrajectory();
    }
    
    // Update current pose
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        currentPose_ = pose;
        lastTimestamp_ = timestamp;
        
        // Update status based on tracking state
        int trackingState = slam_->GetTrackingState();
        if (trackingState == 2) {
            status_ = TRACKING_GOOD;
        } else if (trackingState == 1) {
            status_ = TRACKING_BAD;
        } else {
            status_ = LOST;
        }
    }
    
    return pose;
}

// Get current pose as JSON
json SlamServer::getCurrentPose() {
    std::lock_guard<std::mutex> lock(poseMutex_);
    
    // Extract translation and rotation
    Eigen::Vector3f translation = currentPose_.translation();
    Eigen::Matrix3f rotMatrix = currentPose_.rotationMatrix();
    Eigen::Quaternionf quaternion(rotMatrix);
    
    // Create JSON response
    json response = {
        {"timestamp", lastTimestamp_},
        {"position", {translation.x(), translation.y(), translation.z()}},
        {"orientation", {quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()}}
    };
    
    return response;
}

// Parse IMU data from a JSON string
std::vector<ORB_SLAM3::IMU::Point> SlamServer::parseImuData(const std::string& imuHeader) {
    std::vector<ORB_SLAM3::IMU::Point> imuData;
    
    try {
        // Parse JSON
        json imuJson = json::parse(imuHeader);
        
        // Process each IMU point
        for (const auto& point : imuJson) {
            if (point.contains("timestamp") && point.contains("gyro") && point.contains("acc")) {
                double t = point["timestamp"];
                auto gyro = point["gyro"];
                auto acc = point["acc"];
                
                if (gyro.size() == 3 && acc.size() == 3) {
                    imuData.emplace_back(
                        acc[0], acc[1], acc[2],
                        gyro[0], gyro[1], gyro[2],
                        t
                    );
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing IMU data: " << e.what() << std::endl;
    }
    
    return imuData;
}

// Create directory helper for C++11
bool SlamServer::createDirectory(const std::string& path) {
    // Create directory using system calls
    #ifdef _WIN32
    // Windows
    int result = _mkdir(path.c_str());
    #else
    // Linux/Unix/Mac
    int result = mkdir(path.c_str(), 0777);
    #endif
    
    // Check result
    if (result == 0) {
        return true;
    }
    
    // If directory already exists, that's fine
    if (errno == EEXIST) {
        return true;
    }
    
    std::cerr << "Error creating directory: " << strerror(errno) << std::endl;
    return false;
}

// Save trajectory to file
void SlamServer::saveTrajectory() {
    if (!slam_) return;
    
    try {
        // Create directory if it doesn't exist using C++11 approach
        createDirectory("trajectories");
        
        // Save trajectory
        slam_->SaveTrajectoryEuRoC("trajectories/CameraTrajectory.txt");
        slam_->SaveKeyFrameTrajectoryEuRoC("trajectories/KeyFrameTrajectory.txt");
        
        std::cout << "Trajectory saved (Frame " << frameCount_ << ")" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error saving trajectory: " << e.what() << std::endl;
    }
}