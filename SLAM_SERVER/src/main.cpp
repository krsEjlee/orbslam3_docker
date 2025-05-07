//src/main.cpp
#include "slam_server.hpp"
#include <iostream>
#include <csignal> // Changed from <signal.h> to <csignal>

// Global pointer for signal handling
SlamServer* g_server = nullptr;

// Signal handler
void signalHandler(int signal) {
    if (g_server) {
        std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
        g_server->stop();
    }
    exit(signal);
}

// Print usage
void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <port> <vocabulary_file> <settings_file>" << std::endl;
    std::cout << "Example: " << programName << " 8080 ../ORB_SLAM3/Vocabulary/ORBvoc.txt ../ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml" << std::endl;
}

int main(int argc, char** argv) {
    // Check arguments
    if (argc != 4) {
        printUsage(argv[0]);
        return 1;
    }
    
    // Parse arguments
    int port = std::stoi(argv[1]);
    std::string vocabPath = argv[2];
    std::string settingsPath = argv[3];
    
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create server
        SlamServer server(port, vocabPath, settingsPath);
        g_server = &server;
        
        // Start server
        server.start();
        
        // Server runs in its own thread, so we just wait
        while(true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}