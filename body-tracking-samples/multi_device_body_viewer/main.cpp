// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Multi-device body tracking viewer for Orbbec Femto Bolt cameras
// with multi-camera skeleton fusion support

#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <nlohmann/json.hpp>

// Winsock for UDP communication
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

using json = nlohmann::json;

// ============================================================================
// Fusion Data Structures
// ============================================================================
struct CameraExtrinsics {
    std::string serialNumber;
    int deviceIndex;
    bool isValid;
    float rotation[3][3];    // 3x3 rotation matrix (row-major)
    float translation[3];    // Translation vector (mm)
};

struct CalibrationData {
    int numDevices;
    std::vector<CameraExtrinsics> cameras;
    bool isLoaded;
};

struct FusedJoint {
    k4a_float3_t position;
    k4a_quaternion_t orientation;
    k4abt_joint_confidence_level_t confidence;
    int sourceDeviceIndex;
};

struct FusedBody {
    uint32_t id;
    FusedJoint joints[K4ABT_JOINT_COUNT];
};

struct BodyMatch {
    std::vector<int> bodyIndicesPerCamera;  // -1 if not visible in that camera
};

enum class FusionMode {
    WINNER_TAKES_ALL,
    WEIGHTED_AVERAGE
};

// ============================================================================
// Global State
// ============================================================================
std::atomic<bool> s_isRunning{true};
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;

// Camera view mode: -1 = all cameras, 0+ = specific camera index
int g_cameraViewMode = -1;
int g_numDevices = 0;
std::vector<std::string> g_deviceSerialNumbers;

// CSV Recording State
std::atomic<bool> g_isRecording{false};
std::ofstream g_csvFile;
std::mutex g_csvMutex;
std::string g_outputPath = "";
std::chrono::steady_clock::time_point g_recordingStartTime;

// Fusion state
CalibrationData g_calibration = {0, {}, false};
std::vector<FusedBody> g_fusedBodies;
std::mutex g_fusedBodyMutex;
bool g_fusionEnabled = false;
FusionMode g_fusionMode = FusionMode::WEIGHTED_AVERAGE;
std::string g_calibrationPath = "";
std::string g_primarySerial = "";  // Serial number of PRIMARY camera (sync hub master port)
const float BODY_MATCH_THRESHOLD_MM = 500.0f;

// Thread-safe storage for body tracking results
std::mutex g_bodyDataMutex;
struct DeviceBodyData {
    std::vector<k4abt_body_t> bodies;
    int depthWidth = 0;
    int depthHeight = 0;
    k4a_image_t depthImage = nullptr;
    k4a_image_t bodyIndexMap = nullptr;
    bool hasNewData = false;
    int deviceIndex = 0;
};
std::vector<DeviceBodyData> g_deviceBodyData;

// Forward declarations for CSV recording functions (used by UDP listener)
void StartRecording(const std::string& outputPath);
void StopRecording();

// ============================================================================
// UDP Command Listener (for Unity sync)
// ============================================================================
#ifdef _WIN32
int g_udpListenPort = 9000;
int g_udpSendPort = 9001;
std::string g_udpTargetIP = "127.0.0.1";
std::atomic<bool> g_udpRunning{false};
SOCKET g_udpSocket = INVALID_SOCKET;
std::thread g_udpThread;

// Command queue
std::mutex g_commandMutex;
std::vector<std::string> g_pendingCommands;

void UdpListenerThread()
{
    char buffer[256];
    sockaddr_in senderAddr;
    int senderAddrSize = sizeof(senderAddr);

    std::cout << "[UDP] Listener started on port " << g_udpListenPort << std::endl;

    while (g_udpRunning)
    {
        int recvLen = recvfrom(g_udpSocket, buffer, sizeof(buffer) - 1, 0,
                               (sockaddr*)&senderAddr, &senderAddrSize);

        if (recvLen > 0)
        {
            buffer[recvLen] = '\0';
            std::string command(buffer);

            // Trim whitespace
            command.erase(0, command.find_first_not_of(" \t\n\r"));
            command.erase(command.find_last_not_of(" \t\n\r") + 1);

            if (!command.empty())
            {
                std::lock_guard<std::mutex> lock(g_commandMutex);
                g_pendingCommands.push_back(command);
                std::cout << "[UDP] Received command: " << command << std::endl;
            }
        }
        else if (recvLen == SOCKET_ERROR)
        {
            int err = WSAGetLastError();
            if (err != WSAEWOULDBLOCK && err != WSAETIMEDOUT && g_udpRunning)
            {
                std::cerr << "[UDP] Receive error: " << err << std::endl;
            }
        }
    }

    std::cout << "[UDP] Listener stopped" << std::endl;
}

bool InitUdpListener(int port)
{
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "[UDP] WSAStartup failed" << std::endl;
        return false;
    }

    g_udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_udpSocket == INVALID_SOCKET)
    {
        std::cerr << "[UDP] Socket creation failed" << std::endl;
        WSACleanup();
        return false;
    }

    // Set socket timeout (100ms)
    DWORD timeout = 100;
    setsockopt(g_udpSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

    // Bind to port
    sockaddr_in localAddr;
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(port);
    localAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(g_udpSocket, (sockaddr*)&localAddr, sizeof(localAddr)) == SOCKET_ERROR)
    {
        std::cerr << "[UDP] Bind failed on port " << port << std::endl;
        closesocket(g_udpSocket);
        WSACleanup();
        return false;
    }

    g_udpListenPort = port;
    g_udpRunning = true;
    g_udpThread = std::thread(UdpListenerThread);

    return true;
}

void SendUdpCommand(const std::string& command)
{
    if (g_udpSocket == INVALID_SOCKET) return;

    sockaddr_in targetAddr;
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_port = htons(g_udpSendPort);
    inet_pton(AF_INET, g_udpTargetIP.c_str(), &targetAddr.sin_addr);

    sendto(g_udpSocket, command.c_str(), (int)command.length(), 0,
           (sockaddr*)&targetAddr, sizeof(targetAddr));

    std::cout << "[UDP] Sent: " << command << " -> " << g_udpTargetIP << ":" << g_udpSendPort << std::endl;
}

void ShutdownUdpListener()
{
    g_udpRunning = false;

    if (g_udpSocket != INVALID_SOCKET)
    {
        closesocket(g_udpSocket);
        g_udpSocket = INVALID_SOCKET;
    }

    if (g_udpThread.joinable())
    {
        g_udpThread.join();
    }

    WSACleanup();
    std::cout << "[UDP] Shutdown complete" << std::endl;
}

std::vector<std::string> GetPendingCommands()
{
    std::lock_guard<std::mutex> lock(g_commandMutex);
    std::vector<std::string> commands = std::move(g_pendingCommands);
    g_pendingCommands.clear();
    return commands;
}

void ProcessUdpCommands()
{
    auto commands = GetPendingCommands();
    for (const auto& cmd : commands)
    {
        if (cmd == "TOGGLE_RECORD")
        {
            if (g_isRecording) {
                StopRecording();
                SendUdpCommand("STOP_RECORD");
            } else {
                StartRecording(g_outputPath);
                SendUdpCommand("START_RECORD");
            }
        }
        else if (cmd == "START_RECORD")
        {
            if (!g_isRecording) {
                StartRecording(g_outputPath);
            }
        }
        else if (cmd == "STOP_RECORD")
        {
            if (g_isRecording) {
                StopRecording();
            }
        }
        else if (cmd == "CYCLE_CAMERA")
        {
            g_cameraViewMode++;
            if (g_cameraViewMode >= g_numDevices) {
                g_cameraViewMode = -1;
            }
            std::cout << "Camera view: " << (g_cameraViewMode == -1 ? "ALL" : std::to_string(g_cameraViewMode)) << std::endl;
        }
    }
}
#endif // _WIN32

// ============================================================================
// CSV Recording Functions
// ============================================================================
int64_t GetSystemTimestampMs()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

std::string GenerateTimestampFilename(const std::string& prefix)
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_s(&tm_now, &time_t_now);

    std::ostringstream oss;
    oss << prefix << "_"
        << std::put_time(&tm_now, "%Y%m%d_%H%M%S")
        << ".csv";
    return oss.str();
}

void StartRecording(const std::string& outputPath)
{
    std::lock_guard<std::mutex> lock(g_csvMutex);

    std::string filename = outputPath.empty()
        ? GenerateTimestampFilename("skeleton_data")
        : outputPath;

    g_csvFile.open(filename);
    if (!g_csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << filename << std::endl;
        return;
    }

    // Write CSV header (wide format: one row per frame)
    g_csvFile << "timestamp_ms,device_index,body_id";

    // Add columns for each joint: J{id}_x, J{id}_y, J{id}_z, J{id}_conf
    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        g_csvFile << ",J" << j << "_x,J" << j << "_y,J" << j << "_z,J" << j << "_conf";
    }
    g_csvFile << "\n";

    g_recordingStartTime = std::chrono::steady_clock::now();
    g_isRecording = true;

    std::cout << "Recording started: " << filename << std::endl;
}

void StopRecording()
{
    std::lock_guard<std::mutex> lock(g_csvMutex);

    if (g_csvFile.is_open()) {
        g_csvFile.close();
        std::cout << "Recording stopped." << std::endl;
    }
    g_isRecording = false;
}

void RecordSkeletonFrame(int deviceIndex, const std::vector<k4abt_body_t>& bodies)
{
    if (!g_isRecording) return;

    std::lock_guard<std::mutex> lock(g_csvMutex);
    if (!g_csvFile.is_open()) return;

    int64_t timestamp = GetSystemTimestampMs();

    // Wide format: one row per body, all joints in columns
    for (const auto& body : bodies) {
        g_csvFile << timestamp << ","
                  << deviceIndex << ","
                  << body.id;

        // Write all joints in sequence
        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
            const auto& joint = body.skeleton.joints[j];
            g_csvFile << std::fixed << std::setprecision(3)
                      << "," << joint.position.xyz.x
                      << "," << joint.position.xyz.y
                      << "," << joint.position.xyz.z
                      << "," << static_cast<int>(joint.confidence_level);
        }
        g_csvFile << "\n";
    }

    g_csvFile.flush();  // Ensure data is written immediately
}

// ============================================================================
// Input Handling
// ============================================================================
int64_t ProcessKey(void* /*context*/, int key)
{
    switch (key)
    {
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        // Cycle through camera views: All(-1) -> Cam0 -> Cam1 -> ... -> All(-1)
        g_cameraViewMode++;
        if (g_cameraViewMode >= g_numDevices) {
            g_cameraViewMode = -1;
        }
        if (g_cameraViewMode == -1) {
            std::cout << "Camera view: ALL CAMERAS" << std::endl;
        } else {
            std::cout << "Camera view: Device " << g_cameraViewMode
                      << " (SN: " << g_deviceSerialNumbers[g_cameraViewMode] << ")" << std::endl;
        }
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_F:
        if (g_calibration.isLoaded) {
            g_fusionEnabled = !g_fusionEnabled;
            std::cout << "Skeleton fusion: " << (g_fusionEnabled ? "ON" : "OFF") << std::endl;
        } else {
            std::cout << "Fusion not available (no calibration loaded)" << std::endl;
        }
        break;
    case GLFW_KEY_M:
        if (g_fusionMode == FusionMode::WINNER_TAKES_ALL) {
            g_fusionMode = FusionMode::WEIGHTED_AVERAGE;
            std::cout << "Fusion mode: WEIGHTED_AVERAGE" << std::endl;
        } else {
            g_fusionMode = FusionMode::WINNER_TAKES_ALL;
            std::cout << "Fusion mode: WINNER_TAKES_ALL" << std::endl;
        }
        break;
    case GLFW_KEY_R:
        if (g_isRecording) {
            StopRecording();
#ifdef _WIN32
            SendUdpCommand("STOP_RECORD");
#endif
        } else {
            StartRecording(g_outputPath);
#ifdef _WIN32
            SendUdpCommand("START_RECORD");
#endif
        }
        break;
    case GLFW_KEY_H:
        std::cout << "\n=== Key Shortcuts ===\n"
                  << "ESC: quit\n"
                  << "h: help\n"
                  << "b: body visualization mode\n"
                  << "k: cycle camera view (All -> Cam0 -> Cam1 -> ...)\n"
                  << "f: toggle skeleton fusion\n"
                  << "m: switch fusion mode (winner/weighted)\n"
                  << "r: start/stop CSV recording\n" << std::endl;
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

// ============================================================================
// Device Info Structure
// ============================================================================
struct DeviceInfo {
    k4a_device_t device = nullptr;
    k4abt_tracker_t tracker = nullptr;
    std::string serialNumber;
    int index = 0;
    bool isPrimary = false;
    k4a_calibration_t calibration;
    int depthWidth = 0;
    int depthHeight = 0;
};

// ============================================================================
// Get Device Serial Number
// ============================================================================
std::string GetDeviceSerialNumber(k4a_device_t device)
{
    size_t serialNumberSize = 0;
    k4a_device_get_serialnum(device, nullptr, &serialNumberSize);

    std::string serialNumber(serialNumberSize, '\0');
    k4a_device_get_serialnum(device, &serialNumber[0], &serialNumberSize);

    // Remove null terminator from string
    if (!serialNumber.empty() && serialNumber.back() == '\0') {
        serialNumber.pop_back();
    }
    return serialNumber;
}

// ============================================================================
// Calibration Loading
// ============================================================================
bool LoadCalibration(const std::string& path, CalibrationData& cal)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open calibration file: " << path << std::endl;
        return false;
    }

    try {
        json j;
        file >> j;

        cal.numDevices = j["num_devices"];
        cal.cameras.clear();
        cal.cameras.resize(cal.numDevices);

        for (const auto& camJson : j["calibrations"]) {
            CameraExtrinsics cam;
            cam.deviceIndex = camJson["device_index"];
            cam.serialNumber = camJson["serial_number"];
            cam.isValid = camJson["is_valid"];

            if (cam.isValid) {
                // Load 3x3 rotation matrix
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        cam.rotation[r][c] = camJson["rotation"][r][c];
                    }
                }
                // Load translation vector
                for (int i = 0; i < 3; i++) {
                    cam.translation[i] = camJson["translation"][i];
                }
            } else {
                // Identity for invalid/primary camera
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        cam.rotation[r][c] = (r == c) ? 1.0f : 0.0f;
                    }
                    cam.translation[r] = 0.0f;
                }
            }

            if (cam.deviceIndex >= 0 && cam.deviceIndex < cal.numDevices) {
                cal.cameras[cam.deviceIndex] = cam;
            }
        }

        cal.isLoaded = true;
        std::cout << "Loaded calibration for " << cal.numDevices << " cameras" << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing calibration JSON: " << e.what() << std::endl;
        return false;
    }
}

// ============================================================================
// Coordinate Transformation Functions
// ============================================================================

// Transform a 3D point: P_primary = R * P_camera + t
k4a_float3_t TransformPoint(const k4a_float3_t& point, const CameraExtrinsics& ext)
{
    k4a_float3_t result;
    result.xyz.x = ext.rotation[0][0] * point.xyz.x
                 + ext.rotation[0][1] * point.xyz.y
                 + ext.rotation[0][2] * point.xyz.z
                 + ext.translation[0];

    result.xyz.y = ext.rotation[1][0] * point.xyz.x
                 + ext.rotation[1][1] * point.xyz.y
                 + ext.rotation[1][2] * point.xyz.z
                 + ext.translation[1];

    result.xyz.z = ext.rotation[2][0] * point.xyz.x
                 + ext.rotation[2][1] * point.xyz.y
                 + ext.rotation[2][2] * point.xyz.z
                 + ext.translation[2];

    return result;
}

// Convert rotation matrix to quaternion
k4a_quaternion_t RotationMatrixToQuaternion(const float R[3][3])
{
    k4a_quaternion_t q;
    float trace = R[0][0] + R[1][1] + R[2][2];

    if (trace > 0) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        q.wxyz.w = 0.25f / s;
        q.wxyz.x = (R[2][1] - R[1][2]) * s;
        q.wxyz.y = (R[0][2] - R[2][0]) * s;
        q.wxyz.z = (R[1][0] - R[0][1]) * s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        float s = 2.0f * sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]);
        q.wxyz.w = (R[2][1] - R[1][2]) / s;
        q.wxyz.x = 0.25f * s;
        q.wxyz.y = (R[0][1] + R[1][0]) / s;
        q.wxyz.z = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        float s = 2.0f * sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]);
        q.wxyz.w = (R[0][2] - R[2][0]) / s;
        q.wxyz.x = (R[0][1] + R[1][0]) / s;
        q.wxyz.y = 0.25f * s;
        q.wxyz.z = (R[1][2] + R[2][1]) / s;
    } else {
        float s = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
        q.wxyz.w = (R[1][0] - R[0][1]) / s;
        q.wxyz.x = (R[0][2] + R[2][0]) / s;
        q.wxyz.y = (R[1][2] + R[2][1]) / s;
        q.wxyz.z = 0.25f * s;
    }

    return q;
}

// Multiply two quaternions: q_result = q1 * q2
k4a_quaternion_t QuaternionMultiply(const k4a_quaternion_t& q1, const k4a_quaternion_t& q2)
{
    k4a_quaternion_t result;
    result.wxyz.w = q1.wxyz.w*q2.wxyz.w - q1.wxyz.x*q2.wxyz.x
                  - q1.wxyz.y*q2.wxyz.y - q1.wxyz.z*q2.wxyz.z;
    result.wxyz.x = q1.wxyz.w*q2.wxyz.x + q1.wxyz.x*q2.wxyz.w
                  + q1.wxyz.y*q2.wxyz.z - q1.wxyz.z*q2.wxyz.y;
    result.wxyz.y = q1.wxyz.w*q2.wxyz.y - q1.wxyz.x*q2.wxyz.z
                  + q1.wxyz.y*q2.wxyz.w + q1.wxyz.z*q2.wxyz.x;
    result.wxyz.z = q1.wxyz.w*q2.wxyz.z + q1.wxyz.x*q2.wxyz.y
                  - q1.wxyz.y*q2.wxyz.x + q1.wxyz.z*q2.wxyz.w;
    return result;
}

// Transform joint orientation using rotation matrix
k4a_quaternion_t TransformOrientation(const k4a_quaternion_t& orientation,
                                       const CameraExtrinsics& ext)
{
    k4a_quaternion_t rotQuat = RotationMatrixToQuaternion(ext.rotation);
    return QuaternionMultiply(rotQuat, orientation);
}

// ============================================================================
// Body Matching Algorithm
// ============================================================================
float CalculateDistance(const k4a_float3_t& p1, const k4a_float3_t& p2)
{
    float dx = p1.xyz.x - p2.xyz.x;
    float dy = p1.xyz.y - p2.xyz.y;
    float dz = p1.xyz.z - p2.xyz.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

// Structure for transformed body data per camera
struct CameraBodyData {
    int deviceIndex;
    std::string serialNumber;
    std::vector<k4abt_body_t> transformedBodies;
};

std::vector<BodyMatch> MatchBodiesAcrossCameras(
    const std::vector<CameraBodyData>& cameraData,
    int numCameras)
{
    std::vector<BodyMatch> matches;
    if (numCameras == 0 || cameraData.empty()) return matches;

    // Track which bodies have been matched per camera
    std::vector<std::vector<bool>> used(numCameras);
    for (int c = 0; c < numCameras; c++) {
        if (c < static_cast<int>(cameraData.size())) {
            used[c].resize(cameraData[c].transformedBodies.size(), false);
        }
    }

    // Find camera with bodies to use as anchor (prefer camera 0)
    int anchorCamera = -1;
    for (int c = 0; c < numCameras && c < static_cast<int>(cameraData.size()); c++) {
        if (!cameraData[c].transformedBodies.empty()) {
            anchorCamera = c;
            break;
        }
    }

    if (anchorCamera < 0) return matches;

    // Start with anchor camera bodies as anchors
    for (size_t i = 0; i < cameraData[anchorCamera].transformedBodies.size(); i++) {
        BodyMatch match;
        match.bodyIndicesPerCamera.resize(numCameras, -1);
        match.bodyIndicesPerCamera[anchorCamera] = static_cast<int>(i);
        used[anchorCamera][i] = true;

        const k4a_float3_t& pelvis0 =
            cameraData[anchorCamera].transformedBodies[i].skeleton.joints[K4ABT_JOINT_PELVIS].position;

        // Find closest body in each other camera
        for (int c = 0; c < numCameras && c < static_cast<int>(cameraData.size()); c++) {
            if (c == anchorCamera) continue;

            float minDist = BODY_MATCH_THRESHOLD_MM;
            int bestMatch = -1;

            for (size_t j = 0; j < cameraData[c].transformedBodies.size(); j++) {
                if (used[c][j]) continue;

                const k4a_float3_t& pelvisC =
                    cameraData[c].transformedBodies[j].skeleton.joints[K4ABT_JOINT_PELVIS].position;

                float dist = CalculateDistance(pelvis0, pelvisC);
                if (dist < minDist) {
                    minDist = dist;
                    bestMatch = static_cast<int>(j);
                }
            }

            if (bestMatch >= 0) {
                match.bodyIndicesPerCamera[c] = bestMatch;
                used[c][bestMatch] = true;
            }
        }

        matches.push_back(match);
    }

    // Add unmatched bodies from other cameras as new bodies
    for (int c = 0; c < numCameras && c < static_cast<int>(cameraData.size()); c++) {
        if (c == anchorCamera) continue;
        for (size_t j = 0; j < cameraData[c].transformedBodies.size(); j++) {
            if (!used[c][j]) {
                BodyMatch match;
                match.bodyIndicesPerCamera.resize(numCameras, -1);
                match.bodyIndicesPerCamera[c] = static_cast<int>(j);
                matches.push_back(match);
            }
        }
    }

    return matches;
}

// ============================================================================
// Joint Fusion Algorithms
// ============================================================================
float ConfidenceToWeight(k4abt_joint_confidence_level_t conf)
{
    switch (conf) {
        case K4ABT_JOINT_CONFIDENCE_NONE:   return 0.0f;
        case K4ABT_JOINT_CONFIDENCE_LOW:    return 0.25f;
        case K4ABT_JOINT_CONFIDENCE_MEDIUM: return 0.6f;
        case K4ABT_JOINT_CONFIDENCE_HIGH:   return 1.0f;
        default: return 0.0f;
    }
}

FusedBody FuseBodyWinnerTakesAll(
    const BodyMatch& match,
    const std::vector<CameraBodyData>& cameraData,
    uint32_t fusedBodyId)
{
    FusedBody fused;
    fused.id = fusedBodyId;

    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        k4abt_joint_confidence_level_t bestConfidence = K4ABT_JOINT_CONFIDENCE_NONE;
        int bestCamera = -1;

        // Find camera with highest confidence for this joint
        for (size_t c = 0; c < match.bodyIndicesPerCamera.size(); c++) {
            int bodyIdx = match.bodyIndicesPerCamera[c];
            if (bodyIdx < 0 || c >= cameraData.size()) continue;

            const auto& joint = cameraData[c].transformedBodies[bodyIdx].skeleton.joints[j];

            if (joint.confidence_level > bestConfidence) {
                bestConfidence = joint.confidence_level;
                bestCamera = static_cast<int>(c);
            }
        }

        // Copy best joint
        if (bestCamera >= 0) {
            int bodyIdx = match.bodyIndicesPerCamera[bestCamera];
            const auto& srcJoint = cameraData[bestCamera].transformedBodies[bodyIdx].skeleton.joints[j];

            fused.joints[j].position = srcJoint.position;
            fused.joints[j].orientation = srcJoint.orientation;
            fused.joints[j].confidence = srcJoint.confidence_level;
            fused.joints[j].sourceDeviceIndex = bestCamera;
        } else {
            fused.joints[j].position = {0, 0, 0};
            fused.joints[j].orientation = {1, 0, 0, 0};
            fused.joints[j].confidence = K4ABT_JOINT_CONFIDENCE_NONE;
            fused.joints[j].sourceDeviceIndex = -1;
        }
    }

    return fused;
}

FusedBody FuseBodyWeightedAverage(
    const BodyMatch& match,
    const std::vector<CameraBodyData>& cameraData,
    uint32_t fusedBodyId)
{
    FusedBody fused;
    fused.id = fusedBodyId;

    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        float totalWeight = 0.0f;
        k4a_float3_t avgPosition = {0, 0, 0};
        k4a_quaternion_t bestOrientation = {1, 0, 0, 0};
        float bestOrientationWeight = 0.0f;
        int bestCamera = -1;
        k4abt_joint_confidence_level_t maxConf = K4ABT_JOINT_CONFIDENCE_NONE;

        // Accumulate weighted positions
        for (size_t c = 0; c < match.bodyIndicesPerCamera.size(); c++) {
            int bodyIdx = match.bodyIndicesPerCamera[c];
            if (bodyIdx < 0 || c >= cameraData.size()) continue;

            const auto& joint = cameraData[c].transformedBodies[bodyIdx].skeleton.joints[j];
            float weight = ConfidenceToWeight(joint.confidence_level);

            if (weight > 0) {
                avgPosition.xyz.x += joint.position.xyz.x * weight;
                avgPosition.xyz.y += joint.position.xyz.y * weight;
                avgPosition.xyz.z += joint.position.xyz.z * weight;
                totalWeight += weight;

                // Track best orientation and confidence
                if (weight > bestOrientationWeight) {
                    bestOrientationWeight = weight;
                    bestOrientation = joint.orientation;
                    bestCamera = static_cast<int>(c);
                }
                if (joint.confidence_level > maxConf) {
                    maxConf = joint.confidence_level;
                }
            }
        }

        if (totalWeight > 0) {
            fused.joints[j].position.xyz.x = avgPosition.xyz.x / totalWeight;
            fused.joints[j].position.xyz.y = avgPosition.xyz.y / totalWeight;
            fused.joints[j].position.xyz.z = avgPosition.xyz.z / totalWeight;
            fused.joints[j].orientation = bestOrientation;
            fused.joints[j].confidence = maxConf;
            fused.joints[j].sourceDeviceIndex = bestCamera;
        } else {
            fused.joints[j].position = {0, 0, 0};
            fused.joints[j].orientation = {1, 0, 0, 0};
            fused.joints[j].confidence = K4ABT_JOINT_CONFIDENCE_NONE;
            fused.joints[j].sourceDeviceIndex = -1;
        }
    }

    return fused;
}

// ============================================================================
// Main Fusion Pipeline
// ============================================================================
void PerformSkeletonFusion()
{
    if (!g_calibration.isLoaded) return;

    // Step 1: Quickly copy body data (minimize lock time)
    std::vector<std::vector<k4abt_body_t>> bodiesCopy;
    std::vector<int> deviceIndices;
    int numCameras = 0;

    {
        std::lock_guard<std::mutex> dataLock(g_bodyDataMutex);
        numCameras = static_cast<int>(g_deviceBodyData.size());
        if (numCameras == 0) return;

        bodiesCopy.resize(numCameras);
        deviceIndices.resize(numCameras);

        for (int i = 0; i < numCameras; i++) {
            deviceIndices[i] = g_deviceBodyData[i].deviceIndex;
            bodiesCopy[i] = g_deviceBodyData[i].bodies;  // Copy bodies
        }
    }
    // Lock released - capture threads can continue

    // Step 2: Transform bodies to primary frame (no lock needed)
    std::vector<CameraBodyData> cameraData(numCameras);

    for (int i = 0; i < numCameras; i++) {
        cameraData[i].deviceIndex = deviceIndices[i];

        // Find matching calibration by device index
        const CameraExtrinsics* extrinsics = nullptr;
        for (const auto& cam : g_calibration.cameras) {
            if (cam.deviceIndex == cameraData[i].deviceIndex) {
                extrinsics = &cam;
                break;
            }
        }

        // Transform bodies to primary frame
        for (const auto& body : bodiesCopy[i]) {
            k4abt_body_t transformedBody;
            transformedBody.id = body.id;

            for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                if (extrinsics && extrinsics->isValid) {
                    transformedBody.skeleton.joints[j].position =
                        TransformPoint(body.skeleton.joints[j].position, *extrinsics);
                    transformedBody.skeleton.joints[j].orientation =
                        TransformOrientation(body.skeleton.joints[j].orientation, *extrinsics);
                } else {
                    // Primary camera or uncalibrated: use as-is
                    transformedBody.skeleton.joints[j].position = body.skeleton.joints[j].position;
                    transformedBody.skeleton.joints[j].orientation = body.skeleton.joints[j].orientation;
                }
                transformedBody.skeleton.joints[j].confidence_level = body.skeleton.joints[j].confidence_level;
            }

            cameraData[i].transformedBodies.push_back(transformedBody);
        }
    }

    // Step 3: Match bodies across cameras
    std::vector<BodyMatch> matches = MatchBodiesAcrossCameras(cameraData, numCameras);

    // Step 4: Fuse matched bodies
    std::vector<FusedBody> fusedBodies;
    uint32_t fusedId = 0;

    for (const auto& match : matches) {
        FusedBody fused;
        if (g_fusionMode == FusionMode::WINNER_TAKES_ALL) {
            fused = FuseBodyWinnerTakesAll(match, cameraData, fusedId++);
        } else {
            fused = FuseBodyWeightedAverage(match, cameraData, fusedId++);
        }
        fusedBodies.push_back(fused);
    }

    // Step 5: Update global fused bodies
    {
        std::lock_guard<std::mutex> fusedLock(g_fusedBodyMutex);
        g_fusedBodies = std::move(fusedBodies);
    }
}

// ============================================================================
// Render Fused Bodies
// ============================================================================
void RenderFusedBodies(Window3dWrapper& window3d)
{
    std::lock_guard<std::mutex> lock(g_fusedBodyMutex);

    window3d.CleanJointsAndBones();

    for (const auto& body : g_fusedBodies) {
        Color color = g_bodyColors[body.id % g_bodyColors.size()];
        color.a = 0.6f;
        Color lowConfidenceColor = color;
        lowConfidenceColor.a = 0.2f;

        // Render joints
        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
            if (body.joints[j].confidence >= K4ABT_JOINT_CONFIDENCE_LOW) {
                window3d.AddJoint(
                    body.joints[j].position,
                    body.joints[j].orientation,
                    body.joints[j].confidence >= K4ABT_JOINT_CONFIDENCE_MEDIUM
                        ? color : lowConfidenceColor);
            }
        }

        // Render bones
        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++) {
            k4abt_joint_id_t j1 = g_boneList[boneIdx].first;
            k4abt_joint_id_t j2 = g_boneList[boneIdx].second;

            if (body.joints[j1].confidence >= K4ABT_JOINT_CONFIDENCE_LOW &&
                body.joints[j2].confidence >= K4ABT_JOINT_CONFIDENCE_LOW) {

                bool confidentBone =
                    body.joints[j1].confidence >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                    body.joints[j2].confidence >= K4ABT_JOINT_CONFIDENCE_MEDIUM;

                window3d.AddBone(
                    body.joints[j1].position,
                    body.joints[j2].position,
                    confidentBone ? color : lowConfidenceColor);
            }
        }
    }
}

// ============================================================================
// Device Capture Thread
// ============================================================================
void DeviceCaptureThread(DeviceInfo* deviceInfo, int dataIndex)
{
    std::cout << "[Device " << deviceInfo->index << "] Capture thread started (SN: "
              << deviceInfo->serialNumber << ")" << std::endl;

    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(
            deviceInfo->device, &sensorCapture, 1000);

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // Enqueue capture to body tracker
            k4a_wait_result_t queueResult = k4abt_tracker_enqueue_capture(
                deviceInfo->tracker, sensorCapture, 0);

            if (queueResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cerr << "[Device " << deviceInfo->index
                          << "] Failed to enqueue capture" << std::endl;
            }

            k4a_capture_release(sensorCapture);

            // Try to get body tracking result
            k4abt_frame_t bodyFrame = nullptr;
            k4a_wait_result_t popResult = k4abt_tracker_pop_result(
                deviceInfo->tracker, &bodyFrame, 0);

            if (popResult == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Extract body data
                std::lock_guard<std::mutex> lock(g_bodyDataMutex);

                DeviceBodyData& data = g_deviceBodyData[dataIndex];

                // Release previous images
                if (data.depthImage) {
                    k4a_image_release(data.depthImage);
                    data.depthImage = nullptr;
                }
                if (data.bodyIndexMap) {
                    k4a_image_release(data.bodyIndexMap);
                    data.bodyIndexMap = nullptr;
                }

                // Get capture and depth image
                k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
                data.depthImage = k4a_capture_get_depth_image(originalCapture);
                data.bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
                data.depthWidth = deviceInfo->depthWidth;
                data.depthHeight = deviceInfo->depthHeight;
                data.deviceIndex = deviceInfo->index;

                // Extract bodies
                data.bodies.clear();
                uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
                for (uint32_t i = 0; i < numBodies; i++)
                {
                    k4abt_body_t body;
                    if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) == K4A_RESULT_SUCCEEDED)
                    {
                        body.id = k4abt_frame_get_body_id(bodyFrame, i);
                        // Offset body ID by device index to avoid collisions
                        body.id = body.id + (deviceInfo->index * 100);
                        data.bodies.push_back(body);
                    }
                }

                data.hasNewData = true;

                // Record to CSV if recording is enabled
                if (g_isRecording && !data.bodies.empty()) {
                    RecordSkeletonFrame(deviceInfo->index, data.bodies);
                }

                k4a_capture_release(originalCapture);
                k4abt_frame_release(bodyFrame);
            }
        }
        else if (getCaptureResult == K4A_WAIT_RESULT_FAILED)
        {
            std::cerr << "[Device " << deviceInfo->index
                      << "] Failed to get capture" << std::endl;
            break;
        }
    }

    std::cout << "[Device " << deviceInfo->index << "] Capture thread stopped" << std::endl;
}

// ============================================================================
// Render All Bodies
// ============================================================================
void RenderAllBodies(Window3dWrapper& window3d)
{
    std::lock_guard<std::mutex> lock(g_bodyDataMutex);

    window3d.CleanJointsAndBones();

    for (auto& data : g_deviceBodyData)
    {
        if (!data.hasNewData) continue;

        // Filter by camera view mode (-1 = all, 0+ = specific camera)
        if (g_cameraViewMode >= 0 && data.deviceIndex != g_cameraViewMode) continue;

        // Render point cloud from first device with new data
        if (data.depthImage && data.bodyIndexMap && data.depthWidth > 0 && data.depthHeight > 0)
        {
            const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(data.bodyIndexMap);
            if (bodyIndexMapBuffer != nullptr)
            {
                std::vector<Color> pointCloudColors(data.depthWidth * data.depthHeight,
                                                     {0.4f, 0.4f, 0.4f, 0.3f});

                for (int i = 0; i < data.depthWidth * data.depthHeight; i++)
                {
                    uint8_t bodyIndex = bodyIndexMapBuffer[i];
                    if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
                    {
                        // Use device-specific color offset
                        int colorIdx = (bodyIndex + data.deviceIndex * 5) % g_bodyColors.size();
                        pointCloudColors[i] = g_bodyColors[colorIdx];
                    }
                }

                window3d.UpdatePointClouds(data.depthImage, pointCloudColors);
            }
        }

        // Render skeletons
        for (const auto& body : data.bodies)
        {
            Color color = g_bodyColors[body.id % g_bodyColors.size()];
            color.a = 0.4f;
            Color lowConfidenceColor = color;
            lowConfidenceColor.a = 0.1f;

            // Visualize joints
            for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
            {
                if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
                {
                    const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                    const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                    window3d.AddJoint(
                        jointPosition,
                        jointOrientation,
                        body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM
                            ? color : lowConfidenceColor);
                }
            }

            // Visualize bones
            for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
            {
                k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
                    body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
                {
                    bool confidentBone =
                        body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;

                    const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                    const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                    window3d.AddBone(joint1Position, joint2Position,
                                     confidentBone ? color : lowConfidenceColor);
                }
            }
        }
    }
}

// ============================================================================
// Print Usage
// ============================================================================
void PrintUsage()
{
    std::cout << "\n=== Multi-Device Body Tracking Viewer ===\n"
              << "USAGE: multi_device_body_viewer.exe [OPTIONS]\n\n"
              << "Depth Mode:\n"
              << "  NFOV_UNBINNED  - Narrow FOV Unbinned (default)\n"
              << "  WFOV_BINNED    - Wide FOV Binned\n\n"
              << "Processing Mode:\n"
              << "  CPU            - CPU processing mode\n"
              << "  CUDA           - CUDA processing mode\n"
              << "  DIRECTML       - DirectML processing mode (default on Windows)\n"
              << "  TENSORRT       - TensorRT processing mode\n\n"
              << "Multi-Camera Sync:\n"
              << "  --primary SERIAL     - Serial number of PRIMARY camera (sync hub master port)\n\n"
              << "Skeleton Fusion:\n"
              << "  --calibration FILE   - Load calibration file and enable fusion\n"
              << "  --fusion-mode MODE   - Fusion mode: winner | weighted (default: weighted)\n\n"
              << "CSV Recording:\n"
              << "  --output FILE        - Output CSV file path (default: skeleton_data_YYYYMMDD_HHMMSS.csv)\n\n"
              << "UDP Sync (for Unity):\n"
              << "  --udp-port PORT      - UDP listen port (default: 9000)\n"
              << "  --no-udp             - Disable UDP listener\n\n"
              << "Runtime Controls:\n"
              << "  K - Cycle camera view (All -> Cam0 -> Cam1 -> ...)\n"
              << "  R - Start/stop CSV recording\n"
              << "  F - Toggle skeleton fusion on/off\n"
              << "  M - Switch fusion mode (winner/weighted)\n"
              << "  B - Toggle body visualization\n"
              << "  H - Show help\n"
              << "  ESC - Quit\n"
              << std::endl;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "Multi-Device Body Tracking Viewer" << std::endl;
    std::cout << "For Orbbec Femto Bolt with K4A Wrapper" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Parse arguments
    k4a_depth_mode_t depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
    bool enableUdp = true;
    int udpPort = 9000;

    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "NFOV_UNBINNED") depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        else if (arg == "WFOV_BINNED") depthMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        else if (arg == "CPU") processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
        else if (arg == "CUDA") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
        else if (arg == "DIRECTML") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
        else if (arg == "TENSORRT") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT;
        else if (arg == "--calibration" && i + 1 < argc) {
            g_calibrationPath = argv[++i];
        }
        else if (arg == "--primary" && i + 1 < argc) {
            g_primarySerial = argv[++i];
        }
        else if (arg == "--fusion-mode" && i + 1 < argc) {
            std::string mode(argv[++i]);
            if (mode == "winner") g_fusionMode = FusionMode::WINNER_TAKES_ALL;
            else if (mode == "weighted") g_fusionMode = FusionMode::WEIGHTED_AVERAGE;
            else {
                std::cerr << "Unknown fusion mode: " << mode << std::endl;
                PrintUsage();
                return -1;
            }
        }
        else if (arg == "--output" && i + 1 < argc) {
            g_outputPath = argv[++i];
        }
        else if (arg == "--udp-port" && i + 1 < argc) {
            udpPort = std::stoi(argv[++i]);
        }
        else if (arg == "--no-udp") {
            enableUdp = false;
        }
        else if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return 0;
        }
        else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            PrintUsage();
            return -1;
        }
    }

    // Get device count
    uint32_t deviceCount = k4a_device_get_installed_count();
    std::cout << "Found " << deviceCount << " device(s)" << std::endl;

    if (deviceCount == 0)
    {
        std::cerr << "No devices found!" << std::endl;
        return -1;
    }

    if (deviceCount < 2)
    {
        std::cout << "Warning: Only 1 device found. For multi-device, connect 2+ cameras." << std::endl;
    }

    // Open all devices and get serial numbers
    std::vector<DeviceInfo> devices(deviceCount);

    for (uint32_t i = 0; i < deviceCount; i++)
    {
        devices[i].index = i;

        if (k4a_device_open(i, &devices[i].device) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to open device " << i << std::endl;
            return -1;
        }

        devices[i].serialNumber = GetDeviceSerialNumber(devices[i].device);

        std::cout << "Device " << i << ": SN=" << devices[i].serialNumber << std::endl;
    }

    // Determine primary camera
    if (!g_primarySerial.empty()) {
        // Find device matching the specified serial number
        bool found = false;
        for (uint32_t i = 0; i < deviceCount; i++) {
            if (devices[i].serialNumber == g_primarySerial) {
                devices[i].isPrimary = true;
                found = true;
                std::cout << "Device " << i << " (SN=" << devices[i].serialNumber
                          << ") set as PRIMARY (specified by --primary)" << std::endl;
            } else {
                devices[i].isPrimary = false;
            }
        }
        if (!found) {
            std::cerr << "Warning: Primary camera serial " << g_primarySerial
                      << " not found! Using device 0 as primary." << std::endl;
            devices[0].isPrimary = true;
        }
    } else {
        // Default: first device is primary
        for (uint32_t i = 0; i < deviceCount; i++) {
            devices[i].isPrimary = (i == 0);
        }
        if (deviceCount > 1) {
            std::cout << "Note: Use --primary SERIAL to specify sync hub master camera" << std::endl;
        }
    }

    // Print device roles
    for (uint32_t i = 0; i < deviceCount; i++) {
        std::cout << "Device " << i << ": "
                  << (devices[i].isPrimary ? "PRIMARY (MASTER)" : "SECONDARY (SUBORDINATE)") << std::endl;
    }

    // Load calibration if specified
    if (!g_calibrationPath.empty()) {
        std::cout << "\nLoading calibration from: " << g_calibrationPath << std::endl;
        if (LoadCalibration(g_calibrationPath, g_calibration)) {
            g_fusionEnabled = true;
            std::cout << "Skeleton fusion enabled (mode: "
                      << (g_fusionMode == FusionMode::WEIGHTED_AVERAGE ? "weighted" : "winner")
                      << ")" << std::endl;

            // Verify serial numbers match
            for (const auto& cam : g_calibration.cameras) {
                bool found = false;
                for (const auto& dev : devices) {
                    if (dev.serialNumber == cam.serialNumber) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    std::cerr << "Warning: Calibration camera " << cam.serialNumber
                              << " not found in connected devices" << std::endl;
                }
            }
        } else {
            std::cerr << "Warning: Failed to load calibration, fusion disabled" << std::endl;
            g_fusionEnabled = false;
        }
    }

    // Configure and start cameras
    // IMPORTANT: Start secondary devices first, then primary
    std::cout << "\nConfiguring devices..." << std::endl;

    // Configure all devices
    for (int i = deviceCount - 1; i >= 0; i--)  // Reverse order: secondary first
    {
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.depth_mode = depthMode;
        config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;

        // Set sync mode
        if (deviceCount > 1)
        {
            if (devices[i].isPrimary)
            {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
                config.subordinate_delay_off_master_usec = 0;
            }
            else
            {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
                // Add delay for each subordinate to avoid IR interference
                config.subordinate_delay_off_master_usec = 160 * (devices[i].index);
            }
        }
        else
        {
            config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        }

        std::cout << "Starting device " << i << " ("
                  << (devices[i].isPrimary ? "MASTER" : "SUBORDINATE") << ")..." << std::endl;

        if (k4a_device_start_cameras(devices[i].device, &config) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to start cameras on device " << i << std::endl;
            return -1;
        }

        // Get calibration
        if (k4a_device_get_calibration(devices[i].device, config.depth_mode,
                                        config.color_resolution, &devices[i].calibration) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to get calibration for device " << i << std::endl;
            return -1;
        }

        devices[i].depthWidth = devices[i].calibration.depth_camera_calibration.resolution_width;
        devices[i].depthHeight = devices[i].calibration.depth_camera_calibration.resolution_height;

        // Create body tracker
        k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
        trackerConfig.processing_mode = processingMode;

        std::cout << "Creating body tracker for device " << i << "..." << std::endl;
        if (k4abt_tracker_create(&devices[i].calibration, trackerConfig, &devices[i].tracker) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to create body tracker for device " << i << std::endl;
            return -1;
        }

        // Small delay between starting devices
        if (i > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    std::cout << "\nAll devices started successfully!" << std::endl;

    // Initialize global body data storage
    g_deviceBodyData.resize(deviceCount);
    g_numDevices = static_cast<int>(deviceCount);
    g_deviceSerialNumbers.resize(deviceCount);
    for (uint32_t i = 0; i < deviceCount; i++) {
        g_deviceBodyData[i].deviceIndex = static_cast<int>(i);
        g_deviceBodyData[i].depthWidth = devices[i].depthWidth;
        g_deviceBodyData[i].depthHeight = devices[i].depthHeight;
        g_deviceSerialNumbers[i] = devices[i].serialNumber;
    }

    // Create 3D window (use first device's calibration)
    std::cout << "Creating 3D window..." << std::endl;
    Window3dWrapper window3d;
    try {
        window3d.Create("Multi-Device Body Tracking", devices[0].calibration);
        window3d.SetCloseCallback(CloseCallback);
        window3d.SetKeyCallback(ProcessKey);
        std::cout << "3D window created successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create 3D window: " << e.what() << std::endl;
        return -1;
    }

    // Start capture threads
    std::vector<std::thread> captureThreads;
    for (uint32_t i = 0; i < deviceCount; i++)
    {
        captureThreads.emplace_back(DeviceCaptureThread, &devices[i], i);
    }

    // Initialize UDP listener for Unity sync
#ifdef _WIN32
    if (enableUdp) {
        if (InitUdpListener(udpPort)) {
            std::cout << "UDP sync enabled on port " << udpPort << std::endl;
        } else {
            std::cout << "Warning: Failed to initialize UDP listener" << std::endl;
        }
    }
#endif

    std::cout << "\nPress 'h' for help, ESC to quit\n" << std::endl;

    // Main render loop
    int frameCount = 0;
    try {
        while (s_isRunning)
        {
            try {
                // Process UDP commands from Unity
#ifdef _WIN32
                ProcessUdpCommands();
#endif

                if (g_fusionEnabled && g_calibration.isLoaded) {
                    PerformSkeletonFusion();
                    RenderFusedBodies(window3d);
                } else {
                    RenderAllBodies(window3d);
                }

                window3d.SetLayout3d(s_layoutMode);
                window3d.SetJointFrameVisualization(s_visualizeJointFrame);
                window3d.Render();

                frameCount++;
                if (frameCount % 100 == 0) {
                    std::cout << "[Frame " << frameCount << "] Running..." << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Exception in frame " << frameCount << ": " << e.what() << std::endl;
            }
        }
        std::cout << "Render loop ended normally (s_isRunning = false)" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception in render loop: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in render loop" << std::endl;
    }

    // Cleanup
    std::cout << "\nShutting down..." << std::endl;
    s_isRunning = false;

    // Stop recording if active
    if (g_isRecording) {
        StopRecording();
    }

    // Shutdown UDP listener
#ifdef _WIN32
    ShutdownUdpListener();
#endif

    // Wait for capture threads
    for (auto& thread : captureThreads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    // Release body data images
    for (auto& data : g_deviceBodyData)
    {
        if (data.depthImage) k4a_image_release(data.depthImage);
        if (data.bodyIndexMap) k4a_image_release(data.bodyIndexMap);
    }

    // Cleanup devices
    for (auto& device : devices)
    {
        if (device.tracker)
        {
            k4abt_tracker_shutdown(device.tracker);
            k4abt_tracker_destroy(device.tracker);
        }
        if (device.device)
        {
            k4a_device_stop_cameras(device.device);
            k4a_device_close(device.device);
        }
    }

    window3d.Delete();
    std::cout << "Done!" << std::endl;

    return 0;
}
