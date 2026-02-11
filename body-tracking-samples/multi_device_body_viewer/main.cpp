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
#include <opencv2/opencv.hpp>

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
// Helmet Overlay Types
// ============================================================================
struct Transform {
    cv::Mat rotation;       // 3x3 CV_64F
    cv::Mat translation;    // 3x1 CV_64F
    bool valid = false;
};

struct HelmetCBConfig {
    int rows = 4;
    int cols = 5;
    float squareMm = 30.0f;
    cv::Size patternSize() const { return cv::Size(cols, rows); }
};

struct FixedCameraFrame {
    cv::Mat colorImage;
    cv::Mat depthImage;     // CV_16UC1
    int deviceIndex = -1;
    bool hasNewData = false;
};

struct FixedCameraInfo {
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = nullptr;
    int deviceIndex = -1;
    int colorWidth = 0;
    int colorHeight = 0;
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

// Helmet overlay state
bool g_helmetMode = false;
bool g_helmetOverlayVisible = true;
std::string g_helmetSerial;
int g_helmetDeviceIndex = -1;
Transform g_tCheckerToA;
HelmetCBConfig g_helmetCB;
k4a_calibration_t g_helmetCalibration;
std::string g_tCheckerToAPath;

// Helmet pose (written by detector thread, read by main thread)
Transform g_helmetPose;
bool g_helmetPoseValid = false;
std::chrono::steady_clock::time_point g_helmetPoseTimestamp;
std::mutex g_helmetPoseMutex;

// Helmet color frame (written by helmet capture thread, read by main thread)
cv::Mat g_helmetColorImage;
bool g_helmetColorNew = false;
std::mutex g_helmetColorMutex;

// Fixed camera frames for checkerboard detection
std::vector<FixedCameraFrame> g_fixedFrames;
std::mutex g_fixedFramesMutex;

// Fixed camera calibration/transform handles
std::vector<FixedCameraInfo> g_fixedCameraInfos;

// Mapping from device index to fixed camera array index
std::map<int, int> g_deviceToFixedIndex;

// Detector thread
std::atomic<bool> g_detectorRunning{false};
std::thread g_detectorThread;

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
    case GLFW_KEY_V:
        if (g_helmetMode) {
            g_helmetOverlayVisible = !g_helmetOverlayVisible;
            std::cout << "Helmet overlay: " << (g_helmetOverlayVisible ? "ON" : "OFF") << std::endl;
        } else {
            std::cout << "Helmet mode not enabled (use --helmet-serial)" << std::endl;
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
                  << "r: start/stop CSV recording\n"
                  << "v: toggle helmet camera overlay\n" << std::endl;
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
// Helmet Overlay: Utility Functions
// ============================================================================

cv::Mat K4AImageToMat(k4a_image_t image)
{
    int width = k4a_image_get_width_pixels(image);
    int height = k4a_image_get_height_pixels(image);
    k4a_image_format_t format = k4a_image_get_format(image);
    uint8_t* buffer = k4a_image_get_buffer(image);

    if (format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
    {
        return cv::Mat(height, width, CV_8UC4, buffer).clone();
    }
    else if (format == K4A_IMAGE_FORMAT_DEPTH16)
    {
        return cv::Mat(height, width, CV_16UC1, buffer).clone();
    }

    return cv::Mat();
}

bool HelmetDetectCheckerboardCorners(const cv::Mat& colorImage,
                                      std::vector<cv::Point2f>& corners,
                                      cv::Size patternSize)
{
    cv::Mat gray;
    if (colorImage.channels() == 4)
    {
        cv::cvtColor(colorImage, gray, cv::COLOR_BGRA2GRAY);
    }
    else if (colorImage.channels() == 3)
    {
        cv::cvtColor(colorImage, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = colorImage;
    }

    bool found = cv::findChessboardCorners(
        gray, patternSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK
    );

    if (found)
    {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    return found;
}

bool HelmetConvert2DTo3D(const k4a_calibration_t& calibration,
                          k4a_transformation_t transformation,
                          const cv::Mat& depthImage,
                          const std::vector<cv::Point2f>& corners2D,
                          std::vector<cv::Point3f>& points3D)
{
    points3D.clear();

    int colorWidth = calibration.color_camera_calibration.resolution_width;
    int colorHeight = calibration.color_camera_calibration.resolution_height;

    // Create k4a depth image from cv::Mat
    k4a_image_t depthK4a = nullptr;
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        depthImage.cols, depthImage.rows,
        depthImage.cols * (int)sizeof(uint16_t), &depthK4a);
    memcpy(k4a_image_get_buffer(depthK4a), depthImage.data,
        depthImage.total() * sizeof(uint16_t));

    // Transform depth to color camera space
    k4a_image_t transformedDepth = nullptr;
    if (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                          colorWidth, colorHeight,
                          colorWidth * (int)sizeof(uint16_t),
                          &transformedDepth) != K4A_RESULT_SUCCEEDED)
    {
        k4a_image_release(depthK4a);
        return false;
    }

    if (k4a_transformation_depth_image_to_color_camera(
            transformation, depthK4a, transformedDepth) != K4A_RESULT_SUCCEEDED)
    {
        k4a_image_release(depthK4a);
        k4a_image_release(transformedDepth);
        return false;
    }

    uint16_t* depthBuffer = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(transformedDepth));
    int depthStride = k4a_image_get_stride_bytes(transformedDepth) / (int)sizeof(uint16_t);

    for (const auto& corner : corners2D)
    {
        int x = static_cast<int>(std::round(corner.x));
        int y = static_cast<int>(std::round(corner.y));

        if (x < 0 || x >= colorWidth || y < 0 || y >= colorHeight)
        {
            k4a_image_release(depthK4a);
            k4a_image_release(transformedDepth);
            return false;
        }

        // Expanding ring search for valid depth
        const int maxSearchRadius = 20;
        float depthMm = 0;
        for (int r = 0; r <= maxSearchRadius; r++)
        {
            float depthSum = 0;
            int validCount = 0;
            for (int dy = -r; dy <= r; dy++)
            {
                for (int dx = -r; dx <= r; dx++)
                {
                    if (r > 0 && std::abs(dx) < r && std::abs(dy) < r) continue;
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < colorWidth && ny >= 0 && ny < colorHeight)
                    {
                        uint16_t d = depthBuffer[ny * depthStride + nx];
                        if (d > 0)
                        {
                            depthSum += d;
                            validCount++;
                        }
                    }
                }
            }
            if (validCount > 0)
            {
                depthMm = depthSum / validCount;
                break;
            }
        }

        if (depthMm == 0)
        {
            k4a_image_release(depthK4a);
            k4a_image_release(transformedDepth);
            return false;
        }

        k4a_float2_t sourcePoint2D = { corner.x, corner.y };
        k4a_float3_t targetPoint3D;
        int valid = 0;

        if (k4a_calibration_2d_to_3d(&calibration,
                                      &sourcePoint2D, depthMm,
                                      K4A_CALIBRATION_TYPE_COLOR,
                                      K4A_CALIBRATION_TYPE_COLOR,
                                      &targetPoint3D, &valid) != K4A_RESULT_SUCCEEDED || !valid)
        {
            k4a_image_release(depthK4a);
            k4a_image_release(transformedDepth);
            return false;
        }

        points3D.push_back(cv::Point3f(targetPoint3D.xyz.x, targetPoint3D.xyz.y, targetPoint3D.xyz.z));
    }

    k4a_image_release(depthK4a);
    k4a_image_release(transformedDepth);
    return true;
}

bool LoadHelmetTransform(const std::string& path, Transform& t)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open transform: " << path << std::endl;
        return false;
    }

    try {
        json j;
        file >> j;

        t.rotation = cv::Mat(3, 3, CV_64F);
        t.translation = cv::Mat(3, 1, CV_64F);

        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                t.rotation.at<double>(r, c) = j["rotation"][r][c];
            }
        }

        for (int i = 0; i < 3; i++) {
            t.translation.at<double>(i, 0) = j["translation"][i];
        }

        t.valid = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing transform: " << e.what() << std::endl;
        return false;
    }
}

Transform ComputeCheckerboardPose(const std::vector<cv::Point3f>& points3D,
                                   cv::Size patternSize)
{
    Transform result;

    if (points3D.size() < 4) return result;

    // Compute centroid
    cv::Point3f centroid(0, 0, 0);
    for (const auto& p : points3D) {
        centroid += p;
    }
    centroid *= (1.0f / points3D.size());

    // Use first row direction as X axis
    cv::Point3f x_axis = points3D[patternSize.width - 1] - points3D[0];
    x_axis /= (float)cv::norm(x_axis);

    // Use first column direction as Y axis
    cv::Point3f y_axis = points3D[(patternSize.height - 1) * patternSize.width] - points3D[0];
    y_axis /= (float)cv::norm(y_axis);

    // Z axis from cross product
    cv::Point3f z_axis = x_axis.cross(y_axis);
    z_axis /= (float)cv::norm(z_axis);

    // Re-orthogonalize Y
    y_axis = z_axis.cross(x_axis);
    y_axis /= (float)cv::norm(y_axis);

    result.rotation = cv::Mat(3, 3, CV_64F);
    result.rotation.at<double>(0, 0) = x_axis.x;
    result.rotation.at<double>(1, 0) = x_axis.y;
    result.rotation.at<double>(2, 0) = x_axis.z;
    result.rotation.at<double>(0, 1) = y_axis.x;
    result.rotation.at<double>(1, 1) = y_axis.y;
    result.rotation.at<double>(2, 1) = y_axis.z;
    result.rotation.at<double>(0, 2) = z_axis.x;
    result.rotation.at<double>(1, 2) = z_axis.y;
    result.rotation.at<double>(2, 2) = z_axis.z;

    result.translation = cv::Mat(3, 1, CV_64F);
    result.translation.at<double>(0, 0) = centroid.x;
    result.translation.at<double>(1, 0) = centroid.y;
    result.translation.at<double>(2, 0) = centroid.z;

    result.valid = true;
    return result;
}

void TransformPointsToWorld(std::vector<cv::Point3f>& points,
                             const CameraExtrinsics& ext)
{
    cv::Mat R(3, 3, CV_32F);
    cv::Mat t(3, 1, CV_32F);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            R.at<float>(r, c) = ext.rotation[r][c];
        }
        t.at<float>(r, 0) = ext.translation[r];
    }

    for (auto& p : points) {
        cv::Mat pt = (cv::Mat_<float>(3, 1) << p.x, p.y, p.z);
        cv::Mat result = R * pt + t;
        p.x = result.at<float>(0);
        p.y = result.at<float>(1);
        p.z = result.at<float>(2);
    }
}

// ============================================================================
// Helmet Detector Thread
// ============================================================================
void HelmetDetectorThread()
{
    std::cout << "[Helmet] Detector thread started" << std::endl;

    while (g_detectorRunning)
    {
        // Copy latest frames from fixed cameras
        std::vector<FixedCameraFrame> frames;
        {
            std::lock_guard<std::mutex> lock(g_fixedFramesMutex);
            frames = g_fixedFrames;
            // Clear new data flags
            for (auto& f : g_fixedFrames) {
                f.hasNewData = false;
            }
        }

        bool detected = false;
        cv::Size patternSize = g_helmetCB.patternSize();

        for (size_t i = 0; i < frames.size(); i++)
        {
            if (!frames[i].hasNewData || frames[i].colorImage.empty() || frames[i].depthImage.empty())
                continue;

            // Try to detect checkerboard
            std::vector<cv::Point2f> corners;
            if (!HelmetDetectCheckerboardCorners(frames[i].colorImage, corners, patternSize))
                continue;

            // Find the fixed camera info for this device
            int devIdx = frames[i].deviceIndex;
            FixedCameraInfo* camInfo = nullptr;
            for (auto& fci : g_fixedCameraInfos) {
                if (fci.deviceIndex == devIdx) {
                    camInfo = &fci;
                    break;
                }
            }
            if (!camInfo || !camInfo->transformation) continue;

            // Convert 2D corners to 3D in camera frame
            std::vector<cv::Point3f> points3D;
            if (!HelmetConvert2DTo3D(camInfo->calibration, camInfo->transformation,
                                      frames[i].depthImage, corners, points3D))
                continue;

            // Transform 3D points to world frame using calibration extrinsics
            // Match by serial number (device indices shift when cameras are added)
            const CameraExtrinsics* ext = nullptr;
            if (devIdx >= 0 && devIdx < static_cast<int>(g_deviceSerialNumbers.size()))
            {
                const std::string& serial = g_deviceSerialNumbers[devIdx];
                for (const auto& cam : g_calibration.cameras) {
                    if (cam.serialNumber == serial) {
                        ext = &cam;
                        break;
                    }
                }
            }
            if (ext) {
                TransformPointsToWorld(points3D, *ext);
            }

            // Compute checkerboard pose in world frame
            Transform checkerPose = ComputeCheckerboardPose(points3D, patternSize);
            if (!checkerPose.valid) continue;

            // Compose with T_checker_to_A to get helmet camera pose in world
            // helmetR = checkerR * tCheckerToA.R
            // helmetT = checkerR * tCheckerToA.t + checkerT
            Transform helmetPose;
            helmetPose.rotation = checkerPose.rotation * g_tCheckerToA.rotation;
            helmetPose.translation = checkerPose.rotation * g_tCheckerToA.translation
                                   + checkerPose.translation;
            helmetPose.valid = true;

            // Write to shared state
            {
                std::lock_guard<std::mutex> lock(g_helmetPoseMutex);
                g_helmetPose = helmetPose;
                g_helmetPoseValid = true;
                g_helmetPoseTimestamp = std::chrono::steady_clock::now();
            }

            detected = true;
            break;  // Use first successful detection
        }

        if (!detected)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    std::cout << "[Helmet] Detector thread stopped" << std::endl;
}

// ============================================================================
// Helmet Overlay: Draw skeleton on 2D image
// ============================================================================
void DrawSkeleton2D(cv::Mat& image,
                     const std::vector<cv::Point2f>& joints2D,
                     const std::vector<bool>& visible,
                     const std::vector<k4abt_joint_confidence_level_t>& confidences,
                     const cv::Scalar& color)
{
    // Draw bones
    for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
    {
        int j1 = g_boneList[boneIdx].first;
        int j2 = g_boneList[boneIdx].second;

        if (j1 < (int)visible.size() && j2 < (int)visible.size() &&
            visible[j1] && visible[j2] &&
            confidences[j1] >= K4ABT_JOINT_CONFIDENCE_LOW &&
            confidences[j2] >= K4ABT_JOINT_CONFIDENCE_LOW)
        {
            int thickness = (confidences[j1] >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                           confidences[j2] >= K4ABT_JOINT_CONFIDENCE_MEDIUM) ? 2 : 1;
            cv::line(image, joints2D[j1], joints2D[j2], color, thickness, cv::LINE_AA);
        }
    }

    // Draw joints
    for (int j = 0; j < (int)visible.size(); j++)
    {
        if (visible[j] && confidences[j] >= K4ABT_JOINT_CONFIDENCE_LOW)
        {
            int radius = (confidences[j] >= K4ABT_JOINT_CONFIDENCE_MEDIUM) ? 5 : 3;
            cv::circle(image, joints2D[j], radius, color, -1, cv::LINE_AA);
        }
    }
}

// ============================================================================
// Helmet Overlay: Render fused skeleton on helmet camera image
// ============================================================================
void RenderHelmetOverlay()
{
    if (!g_helmetOverlayVisible) return;

    // Copy helmet color image
    cv::Mat display;
    {
        std::lock_guard<std::mutex> lock(g_helmetColorMutex);
        if (!g_helmetColorNew || g_helmetColorImage.empty()) return;
        display = g_helmetColorImage.clone();
        g_helmetColorNew = false;
    }

    // Convert BGRA to BGR for display
    if (display.channels() == 4)
    {
        cv::cvtColor(display, display, cv::COLOR_BGRA2BGR);
    }

    // Copy helmet pose
    Transform pose;
    bool poseValid = false;
    std::chrono::steady_clock::time_point poseTime;
    {
        std::lock_guard<std::mutex> lock(g_helmetPoseMutex);
        pose = g_helmetPose;
        poseValid = g_helmetPoseValid;
        poseTime = g_helmetPoseTimestamp;
    }

    // Check staleness (500ms timeout)
    auto now = std::chrono::steady_clock::now();
    auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - poseTime).count();
    bool stale = (ageMs > 500);

    // Copy fused bodies
    std::vector<FusedBody> bodies;
    {
        std::lock_guard<std::mutex> lock(g_fusedBodyMutex);
        bodies = g_fusedBodies;
    }

    if (poseValid && !stale && pose.valid)
    {
        // Compute inverse transform: world -> helmet camera
        // P_helmet = R^T * (P_world - t)
        cv::Mat Rt = pose.rotation.t();  // 3x3
        cv::Mat tNeg = -Rt * pose.translation;  // 3x1

        for (const auto& body : bodies)
        {
            Color bodyColor = g_bodyColors[body.id % g_bodyColors.size()];
            cv::Scalar cvColor(
                static_cast<int>(bodyColor.b * 255),
                static_cast<int>(bodyColor.g * 255),
                static_cast<int>(bodyColor.r * 255)
            );

            std::vector<cv::Point2f> joints2D(K4ABT_JOINT_COUNT);
            std::vector<bool> visible(K4ABT_JOINT_COUNT, false);
            std::vector<k4abt_joint_confidence_level_t> confidences(K4ABT_JOINT_COUNT);

            for (int j = 0; j < K4ABT_JOINT_COUNT; j++)
            {
                confidences[j] = body.joints[j].confidence;
                if (body.joints[j].confidence < K4ABT_JOINT_CONFIDENCE_LOW) continue;

                // Transform world -> helmet camera
                cv::Mat pw = (cv::Mat_<double>(3, 1) <<
                    (double)body.joints[j].position.xyz.x,
                    (double)body.joints[j].position.xyz.y,
                    (double)body.joints[j].position.xyz.z);

                cv::Mat pCam = Rt * pw + tNeg;

                // Only show joints in front of camera (positive Z)
                if (pCam.at<double>(2) <= 0) continue;

                // Project to 2D using helmet calibration
                k4a_float3_t point3d;
                point3d.xyz.x = (float)pCam.at<double>(0);
                point3d.xyz.y = (float)pCam.at<double>(1);
                point3d.xyz.z = (float)pCam.at<double>(2);

                k4a_float2_t point2d;
                int validProj = 0;

                if (k4a_calibration_3d_to_2d(&g_helmetCalibration,
                                              &point3d,
                                              K4A_CALIBRATION_TYPE_COLOR,
                                              K4A_CALIBRATION_TYPE_COLOR,
                                              &point2d, &validProj) == K4A_RESULT_SUCCEEDED && validProj)
                {
                    // Check bounds
                    if (point2d.xy.x >= 0 && point2d.xy.x < display.cols &&
                        point2d.xy.y >= 0 && point2d.xy.y < display.rows)
                    {
                        joints2D[j] = cv::Point2f(point2d.xy.x, point2d.xy.y);
                        visible[j] = true;
                    }
                }
            }

            DrawSkeleton2D(display, joints2D, visible, confidences, cvColor);
        }

        // Status text
        std::string status = "TRACKING (age: " + std::to_string(ageMs) + "ms)";
        cv::putText(display, status, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
    else
    {
        // No valid pose
        std::string status = stale ? "CHECKERBOARD LOST (stale)" : "NO CHECKERBOARD";
        cv::putText(display, status, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    cv::imshow("Helmet Camera", display);
    cv::waitKey(1);
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

    // Periodic fusion diagnostic (every 300 frames)
    static int fusionDiagCounter = 0;
    bool showDiag = (fusionDiagCounter++ % 300 == 0);

    // Diagnostic: log when bodies first appear in fusion
    {
        static bool fusionFirstBody = false;
        if (!fusionFirstBody) {
            int totalBodies = 0;
            for (int i = 0; i < numCameras; i++)
                totalBodies += static_cast<int>(bodiesCopy[i].size());
            if (totalBodies > 0) {
                std::cerr << "[DIAG-FUSION] First bodies in fusion pipeline: "
                          << totalBodies << " total across " << numCameras << " cameras"
                          << std::endl << std::flush;
                fusionFirstBody = true;
            }
        }
    }

    // Step 2: Transform bodies to primary frame (no lock needed)
    std::vector<CameraBodyData> cameraData(numCameras);

    for (int i = 0; i < numCameras; i++) {
        cameraData[i].deviceIndex = deviceIndices[i];

        // Find matching calibration by serial number (device indices may change
        // when cameras are added/removed, but serials are stable)
        const CameraExtrinsics* extrinsics = nullptr;
        std::string serialForDiag = "?";
        if (cameraData[i].deviceIndex >= 0 &&
            cameraData[i].deviceIndex < static_cast<int>(g_deviceSerialNumbers.size()))
        {
            serialForDiag = g_deviceSerialNumbers[cameraData[i].deviceIndex];
            for (const auto& cam : g_calibration.cameras) {
                if (cam.serialNumber == serialForDiag) {
                    extrinsics = &cam;
                    break;
                }
            }
        }

        if (showDiag) {
            std::cout << "[Fusion] slot=" << i
                      << " dev=" << deviceIndices[i]
                      << " SN=" << serialForDiag
                      << " bodies=" << bodiesCopy[i].size()
                      << " calib=" << (extrinsics ? (extrinsics->isValid ? "YES" : "IDENTITY") : "NOT_FOUND")
                      << std::endl;
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

    // Diagnostic: log after transform step
    {
        static bool transformLogged = false;
        if (!transformLogged) {
            int totalTransformed = 0;
            for (int i = 0; i < numCameras; i++)
                totalTransformed += static_cast<int>(cameraData[i].transformedBodies.size());
            if (totalTransformed > 0) {
                std::cerr << "[DIAG-FUSION] Transform complete: " << totalTransformed
                          << " bodies transformed" << std::endl << std::flush;
                transformLogged = true;
            }
        }
    }

    // Step 3: Match bodies across cameras
    std::vector<BodyMatch> matches = MatchBodiesAcrossCameras(cameraData, numCameras);

    if (showDiag) {
        std::cout << "[Fusion] matches=" << matches.size() << std::endl;
        for (size_t m = 0; m < matches.size(); m++) {
            std::cout << "[Fusion]   match " << m << ": cams=[";
            for (size_t c = 0; c < matches[m].bodyIndicesPerCamera.size(); c++) {
                if (c > 0) std::cout << ",";
                std::cout << matches[m].bodyIndicesPerCamera[c];
            }
            std::cout << "]" << std::endl;
        }

        // Print pelvis positions (raw and transformed) for each camera with bodies
        for (int i = 0; i < numCameras; i++) {
            for (size_t b = 0; b < bodiesCopy[i].size(); b++) {
                const auto& rawPelvis = bodiesCopy[i][b].skeleton.joints[K4ABT_JOINT_PELVIS].position;
                const auto& tPelvis = cameraData[i].transformedBodies[b].skeleton.joints[K4ABT_JOINT_PELVIS].position;
                std::cout << "[Fusion]   cam" << i << " body" << b
                          << " raw=(" << (int)rawPelvis.xyz.x << "," << (int)rawPelvis.xyz.y << "," << (int)rawPelvis.xyz.z << ")"
                          << " xform=(" << (int)tPelvis.xyz.x << "," << (int)tPelvis.xyz.y << "," << (int)tPelvis.xyz.z << ")"
                          << std::endl;
            }
        }
    }

    // Diagnostic: log after matching
    {
        static bool matchLogged = false;
        if (!matchLogged && !matches.empty()) {
            std::cerr << "[DIAG-FUSION] Matching complete: " << matches.size()
                      << " matches found" << std::endl << std::flush;
            matchLogged = true;
        }
    }

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

    // Diagnostic: log after fusion
    {
        static bool fuseLogged = false;
        if (!fuseLogged && !fusedBodies.empty()) {
            std::cerr << "[DIAG-FUSION] Fusion complete: " << fusedBodies.size()
                      << " fused bodies" << std::endl << std::flush;
            // Print first body's pelvis position
            const auto& pelvis = fusedBodies[0].joints[K4ABT_JOINT_PELVIS];
            std::cerr << "[DIAG-FUSION] Pelvis: ("
                      << pelvis.position.xyz.x << ", "
                      << pelvis.position.xyz.y << ", "
                      << pelvis.position.xyz.z << ") conf="
                      << pelvis.confidence << std::endl << std::flush;
            fuseLogged = true;
        }
    }

    // Step 5: Update global fused bodies
    {
        std::lock_guard<std::mutex> fusedLock(g_fusedBodyMutex);
        g_fusedBodies = std::move(fusedBodies);
    }

    // Diagnostic: log after global update
    {
        static bool updateLogged = false;
        if (!updateLogged && !g_fusedBodies.empty()) {
            std::cerr << "[DIAG-FUSION] Global fused bodies updated: "
                      << g_fusedBodies.size() << std::endl << std::flush;
            updateLogged = true;
        }
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
// Helmet Camera Capture Thread (color only, no body tracking)
// ============================================================================
void HelmetCameraCaptureThread(DeviceInfo* deviceInfo)
{
    std::cout << "[Helmet] Capture thread started (SN: "
              << deviceInfo->serialNumber << ")" << std::endl;

    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(
            deviceInfo->device, &sensorCapture, 1000);

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_image_t colorImg = k4a_capture_get_color_image(sensorCapture);
            if (colorImg)
            {
                std::lock_guard<std::mutex> hlock(g_helmetColorMutex);
                g_helmetColorImage = K4AImageToMat(colorImg);
                g_helmetColorNew = true;
                k4a_image_release(colorImg);
            }
            k4a_capture_release(sensorCapture);
        }
        else if (getCaptureResult == K4A_WAIT_RESULT_FAILED)
        {
            std::cerr << "[Helmet] Failed to get capture" << std::endl;
            break;
        }
    }

    std::cout << "[Helmet] Capture thread stopped" << std::endl;
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

                // Helmet mode: capture color+depth from fixed cameras for checkerboard detection
                if (g_helmetMode)
                {
                    auto it = g_deviceToFixedIndex.find(deviceInfo->index);
                    if (it != g_deviceToFixedIndex.end())
                    {
                        k4a_image_t colorImg = k4a_capture_get_color_image(originalCapture);
                        if (colorImg)
                        {
                            int fixedIdx = it->second;
                            std::lock_guard<std::mutex> flock(g_fixedFramesMutex);
                            g_fixedFrames[fixedIdx].colorImage = K4AImageToMat(colorImg);
                            if (data.depthImage) {
                                g_fixedFrames[fixedIdx].depthImage = K4AImageToMat(data.depthImage);
                            }
                            g_fixedFrames[fixedIdx].deviceIndex = deviceInfo->index;
                            g_fixedFrames[fixedIdx].hasNewData = true;
                            k4a_image_release(colorImg);
                        }
                    }
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
              << "Helmet Camera Overlay:\n"
              << "  --helmet-serial SN   - Serial number of helmet-mounted camera\n"
              << "  --t-checker-to-a FILE - Path to T_checker_to_A.json transform\n"
              << "  --helmet-cb-rows N   - Checkerboard inner rows (default: 4)\n"
              << "  --helmet-cb-cols N   - Checkerboard inner cols (default: 5)\n"
              << "  --helmet-cb-square N - Checkerboard square size in mm (default: 30)\n\n"
              << "Runtime Controls:\n"
              << "  K - Cycle camera view (All -> Cam0 -> Cam1 -> ...)\n"
              << "  R - Start/stop CSV recording\n"
              << "  F - Toggle skeleton fusion on/off\n"
              << "  M - Switch fusion mode (winner/weighted)\n"
              << "  V - Toggle helmet camera overlay\n"
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
        else if (arg == "--helmet-serial" && i + 1 < argc) {
            g_helmetSerial = argv[++i];
            g_helmetMode = true;
        }
        else if (arg == "--t-checker-to-a" && i + 1 < argc) {
            g_tCheckerToAPath = argv[++i];
        }
        else if (arg == "--helmet-cb-rows" && i + 1 < argc) {
            g_helmetCB.rows = std::stoi(argv[++i]);
        }
        else if (arg == "--helmet-cb-cols" && i + 1 < argc) {
            g_helmetCB.cols = std::stoi(argv[++i]);
        }
        else if (arg == "--helmet-cb-square" && i + 1 < argc) {
            g_helmetCB.squareMm = std::stof(argv[++i]);
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

    // Identify helmet device index early (needed before device config loop)
    if (g_helmetMode) {
        for (uint32_t i = 0; i < deviceCount; i++) {
            if (devices[i].serialNumber == g_helmetSerial) {
                g_helmetDeviceIndex = static_cast<int>(i);
                std::cout << "[Helmet] Identified helmet camera at device " << i
                          << " (SN: " << devices[i].serialNumber << ")" << std::endl;
                break;
            }
        }
        if (g_helmetDeviceIndex < 0) {
            std::cerr << "Warning: Helmet camera serial " << g_helmetSerial
                      << " not found! Disabling helmet mode." << std::endl;
            g_helmetMode = false;
        }
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

    // Validate helmet mode requires calibration
    if (g_helmetMode && !g_calibration.isLoaded) {
        std::cerr << "Error: Helmet mode requires --calibration. Disabling helmet mode." << std::endl;
        g_helmetMode = false;
    }

    // Configure and start cameras
    // IMPORTANT: Start secondary devices first, then primary
    std::cout << "\nConfiguring devices..." << std::endl;

    // Configure all devices
    int subordinateCount = 0;
    for (int i = deviceCount - 1; i >= 0; i--)  // Reverse order: secondary first
    {
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.depth_mode = depthMode;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;

        // Enable color camera when helmet mode is active
        if (g_helmetMode)
        {
            config.color_resolution = K4A_COLOR_RESOLUTION_720P;
            config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
            config.synchronized_images_only = true;
        }
        else
        {
            config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
        }

        // Subordinate delay prevents IR interference between depth cameras.
        // Each subordinate offsets by 160us * N (N=1,2,...) from master.
        // Reference: Orbbec Femto Bolt multi-device sync documentation
        if (deviceCount > 1)
        {
            if (devices[i].isPrimary)
            {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
            }
            else
            {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
                subordinateCount++;
                config.subordinate_delay_off_master_usec = 160 * subordinateCount;
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

        // Create body tracker (skip for helmet camera - it only provides color)
        if (g_helmetMode && i == g_helmetDeviceIndex)
        {
            devices[i].tracker = nullptr;
            std::cout << "Skipping body tracker for device " << i << " (helmet camera)" << std::endl;
        }
        else
        {
            k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
            trackerConfig.processing_mode = processingMode;

            std::cout << "Creating body tracker for device " << i << "..." << std::endl;
            if (k4abt_tracker_create(&devices[i].calibration, trackerConfig, &devices[i].tracker) != K4A_RESULT_SUCCEEDED)
            {
                std::cerr << "Failed to create body tracker for device " << i << std::endl;
                return -1;
            }
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

    // Setup helmet overlay if enabled
    if (g_helmetMode)
    {
        // g_helmetDeviceIndex already set during early identification
        g_helmetCalibration = devices[g_helmetDeviceIndex].calibration;
        {
            // Load T_checker_to_A transform
            if (g_tCheckerToAPath.empty()) {
                g_tCheckerToAPath = "T_checker_to_A.json";
            }
            if (!LoadHelmetTransform(g_tCheckerToAPath, g_tCheckerToA)) {
                std::cerr << "Warning: Failed to load " << g_tCheckerToAPath
                          << "! Disabling helmet mode." << std::endl;
                g_helmetMode = false;
            }
            else
            {
                std::cout << "[Helmet] Loaded T_checker_to_A from " << g_tCheckerToAPath << std::endl;
                std::cout << "[Helmet] Checkerboard: " << g_helmetCB.rows << "x" << g_helmetCB.cols
                          << " squares, " << g_helmetCB.squareMm << "mm" << std::endl;

                // Create transformation handles for each fixed camera
                int fixedIdx = 0;
                for (uint32_t i = 0; i < deviceCount; i++) {
                    if (static_cast<int>(i) == g_helmetDeviceIndex) continue;

                    FixedCameraInfo fci;
                    fci.calibration = devices[i].calibration;
                    fci.deviceIndex = static_cast<int>(i);
                    fci.colorWidth = devices[i].calibration.color_camera_calibration.resolution_width;
                    fci.colorHeight = devices[i].calibration.color_camera_calibration.resolution_height;

                    fci.transformation = k4a_transformation_create(&devices[i].calibration);
                    if (!fci.transformation) {
                        std::cerr << "[Helmet] Warning: Failed to create transformation for device " << i << std::endl;
                        continue;
                    }

                    g_deviceToFixedIndex[static_cast<int>(i)] = fixedIdx;
                    g_fixedCameraInfos.push_back(fci);
                    fixedIdx++;
                }

                g_fixedFrames.resize(g_fixedCameraInfos.size());

                std::cout << "[Helmet] " << g_fixedCameraInfos.size()
                          << " fixed cameras configured for checkerboard detection" << std::endl;

                // Start detector thread
                g_detectorRunning = true;
                g_detectorThread = std::thread(HelmetDetectorThread);
            }
        }
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
    std::thread helmetCaptureThread;
    for (uint32_t i = 0; i < deviceCount; i++)
    {
        if (g_helmetMode && static_cast<int>(i) == g_helmetDeviceIndex)
        {
            // Helmet device gets a color-only capture thread (no body tracking)
            helmetCaptureThread = std::thread(HelmetCameraCaptureThread, &devices[i]);
        }
        else
        {
            captureThreads.emplace_back(DeviceCaptureThread, &devices[i], i);
        }
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

                    // Diagnostic: log before render
                    {
                        static bool renderLogged = false;
                        if (!renderLogged) {
                            std::lock_guard<std::mutex> fLock(g_fusedBodyMutex);
                            if (!g_fusedBodies.empty()) {
                                std::cerr << "[DIAG-RENDER] About to render "
                                          << g_fusedBodies.size() << " fused bodies"
                                          << std::endl << std::flush;
                                renderLogged = true;
                            }
                        }
                    }

                    RenderFusedBodies(window3d);

                    // Diagnostic: log after render
                    {
                        static bool postRenderLogged = false;
                        if (!postRenderLogged) {
                            std::lock_guard<std::mutex> fLock(g_fusedBodyMutex);
                            if (!g_fusedBodies.empty()) {
                                std::cerr << "[DIAG-RENDER] RenderFusedBodies completed OK"
                                          << std::endl << std::flush;
                                postRenderLogged = true;
                            }
                        }
                    }
                } else {
                    RenderAllBodies(window3d);
                }

                window3d.SetLayout3d(s_layoutMode);
                window3d.SetJointFrameVisualization(s_visualizeJointFrame);

                // Diagnostic: log before window3d.Render()
                {
                    static bool glRenderLogged = false;
                    if (!glRenderLogged) {
                        std::lock_guard<std::mutex> fLock(g_fusedBodyMutex);
                        if (!g_fusedBodies.empty()) {
                            std::cerr << "[DIAG-RENDER] About to call window3d.Render()"
                                      << std::endl << std::flush;
                            glRenderLogged = true;
                        }
                    }
                }

                window3d.Render();

                // Diagnostic: log after window3d.Render()
                {
                    static bool postGlLogged = false;
                    if (!postGlLogged) {
                        std::lock_guard<std::mutex> fLock(g_fusedBodyMutex);
                        if (!g_fusedBodies.empty()) {
                            std::cerr << "[DIAG-RENDER] window3d.Render() completed OK"
                                      << std::endl << std::flush;
                            postGlLogged = true;
                        }
                    }
                }

                // Helmet overlay
                if (g_helmetMode) {
                    RenderHelmetOverlay();
                }

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

    // Shutdown helmet detector thread
    if (g_helmetMode)
    {
        g_detectorRunning = false;
        if (g_detectorThread.joinable()) {
            g_detectorThread.join();
        }

        // Destroy transformation handles
        for (auto& fci : g_fixedCameraInfos) {
            if (fci.transformation) {
                k4a_transformation_destroy(fci.transformation);
                fci.transformation = nullptr;
            }
        }

        cv::destroyAllWindows();
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
    if (helmetCaptureThread.joinable())
    {
        helmetCaptureThread.join();
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
