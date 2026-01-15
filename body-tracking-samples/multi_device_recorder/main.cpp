// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Multi-device MKV recorder for Orbbec Femto Bolt cameras
// Records raw depth/IR/color without body tracking for offline processing

#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <k4a/k4a.h>
#include <k4arecord/record.h>

// Winsock for UDP communication
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <conio.h>
#pragma comment(lib, "ws2_32.lib")
#endif

// ============================================================================
// Global State
// ============================================================================
std::atomic<bool> s_isRunning{true};
std::atomic<bool> g_isRecording{false};
std::string g_outputDir = ".";
std::string g_sessionName = "";

// Recording handles (one per device)
std::vector<k4a_record_t> g_recordings;
std::mutex g_recordMutex;

// Device info
struct DeviceInfo {
    k4a_device_t device = nullptr;
    std::string serialNumber;
    int index = 0;
    bool isPrimary = false;
    k4a_calibration_t calibration;
};
std::vector<DeviceInfo> g_devices;
std::string g_primarySerial = "";

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

    DWORD timeout = 100;
    setsockopt(g_udpSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

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

    std::cout << "[UDP] Sent: " << command << std::endl;
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
}

std::vector<std::string> GetPendingCommands()
{
    std::lock_guard<std::mutex> lock(g_commandMutex);
    std::vector<std::string> commands = std::move(g_pendingCommands);
    g_pendingCommands.clear();
    return commands;
}
#endif // _WIN32

// ============================================================================
// Utility Functions
// ============================================================================
std::string GetDeviceSerialNumber(k4a_device_t device)
{
    size_t serialNumberSize = 0;
    k4a_device_get_serialnum(device, nullptr, &serialNumberSize);

    std::string serialNumber(serialNumberSize, '\0');
    k4a_device_get_serialnum(device, &serialNumber[0], &serialNumberSize);

    if (!serialNumber.empty() && serialNumber.back() == '\0') {
        serialNumber.pop_back();
    }
    return serialNumber;
}

std::string GenerateTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_s(&tm_now, &time_t_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ============================================================================
// Recording Functions
// ============================================================================
void StartRecording()
{
    std::lock_guard<std::mutex> lock(g_recordMutex);

    if (g_isRecording) {
        std::cout << "Already recording!" << std::endl;
        return;
    }

    std::string timestamp = GenerateTimestamp();
    std::string session = g_sessionName.empty() ? timestamp : g_sessionName + "_" + timestamp;

    g_recordings.resize(g_devices.size());

    for (size_t i = 0; i < g_devices.size(); i++)
    {
        std::ostringstream filename;
        filename << g_outputDir << "/recording_cam" << i
                 << "_" << g_devices[i].serialNumber
                 << "_" << session << ".mkv";

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;

        if (g_devices.size() > 1) {
            config.wired_sync_mode = g_devices[i].isPrimary
                ? K4A_WIRED_SYNC_MODE_MASTER
                : K4A_WIRED_SYNC_MODE_SUBORDINATE;
        }

        k4a_result_t result = k4a_record_create(
            filename.str().c_str(),
            g_devices[i].device,
            config,
            &g_recordings[i]);

        if (result != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to create recording for device " << i << std::endl;
            // Cleanup already created recordings
            for (size_t j = 0; j < i; j++) {
                if (g_recordings[j]) {
                    k4a_record_close(g_recordings[j]);
                    g_recordings[j] = nullptr;
                }
            }
            return;
        }

        // Write header
        result = k4a_record_write_header(g_recordings[i]);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to write header for device " << i << std::endl;
        }

        std::cout << "Recording started: " << filename.str() << std::endl;
    }

    g_isRecording = true;
    std::cout << "\n=== RECORDING STARTED ===" << std::endl;
}

void StopRecording()
{
    std::lock_guard<std::mutex> lock(g_recordMutex);

    if (!g_isRecording) {
        std::cout << "Not recording!" << std::endl;
        return;
    }

    for (size_t i = 0; i < g_recordings.size(); i++)
    {
        if (g_recordings[i])
        {
            k4a_record_flush(g_recordings[i]);
            k4a_record_close(g_recordings[i]);
            g_recordings[i] = nullptr;
            std::cout << "Recording stopped for device " << i << std::endl;
        }
    }

    g_recordings.clear();
    g_isRecording = false;
    std::cout << "\n=== RECORDING STOPPED ===" << std::endl;
}

void WriteCapture(int deviceIndex, k4a_capture_t capture)
{
    std::lock_guard<std::mutex> lock(g_recordMutex);

    if (!g_isRecording || deviceIndex >= (int)g_recordings.size()) return;
    if (!g_recordings[deviceIndex]) return;

    k4a_result_t result = k4a_record_write_capture(g_recordings[deviceIndex], capture);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to write capture for device " << deviceIndex << std::endl;
    }
}

// ============================================================================
// Process UDP Commands
// ============================================================================
void ProcessUdpCommands()
{
#ifdef _WIN32
    auto commands = GetPendingCommands();
    for (const auto& cmd : commands)
    {
        if (cmd == "TOGGLE_RECORD")
        {
            if (g_isRecording) {
                StopRecording();
                SendUdpCommand("STOP_RECORD");
            } else {
                StartRecording();
                SendUdpCommand("START_RECORD");
            }
        }
        else if (cmd == "START_RECORD")
        {
            if (!g_isRecording) {
                StartRecording();
            }
        }
        else if (cmd == "STOP_RECORD")
        {
            if (g_isRecording) {
                StopRecording();
            }
        }
    }
#endif
}

// ============================================================================
// Capture Thread
// ============================================================================
void CaptureThread(int deviceIndex)
{
    DeviceInfo& info = g_devices[deviceIndex];
    std::cout << "[Device " << deviceIndex << "] Capture thread started" << std::endl;

    uint64_t frameCount = 0;

    while (s_isRunning)
    {
        k4a_capture_t capture = nullptr;
        k4a_wait_result_t result = k4a_device_get_capture(info.device, &capture, 1000);

        if (result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // Write to MKV if recording
            if (g_isRecording)
            {
                WriteCapture(deviceIndex, capture);
            }

            k4a_capture_release(capture);
            frameCount++;

            // Status update every 100 frames
            if (frameCount % 100 == 0)
            {
                std::cout << "[Device " << deviceIndex << "] Frame " << frameCount;
                if (g_isRecording) std::cout << " [REC]";
                std::cout << std::endl;
            }
        }
        else if (result == K4A_WAIT_RESULT_FAILED)
        {
            std::cerr << "[Device " << deviceIndex << "] Capture failed!" << std::endl;
            break;
        }
    }

    std::cout << "[Device " << deviceIndex << "] Capture thread stopped" << std::endl;
}

// ============================================================================
// Print Usage
// ============================================================================
void PrintUsage()
{
    std::cout << "\n=== Multi-Device MKV Recorder ===\n"
              << "Records raw depth data from multiple Orbbec cameras for offline processing.\n\n"
              << "USAGE: multi_device_recorder.exe [OPTIONS]\n\n"
              << "Options:\n"
              << "  --output DIR         - Output directory for MKV files (default: current)\n"
              << "  --session NAME       - Session name prefix for recordings\n"
              << "  --primary SERIAL     - Serial number of PRIMARY camera\n"
              << "  --udp-port PORT      - UDP listen port (default: 9000)\n"
              << "  --no-udp             - Disable UDP listener\n\n"
              << "Runtime Controls:\n"
              << "  R         - Start/stop recording\n"
              << "  Q or ESC  - Quit\n\n"
              << "UDP Commands (from Unity):\n"
              << "  TOGGLE_RECORD  - Toggle recording on/off\n"
              << "  START_RECORD   - Start recording\n"
              << "  STOP_RECORD    - Stop recording\n"
              << std::endl;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "Multi-Device MKV Recorder" << std::endl;
    std::cout << "For Orbbec Femto Bolt with K4A Wrapper" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Parse arguments
    bool enableUdp = true;
    int udpPort = 9000;

    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "--output" && i + 1 < argc) {
            g_outputDir = argv[++i];
        }
        else if (arg == "--session" && i + 1 < argc) {
            g_sessionName = argv[++i];
        }
        else if (arg == "--primary" && i + 1 < argc) {
            g_primarySerial = argv[++i];
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

    // Open all devices
    g_devices.resize(deviceCount);

    for (uint32_t i = 0; i < deviceCount; i++)
    {
        g_devices[i].index = i;

        if (k4a_device_open(i, &g_devices[i].device) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to open device " << i << std::endl;
            return -1;
        }

        g_devices[i].serialNumber = GetDeviceSerialNumber(g_devices[i].device);
        std::cout << "Device " << i << ": SN=" << g_devices[i].serialNumber << std::endl;
    }

    // Determine primary camera
    if (!g_primarySerial.empty()) {
        bool found = false;
        for (uint32_t i = 0; i < deviceCount; i++) {
            if (g_devices[i].serialNumber == g_primarySerial) {
                g_devices[i].isPrimary = true;
                found = true;
            } else {
                g_devices[i].isPrimary = false;
            }
        }
        if (!found) {
            std::cerr << "Warning: Primary serial not found, using device 0" << std::endl;
            g_devices[0].isPrimary = true;
        }
    } else {
        g_devices[0].isPrimary = true;
    }

    // Print device roles
    for (uint32_t i = 0; i < deviceCount; i++) {
        std::cout << "Device " << i << ": "
                  << (g_devices[i].isPrimary ? "PRIMARY" : "SECONDARY") << std::endl;
    }

    // Configure and start cameras (secondary first)
    std::cout << "\nConfiguring devices..." << std::endl;

    for (int i = (int)deviceCount - 1; i >= 0; i--)
    {
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;

        if (deviceCount > 1) {
            if (g_devices[i].isPrimary) {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
            } else {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
                config.subordinate_delay_off_master_usec = 160 * g_devices[i].index;
            }
        } else {
            config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        }

        std::cout << "Starting device " << i << " ("
                  << (g_devices[i].isPrimary ? "MASTER" : "SUBORDINATE") << ")..." << std::endl;

        if (k4a_device_start_cameras(g_devices[i].device, &config) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to start cameras on device " << i << std::endl;
            return -1;
        }

        // Get calibration
        k4a_device_get_calibration(g_devices[i].device, config.depth_mode,
                                    config.color_resolution, &g_devices[i].calibration);

        if (i > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    std::cout << "\nAll devices started!" << std::endl;

    // Initialize UDP listener
#ifdef _WIN32
    if (enableUdp) {
        if (InitUdpListener(udpPort)) {
            std::cout << "UDP sync enabled on port " << udpPort << std::endl;
        } else {
            std::cout << "Warning: UDP initialization failed" << std::endl;
        }
    }
#endif

    // Start capture threads
    std::vector<std::thread> captureThreads;
    for (uint32_t i = 0; i < deviceCount; i++)
    {
        captureThreads.emplace_back(CaptureThread, i);
    }

    std::cout << "\nPress 'R' to start/stop recording, 'Q' to quit\n" << std::endl;

    // Main loop (keyboard input)
    while (s_isRunning)
    {
        // Process UDP commands
        ProcessUdpCommands();

        // Check for keyboard input (Windows console)
#ifdef _WIN32
        if (_kbhit())
        {
            int key = _getch();
            if (key == 'r' || key == 'R')
            {
                if (g_isRecording) {
                    StopRecording();
                    SendUdpCommand("STOP_RECORD");
                } else {
                    StartRecording();
                    SendUdpCommand("START_RECORD");
                }
            }
            else if (key == 'q' || key == 'Q' || key == 27) // Q or ESC
            {
                s_isRunning = false;
            }
        }
#endif

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Cleanup
    std::cout << "\nShutting down..." << std::endl;

    if (g_isRecording) {
        StopRecording();
    }

#ifdef _WIN32
    ShutdownUdpListener();
#endif

    // Wait for capture threads
    for (auto& thread : captureThreads)
    {
        if (thread.joinable()) thread.join();
    }

    // Stop and close devices
    for (auto& device : g_devices)
    {
        if (device.device)
        {
            k4a_device_stop_cameras(device.device);
            k4a_device_close(device.device);
        }
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
