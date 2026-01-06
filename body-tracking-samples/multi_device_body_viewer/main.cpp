// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Multi-device body tracking viewer for Orbbec Femto Bolt cameras

#include <array>
#include <iostream>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <k4a/k4a.h>
#include <k4abt.h>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

// ============================================================================
// Forward Declarations
// ============================================================================
struct DeviceInfo;
void ExportCalibrationToYAML(const std::vector<DeviceInfo>& devices, const std::string& outputPath);
void ExportCalibrationToJSON(const std::vector<DeviceInfo>& devices, const std::string& outputPath);

// ============================================================================
// Global State
// ============================================================================
std::atomic<bool> s_isRunning{true};
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;
std::vector<DeviceInfo>* g_pDevices = nullptr;

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
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_C:
        // Save calibration files
        if (g_pDevices != nullptr && !g_pDevices->empty())
        {
            ExportCalibrationToYAML(*g_pDevices, "intri.yml");
            ExportCalibrationToJSON(*g_pDevices, "calibration.json");
            std::cout << "\nCalibration files saved!" << std::endl;
        }
        break;
    case GLFW_KEY_H:
        std::cout << "\n=== Key Shortcuts ===\n"
                  << "ESC: quit\n"
                  << "h: help\n"
                  << "b: body visualization mode\n"
                  << "k: 3d window layout\n"
                  << "c: save calibration (intri.yml, calibration.json)\n" << std::endl;
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

        // Render point cloud from first device with new data
        if (data.depthImage && data.bodyIndexMap)
        {
            std::vector<Color> pointCloudColors(data.depthWidth * data.depthHeight,
                                                 {0.4f, 0.4f, 0.4f, 0.3f});

            const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(data.bodyIndexMap);
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
              << "Options:\n"
              << "  NFOV_UNBINNED  - Narrow FOV Unbinned (default)\n"
              << "  WFOV_BINNED    - Wide FOV Binned\n"
              << "  CPU            - CPU processing mode\n"
              << "  CUDA           - CUDA processing mode\n"
              << "  DIRECTML       - DirectML processing mode (default on Windows)\n"
              << "  TENSORRT       - TensorRT processing mode\n"
              << std::endl;
}

// ============================================================================
// Export Calibration to EasyMocap Format (intri.yml)
// ============================================================================
void ExportCalibrationToYAML(const std::vector<DeviceInfo>& devices, const std::string& outputPath)
{
    std::ofstream file(outputPath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << outputPath << std::endl;
        return;
    }

    file << "%YAML:1.0\n";
    file << "---\n";

    // Write camera names
    file << "names:\n";
    for (size_t i = 0; i < devices.size(); i++)
    {
        file << "   - \"" << i << "\"\n";
    }

    // Write calibration for each camera
    for (size_t i = 0; i < devices.size(); i++)
    {
        const auto& calib = devices[i].calibration.depth_camera_calibration;
        const auto& params = calib.intrinsics.parameters.param;

        // Intrinsic matrix K (3x3)
        // | fx  0  cx |
        // | 0  fy  cy |
        // | 0   0   1 |
        file << "K_" << i << ": !!opencv-matrix\n";
        file << "   rows: 3\n";
        file << "   cols: 3\n";
        file << "   dt: d\n";
        file << std::fixed << std::setprecision(6);
        file << "   data: [ " << params.fx << ", 0., " << params.cx << ", "
             << "0., " << params.fy << ", " << params.cy << ", "
             << "0., 0., 1. ]\n";

        // Distortion coefficients (k1, k2, p1, p2, k3)
        // Note: K4A uses Brown-Conrady model with k1,k2,k3,k4,k5,k6,p1,p2
        // OpenCV typically uses k1,k2,p1,p2,k3 (5 params) or more
        file << "dist_" << i << ": !!opencv-matrix\n";
        file << "   rows: 1\n";
        file << "   cols: 5\n";
        file << "   dt: d\n";
        file << "   data: [ " << params.k1 << ", " << params.k2 << ", "
             << params.p1 << ", " << params.p2 << ", " << params.k3 << " ]\n";

        // Image dimensions
        file << "H_" << i << ": " << calib.resolution_height << "\n";
        file << "W_" << i << ": " << calib.resolution_width << "\n";
    }

    file.close();
    std::cout << "Intrinsic calibration saved to: " << outputPath << std::endl;
}

// ============================================================================
// Export Calibration to JSON Format (alternative)
// ============================================================================
void ExportCalibrationToJSON(const std::vector<DeviceInfo>& devices, const std::string& outputPath)
{
    std::ofstream file(outputPath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << outputPath << std::endl;
        return;
    }

    file << "{\n";
    file << "  \"cameras\": [\n";

    for (size_t i = 0; i < devices.size(); i++)
    {
        const auto& dev = devices[i];
        const auto& calib = dev.calibration.depth_camera_calibration;
        const auto& params = calib.intrinsics.parameters.param;

        file << "    {\n";
        file << "      \"id\": " << i << ",\n";
        file << "      \"serial_number\": \"" << dev.serialNumber << "\",\n";
        file << "      \"is_primary\": " << (dev.isPrimary ? "true" : "false") << ",\n";
        file << "      \"width\": " << calib.resolution_width << ",\n";
        file << "      \"height\": " << calib.resolution_height << ",\n";

        // Intrinsic matrix K
        file << std::fixed << std::setprecision(6);
        file << "      \"K\": [\n";
        file << "        [" << params.fx << ", 0.0, " << params.cx << "],\n";
        file << "        [0.0, " << params.fy << ", " << params.cy << "],\n";
        file << "        [0.0, 0.0, 1.0]\n";
        file << "      ],\n";

        // Distortion coefficients
        file << "      \"dist\": [" << params.k1 << ", " << params.k2 << ", "
             << params.p1 << ", " << params.p2 << ", " << params.k3 << ", "
             << params.k4 << ", " << params.k5 << ", " << params.k6 << "],\n";

        // Additional distortion parameters
        file << "      \"codx\": " << params.codx << ",\n";
        file << "      \"cody\": " << params.cody << ",\n";
        file << "      \"metric_radius\": " << params.metric_radius << "\n";

        file << "    }" << (i < devices.size() - 1 ? "," : "") << "\n";
    }

    file << "  ]\n";
    file << "}\n";

    file.close();
    std::cout << "Calibration saved to: " << outputPath << std::endl;
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

    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "NFOV_UNBINNED") depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        else if (arg == "WFOV_BINNED") depthMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        else if (arg == "CPU") processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
        else if (arg == "CUDA") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
        else if (arg == "DIRECTML") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
        else if (arg == "TENSORRT") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT;
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
        devices[i].isPrimary = (i == 0);  // First device is primary

        std::cout << "Device " << i << ": SN=" << devices[i].serialNumber
                  << " (" << (devices[i].isPrimary ? "PRIMARY" : "SECONDARY") << ")" << std::endl;
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

    // Set global pointer for key callback
    g_pDevices = &devices;

    // Initialize global body data storage
    g_deviceBodyData.resize(deviceCount);

    // Create 3D window (use first device's calibration)
    Window3dWrapper window3d;
    window3d.Create("Multi-Device Body Tracking", devices[0].calibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    // Start capture threads
    std::vector<std::thread> captureThreads;
    for (uint32_t i = 0; i < deviceCount; i++)
    {
        captureThreads.emplace_back(DeviceCaptureThread, &devices[i], i);
    }

    std::cout << "\nPress 'h' for help, 'c' to save calibration, ESC to quit\n" << std::endl;

    // Main render loop
    while (s_isRunning)
    {
        RenderAllBodies(window3d);

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    // Cleanup
    std::cout << "\nShutting down..." << std::endl;
    s_isRunning = false;

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
