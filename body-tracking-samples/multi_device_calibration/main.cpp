// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Multi-device extrinsic calibration tool using checkerboard
// Based on IEEE paper: "Accurate Extrinsic Calibration of Multiple Azure Kinect Using a Planar Checkerboard"

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>
#include <k4a/k4a.h>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// ============================================================================
// Configuration
// ============================================================================
const int CHECKERBOARD_ROWS = 6;      // Inner corners (rows)
const int CHECKERBOARD_COLS = 9;      // Inner corners (cols)
const float SQUARE_SIZE_MM = 25.0f;   // Checkerboard square size in mm

// ============================================================================
// Device Info Structure
// ============================================================================
struct DeviceInfo {
    k4a_device_t device = nullptr;
    k4a_transformation_t transformation = nullptr;
    std::string serialNumber;
    int index = 0;
    bool isPrimary = false;
    k4a_calibration_t calibration;
    int colorWidth = 0;
    int colorHeight = 0;
    int depthWidth = 0;
    int depthHeight = 0;
};

// ============================================================================
// Extrinsic calibration result
// ============================================================================
struct ExtrinsicCalibration {
    cv::Mat rotation;       // 3x3 rotation matrix
    cv::Mat translation;    // 3x1 translation vector (mm)
    std::string serialNumber;
    int deviceIndex;
    bool isValid = false;
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

    if (!serialNumber.empty() && serialNumber.back() == '\0') {
        serialNumber.pop_back();
    }
    return serialNumber;
}

// ============================================================================
// Convert K4A image to OpenCV Mat
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

// ============================================================================
// Detect checkerboard corners in image
// ============================================================================
bool DetectCheckerboardCorners(const cv::Mat& colorImage,
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
        // Refine corner positions
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    return found;
}

// ============================================================================
// Convert 2D color points to 3D using depth and K4A calibration
// Algorithm from IEEE paper: Use k4a_calibration_2d_to_3d
// ============================================================================
bool Convert2DTo3D(DeviceInfo& device,
                   k4a_image_t depthImage,
                   k4a_image_t colorImage,
                   const std::vector<cv::Point2f>& corners2D,
                   std::vector<cv::Point3f>& points3D)
{
    points3D.clear();

    // Create transformed depth image (depth aligned to color camera)
    k4a_image_t transformedDepth = nullptr;
    if (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                          device.colorWidth, device.colorHeight,
                          device.colorWidth * sizeof(uint16_t),
                          &transformedDepth) != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to create transformed depth image" << std::endl;
        return false;
    }

    // Transform depth to color camera space
    if (k4a_transformation_depth_image_to_color_camera(
            device.transformation, depthImage, transformedDepth) != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to transform depth to color camera" << std::endl;
        k4a_image_release(transformedDepth);
        return false;
    }

    uint16_t* depthBuffer = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(transformedDepth));
    int depthStride = k4a_image_get_stride_bytes(transformedDepth) / sizeof(uint16_t);

    for (const auto& corner : corners2D)
    {
        int x = static_cast<int>(std::round(corner.x));
        int y = static_cast<int>(std::round(corner.y));

        // Bounds check
        if (x < 0 || x >= device.colorWidth || y < 0 || y >= device.colorHeight)
        {
            std::cerr << "Corner out of bounds: (" << x << ", " << y << ")" << std::endl;
            k4a_image_release(transformedDepth);
            return false;
        }

        // Get depth value at corner location (sample 3x3 region for robustness)
        float depthSum = 0;
        int validCount = 0;
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < device.colorWidth && ny >= 0 && ny < device.colorHeight)
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

        if (validCount == 0)
        {
            std::cerr << "No valid depth at corner (" << x << ", " << y << ")" << std::endl;
            k4a_image_release(transformedDepth);
            return false;
        }

        float depthMm = depthSum / validCount;

        // Convert 2D color + depth to 3D
        k4a_float2_t sourcePoint2D = { corner.x, corner.y };
        k4a_float3_t targetPoint3D;
        int valid = 0;

        if (k4a_calibration_2d_to_3d(&device.calibration,
                                      &sourcePoint2D,
                                      depthMm,
                                      K4A_CALIBRATION_TYPE_COLOR,
                                      K4A_CALIBRATION_TYPE_COLOR,
                                      &targetPoint3D,
                                      &valid) != K4A_RESULT_SUCCEEDED || !valid)
        {
            std::cerr << "Failed to convert 2D to 3D at (" << corner.x << ", " << corner.y << ")" << std::endl;
            k4a_image_release(transformedDepth);
            return false;
        }

        points3D.push_back(cv::Point3f(targetPoint3D.xyz.x, targetPoint3D.xyz.y, targetPoint3D.xyz.z));
    }

    k4a_image_release(transformedDepth);
    return true;
}

// ============================================================================
// Compute SVD-based extrinsic calibration
// Algorithm from IEEE paper:
// 1. Compute centroid of 3D points
// 2. Center the points (X = points - centroid)
// 3. Compute correlation matrix C = X^T * X
// 4. SVD(C) = U * S * V^T
// 5. R = U^(-1) = U^T (orthogonal matrix)
// 6. t = -centroid
// ============================================================================
ExtrinsicCalibration ComputeExtrinsicSVD(const std::vector<cv::Point3f>& points3D,
                                          const std::string& serialNumber,
                                          int deviceIndex)
{
    ExtrinsicCalibration result;
    result.serialNumber = serialNumber;
    result.deviceIndex = deviceIndex;

    if (points3D.size() < 4)
    {
        std::cerr << "Not enough 3D points for calibration" << std::endl;
        return result;
    }

    // Step 1: Compute centroid
    cv::Point3f centroid(0, 0, 0);
    for (const auto& p : points3D)
    {
        centroid.x += p.x;
        centroid.y += p.y;
        centroid.z += p.z;
    }
    centroid.x /= points3D.size();
    centroid.y /= points3D.size();
    centroid.z /= points3D.size();

    std::cout << "  Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ") mm" << std::endl;

    // Step 2: Center the points and create matrix X (N x 3)
    cv::Mat X(static_cast<int>(points3D.size()), 3, CV_64F);
    for (size_t i = 0; i < points3D.size(); i++)
    {
        X.at<double>(i, 0) = points3D[i].x - centroid.x;
        X.at<double>(i, 1) = points3D[i].y - centroid.y;
        X.at<double>(i, 2) = points3D[i].z - centroid.z;
    }

    // Step 3: Compute correlation matrix C = X^T * X (3x3)
    cv::Mat C = X.t() * X;

    // Step 4: SVD of C
    cv::Mat U, S, Vt;
    cv::SVD::compute(C, S, U, Vt);

    // Step 5: R = U^T (since U is orthogonal, U^(-1) = U^T)
    // The eigenvectors in U define the principal axes of the point cloud
    result.rotation = U.t();

    // Ensure proper rotation (det(R) = +1)
    double det = cv::determinant(result.rotation);
    if (det < 0)
    {
        // Flip sign of last column to ensure proper rotation
        result.rotation.col(2) *= -1.0;
    }

    // Step 6: Translation is negative centroid
    result.translation = cv::Mat(3, 1, CV_64F);
    result.translation.at<double>(0, 0) = -centroid.x;
    result.translation.at<double>(1, 0) = -centroid.y;
    result.translation.at<double>(2, 0) = -centroid.z;

    result.isValid = true;
    return result;
}

// ============================================================================
// Compute relative transformation (secondary to primary)
// T_s_to_p = T_p^(-1) * T_s
// ============================================================================
void ComputeRelativeTransform(const ExtrinsicCalibration& primary,
                               ExtrinsicCalibration& secondary)
{
    if (!primary.isValid || !secondary.isValid)
    {
        std::cerr << "Cannot compute relative transform: invalid calibration" << std::endl;
        return;
    }

    // T_p^(-1): inverse of primary transform
    cv::Mat R_p_inv = primary.rotation.t();  // R^(-1) = R^T for rotation matrices
    cv::Mat t_p_inv = -R_p_inv * primary.translation;

    // T_rel = T_p^(-1) * T_s
    // R_rel = R_p^(-1) * R_s
    // t_rel = R_p^(-1) * t_s + t_p^(-1)
    secondary.rotation = R_p_inv * secondary.rotation;
    secondary.translation = R_p_inv * secondary.translation + t_p_inv;
}

// ============================================================================
// Save calibration to YAML file (OpenCV format)
// ============================================================================
void SaveCalibrationYAML(const std::vector<ExtrinsicCalibration>& calibrations,
                          const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        return;
    }

    fs << "num_devices" << static_cast<int>(calibrations.size());
    fs << "calibrations" << "[";

    for (const auto& calib : calibrations)
    {
        fs << "{";
        fs << "device_index" << calib.deviceIndex;
        fs << "serial_number" << calib.serialNumber;
        fs << "is_valid" << calib.isValid;
        if (calib.isValid)
        {
            fs << "rotation" << calib.rotation;
            fs << "translation" << calib.translation;
        }
        fs << "}";
    }

    fs << "]";
    fs.release();

    std::cout << "Saved calibration to: " << filename << std::endl;
}

// ============================================================================
// Save calibration to JSON file
// ============================================================================
void SaveCalibrationJSON(const std::vector<ExtrinsicCalibration>& calibrations,
                          const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6);
    file << "{\n";
    file << "  \"num_devices\": " << calibrations.size() << ",\n";
    file << "  \"calibrations\": [\n";

    for (size_t i = 0; i < calibrations.size(); i++)
    {
        const auto& calib = calibrations[i];
        file << "    {\n";
        file << "      \"device_index\": " << calib.deviceIndex << ",\n";
        file << "      \"serial_number\": \"" << calib.serialNumber << "\",\n";
        file << "      \"is_valid\": " << (calib.isValid ? "true" : "false");

        if (calib.isValid)
        {
            file << ",\n";
            file << "      \"rotation\": [\n";
            for (int r = 0; r < 3; r++)
            {
                file << "        [";
                for (int c = 0; c < 3; c++)
                {
                    file << calib.rotation.at<double>(r, c);
                    if (c < 2) file << ", ";
                }
                file << "]";
                if (r < 2) file << ",";
                file << "\n";
            }
            file << "      ],\n";
            file << "      \"translation\": ["
                 << calib.translation.at<double>(0, 0) << ", "
                 << calib.translation.at<double>(1, 0) << ", "
                 << calib.translation.at<double>(2, 0) << "]\n";
        }
        else
        {
            file << "\n";
        }

        file << "    }";
        if (i < calibrations.size() - 1) file << ",";
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";
    file.close();

    std::cout << "Saved calibration to: " << filename << std::endl;
}

// ============================================================================
// Print Usage
// ============================================================================
void PrintUsage()
{
    std::cout << "\n=== Multi-Device Extrinsic Calibration Tool ===\n"
              << "USAGE: multi_device_calibration.exe [OPTIONS]\n\n"
              << "Options:\n"
              << "  --rows N       Checkerboard inner corners (rows), default: " << CHECKERBOARD_ROWS << "\n"
              << "  --cols N       Checkerboard inner corners (cols), default: " << CHECKERBOARD_COLS << "\n"
              << "  --square N     Square size in mm, default: " << SQUARE_SIZE_MM << "\n"
              << "  --output FILE  Output filename prefix, default: calibration\n"
              << "\nInstructions:\n"
              << "  1. Place checkerboard visible to ALL cameras\n"
              << "  2. Press SPACE to capture and calibrate\n"
              << "  3. Press 'S' to save calibration\n"
              << "  4. Press ESC to quit\n"
              << std::endl;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "Multi-Device Extrinsic Calibration Tool" << std::endl;
    std::cout << "For Orbbec Femto Bolt with K4A Wrapper" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Parse arguments
    int checkerboardRows = CHECKERBOARD_ROWS;
    int checkerboardCols = CHECKERBOARD_COLS;
    float squareSize = SQUARE_SIZE_MM;
    std::string outputPrefix = "calibration";

    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "--rows" && i + 1 < argc) checkerboardRows = std::atoi(argv[++i]);
        else if (arg == "--cols" && i + 1 < argc) checkerboardCols = std::atoi(argv[++i]);
        else if (arg == "--square" && i + 1 < argc) squareSize = std::atof(argv[++i]);
        else if (arg == "--output" && i + 1 < argc) outputPrefix = argv[++i];
        else if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return 0;
        }
    }

    cv::Size patternSize(checkerboardCols, checkerboardRows);
    std::cout << "Checkerboard: " << checkerboardCols << "x" << checkerboardRows
              << " inner corners, " << squareSize << "mm squares" << std::endl;

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
        std::cout << "Warning: Only 1 device found. Multi-device calibration requires 2+ cameras." << std::endl;
    }

    // Open all devices
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
        devices[i].isPrimary = (i == 0);

        std::cout << "Device " << i << ": SN=" << devices[i].serialNumber
                  << " (" << (devices[i].isPrimary ? "PRIMARY" : "SECONDARY") << ")" << std::endl;
    }

    // Configure and start cameras
    // Need COLOR for checkerboard detection, DEPTH for 3D conversion
    std::cout << "\nStarting cameras..." << std::endl;

    for (int i = deviceCount - 1; i >= 0; i--)
    {
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.synchronized_images_only = true;

        // Set sync mode for multi-device
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
                config.subordinate_delay_off_master_usec = 160 * devices[i].index;
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

        devices[i].colorWidth = devices[i].calibration.color_camera_calibration.resolution_width;
        devices[i].colorHeight = devices[i].calibration.color_camera_calibration.resolution_height;
        devices[i].depthWidth = devices[i].calibration.depth_camera_calibration.resolution_width;
        devices[i].depthHeight = devices[i].calibration.depth_camera_calibration.resolution_height;

        // Create transformation handle
        devices[i].transformation = k4a_transformation_create(&devices[i].calibration);
        if (!devices[i].transformation)
        {
            std::cerr << "Failed to create transformation for device " << i << std::endl;
            return -1;
        }

        if (i > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    std::cout << "\nAll devices started successfully!" << std::endl;
    std::cout << "\nInstructions:" << std::endl;
    std::cout << "  - Hold checkerboard visible to ALL cameras" << std::endl;
    std::cout << "  - Press SPACE to capture and calibrate" << std::endl;
    std::cout << "  - Press 'S' to save calibration to file" << std::endl;
    std::cout << "  - Press ESC to quit" << std::endl;

    // Create display windows
    for (uint32_t i = 0; i < deviceCount; i++)
    {
        std::string windowName = "Device " + std::to_string(i) + " (" +
                                  (devices[i].isPrimary ? "PRIMARY" : "SECONDARY") + ")";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(windowName, 640, 360);
    }

    std::vector<ExtrinsicCalibration> calibrations(deviceCount);
    bool calibrationDone = false;
    bool running = true;

    while (running)
    {
        std::vector<cv::Mat> colorImages(deviceCount);
        std::vector<k4a_image_t> depthImages(deviceCount, nullptr);
        std::vector<std::vector<cv::Point2f>> allCorners(deviceCount);
        std::vector<bool> cornersFound(deviceCount, false);

        // Capture from all devices
        for (uint32_t i = 0; i < deviceCount; i++)
        {
            k4a_capture_t capture = nullptr;
            k4a_wait_result_t result = k4a_device_get_capture(devices[i].device, &capture, 1000);

            if (result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                k4a_image_t colorImage = k4a_capture_get_color_image(capture);
                k4a_image_t depthImage = k4a_capture_get_depth_image(capture);

                if (colorImage && depthImage)
                {
                    colorImages[i] = K4AImageToMat(colorImage);
                    depthImages[i] = depthImage;
                    k4a_image_reference(depthImage);  // Keep reference

                    // Detect checkerboard
                    cornersFound[i] = DetectCheckerboardCorners(colorImages[i], allCorners[i], patternSize);
                }

                if (colorImage) k4a_image_release(colorImage);
                if (depthImage) k4a_image_release(depthImage);
                k4a_capture_release(capture);
            }
        }

        // Display images with corner detection
        for (uint32_t i = 0; i < deviceCount; i++)
        {
            if (!colorImages[i].empty())
            {
                cv::Mat display;
                cv::cvtColor(colorImages[i], display, cv::COLOR_BGRA2BGR);

                if (cornersFound[i])
                {
                    cv::drawChessboardCorners(display, patternSize, allCorners[i], true);
                    cv::putText(display, "Checkerboard FOUND", cv::Point(10, 30),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                }
                else
                {
                    cv::putText(display, "Checkerboard NOT found", cv::Point(10, 30),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                }

                if (calibrationDone && calibrations[i].isValid)
                {
                    cv::putText(display, "CALIBRATED", cv::Point(10, 60),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                }

                std::string windowName = "Device " + std::to_string(i) + " (" +
                                          (devices[i].isPrimary ? "PRIMARY" : "SECONDARY") + ")";
                cv::imshow(windowName, display);
            }
        }

        // Handle key input
        int key = cv::waitKey(30);

        if (key == 27)  // ESC
        {
            running = false;
        }
        else if (key == ' ')  // SPACE - Calibrate
        {
            // Check if all devices found checkerboard
            bool allFound = true;
            for (uint32_t i = 0; i < deviceCount; i++)
            {
                if (!cornersFound[i])
                {
                    allFound = false;
                    std::cout << "Device " << i << ": Checkerboard NOT found" << std::endl;
                }
            }

            if (allFound)
            {
                std::cout << "\n=== Computing Calibration ===" << std::endl;

                // Convert 2D to 3D for each device
                std::vector<std::vector<cv::Point3f>> allPoints3D(deviceCount);
                bool allConverted = true;

                for (uint32_t i = 0; i < deviceCount; i++)
                {
                    std::cout << "Device " << i << " (" << devices[i].serialNumber << "):" << std::endl;

                    if (!Convert2DTo3D(devices[i], depthImages[i], nullptr,
                                       allCorners[i], allPoints3D[i]))
                    {
                        std::cerr << "  Failed to convert 2D to 3D" << std::endl;
                        allConverted = false;
                        break;
                    }

                    std::cout << "  Converted " << allPoints3D[i].size() << " points to 3D" << std::endl;

                    // Compute extrinsic calibration using SVD
                    calibrations[i] = ComputeExtrinsicSVD(allPoints3D[i],
                                                          devices[i].serialNumber,
                                                          devices[i].index);
                }

                if (allConverted)
                {
                    // Compute relative transforms (secondary cameras relative to primary)
                    if (calibrations[0].isValid)
                    {
                        for (uint32_t i = 1; i < deviceCount; i++)
                        {
                            if (calibrations[i].isValid)
                            {
                                ComputeRelativeTransform(calibrations[0], calibrations[i]);
                                std::cout << "\nDevice " << i << " -> Device 0 transformation:" << std::endl;
                                std::cout << "  Rotation:\n" << calibrations[i].rotation << std::endl;
                                std::cout << "  Translation: " << calibrations[i].translation.t() << " mm" << std::endl;
                            }
                        }

                        // Primary camera is identity in its own coordinate system
                        calibrations[0].rotation = cv::Mat::eye(3, 3, CV_64F);
                        calibrations[0].translation = cv::Mat::zeros(3, 1, CV_64F);
                    }

                    calibrationDone = true;
                    std::cout << "\n=== Calibration Complete ===" << std::endl;
                    std::cout << "Press 'S' to save, ESC to quit" << std::endl;
                }
            }
            else
            {
                std::cout << "Cannot calibrate: checkerboard not visible in all cameras" << std::endl;
            }
        }
        else if (key == 's' || key == 'S')  // Save
        {
            if (calibrationDone)
            {
                SaveCalibrationYAML(calibrations, outputPrefix + ".yml");
                SaveCalibrationJSON(calibrations, outputPrefix + ".json");
            }
            else
            {
                std::cout << "No calibration to save. Press SPACE to calibrate first." << std::endl;
            }
        }

        // Release depth images
        for (auto& img : depthImages)
        {
            if (img) k4a_image_release(img);
        }
    }

    // Cleanup
    std::cout << "\nShutting down..." << std::endl;
    cv::destroyAllWindows();

    for (auto& device : devices)
    {
        if (device.transformation)
        {
            k4a_transformation_destroy(device.transformation);
        }
        if (device.device)
        {
            k4a_device_stop_cameras(device.device);
            k4a_device_close(device.device);
        }
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
