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
#include <mutex>
#include <atomic>
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

// HMD Calibration defaults (two-checkerboard method)
const int HEAD_CB_ROWS = 4;           // Head-checkerboard inner corners (rows)
const int HEAD_CB_COLS = 5;           // Head-checkerboard inner corners (cols)
const float HEAD_CB_SQUARE_MM = 30.0f; // Head-checkerboard square size in mm

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
// HMD Calibration Result (T_checker_to_A)
// ============================================================================
struct HMDCalibration {
    cv::Mat rotation;       // 3x3 rotation matrix (checkerboard to camera)
    cv::Mat translation;    // 3x1 translation vector (mm)
    cv::Mat rvec;           // Rodrigues rotation vector
    bool isValid = false;
    int numCaptures = 0;
    int numUsed = 0;        // Captures used after outlier removal
    double translationStdDev = 0.0;  // ||t|| std dev across captures (mm)
    double rotationStdDev = 0.0;     // Rotation angle std dev across captures (degrees)
    double maxTranslationError = 0.0; // Max ||t|| deviation from mean (mm)
    double maxRotationError = 0.0;    // Max rotation angle deviation from mean (degrees)
    std::string method = "solvePnP";  // "solvePnP" or "horn_3d_depth"
};

// ============================================================================
// Thread-safe capture data for each device
// ============================================================================
struct CaptureData {
    cv::Mat colorImage;
    k4a_image_t depthImage = nullptr;
    std::vector<cv::Point2f> corners;
    bool cornersFound = false;
    bool hasNewData = false;
    std::mutex mutex;
};

std::atomic<bool> g_captureRunning{true};

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
// Fix corner orientation between cameras
// When checkerboard appears rotated 180 degrees, corners are detected in reverse order.
// This function ensures all cameras have consistent corner ordering relative to primary.
// Reference: green_screen example from OrbbecSDK-K4A-Wrapper
// ============================================================================
void FixCornerOrientation(const std::vector<cv::Point2f>& primaryCorners,
                          std::vector<cv::Point2f>& secondaryCorners)
{
    if (primaryCorners.size() < 2 || secondaryCorners.size() < 2)
    {
        return;
    }

    // Vector from first to last corner in primary camera
    cv::Vec2f primaryVec(
        primaryCorners.back().x - primaryCorners.front().x,
        primaryCorners.back().y - primaryCorners.front().y
    );

    // Vector from first to last corner in secondary camera
    cv::Vec2f secondaryVec(
        secondaryCorners.back().x - secondaryCorners.front().x,
        secondaryCorners.back().y - secondaryCorners.front().y
    );

    // If dot product is negative, corners are in opposite order
    float dotProduct = primaryVec[0] * secondaryVec[0] + primaryVec[1] * secondaryVec[1];

    if (dotProduct <= 0.0f)
    {
        std::cout << "  Corner orientation mismatch detected, reversing order..." << std::endl;
        std::reverse(secondaryCorners.begin(), secondaryCorners.end());
    }
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

        // Get depth value at corner using expanding ring search.
        // Transformed depth (640x576 → 1920x1080) has large holes; search outward
        // from the corner pixel until valid depth is found.
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
                    // Only check the outermost ring (interior already checked)
                    if (r > 0 && std::abs(dx) < r && std::abs(dy) < r) continue;

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
            if (validCount > 0)
            {
                depthMm = depthSum / validCount;
                break;
            }
        }

        if (depthMm == 0)
        {
            std::cerr << "No valid depth within " << maxSearchRadius
                      << "px of corner (" << x << ", " << y << ")" << std::endl;
            k4a_image_release(transformedDepth);
            return false;
        }

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
// Compute 3D-to-3D pose using Horn's method (closed-form point registration)
// Input: objectPoints (known checkerboard geometry), cameraPoints (measured via depth)
// Output: 4x4 T_object_to_camera (same convention as solvePnP)
// Algorithm: centroids → center → cross-covariance H = P^T * Q → SVD(H) → R, t
// ============================================================================
bool ComputePose3DTo3D(const std::vector<cv::Point3f>& objectPoints,
                       const std::vector<cv::Point3f>& cameraPoints,
                       cv::Mat& T_out,
                       double& rmsResidual)
{
    if (objectPoints.size() != cameraPoints.size() || objectPoints.size() < 3)
    {
        std::cerr << "ComputePose3DTo3D: need >= 3 matching point pairs" << std::endl;
        return false;
    }

    int N = static_cast<int>(objectPoints.size());

    // Step 1: Compute centroids
    cv::Point3d centroidObj(0, 0, 0), centroidCam(0, 0, 0);
    for (int i = 0; i < N; i++)
    {
        centroidObj.x += objectPoints[i].x;
        centroidObj.y += objectPoints[i].y;
        centroidObj.z += objectPoints[i].z;
        centroidCam.x += cameraPoints[i].x;
        centroidCam.y += cameraPoints[i].y;
        centroidCam.z += cameraPoints[i].z;
    }
    centroidObj *= (1.0 / N);
    centroidCam *= (1.0 / N);

    // Step 2: Center points
    cv::Mat P(N, 3, CV_64F);  // centered object points
    cv::Mat Q(N, 3, CV_64F);  // centered camera points
    for (int i = 0; i < N; i++)
    {
        P.at<double>(i, 0) = objectPoints[i].x - centroidObj.x;
        P.at<double>(i, 1) = objectPoints[i].y - centroidObj.y;
        P.at<double>(i, 2) = objectPoints[i].z - centroidObj.z;
        Q.at<double>(i, 0) = cameraPoints[i].x - centroidCam.x;
        Q.at<double>(i, 1) = cameraPoints[i].y - centroidCam.y;
        Q.at<double>(i, 2) = cameraPoints[i].z - centroidCam.z;
    }

    // Step 3: Cross-covariance matrix H = P^T * Q (3x3)
    cv::Mat H = P.t() * Q;

    // Step 4: SVD(H) = U * S * V^T
    cv::Mat U, S, Vt;
    cv::SVD::compute(H, S, U, Vt);

    // Step 5: R = V * U^T
    cv::Mat R = Vt.t() * U.t();

    // Ensure proper rotation (det = +1, not reflection)
    if (cv::determinant(R) < 0)
    {
        // Flip sign of last column of Vt (i.e., last row before transpose)
        Vt.row(2) *= -1.0;
        R = Vt.t() * U.t();
    }

    // Step 6: t = centroid_cam - R * centroid_obj
    cv::Mat centObjMat = (cv::Mat_<double>(3, 1) << centroidObj.x, centroidObj.y, centroidObj.z);
    cv::Mat centCamMat = (cv::Mat_<double>(3, 1) << centroidCam.x, centroidCam.y, centroidCam.z);
    cv::Mat t = centCamMat - R * centObjMat;

    // Build 4x4 homogeneous transform
    T_out = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T_out(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T_out(cv::Rect(3, 0, 1, 3)));

    // Compute RMS residual
    double sumSqErr = 0.0;
    for (int i = 0; i < N; i++)
    {
        cv::Mat objPt = (cv::Mat_<double>(3, 1) <<
            objectPoints[i].x, objectPoints[i].y, objectPoints[i].z);
        cv::Mat predicted = R * objPt + t;
        double dx = predicted.at<double>(0) - cameraPoints[i].x;
        double dy = predicted.at<double>(1) - cameraPoints[i].y;
        double dz = predicted.at<double>(2) - cameraPoints[i].z;
        sumSqErr += dx * dx + dy * dy + dz * dz;
    }
    rmsResidual = std::sqrt(sumSqErr / N);

    return true;
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
// Capture Thread Function - runs continuously for each device
// ============================================================================
void CaptureThread(DeviceInfo* device, CaptureData* captureData, cv::Size patternSize)
{
    std::cout << "[Device " << device->index << "] Capture thread started" << std::endl;

    while (g_captureRunning)
    {
        k4a_capture_t capture = nullptr;
        // Use shorter timeout (100ms) to be more responsive
        k4a_wait_result_t result = k4a_device_get_capture(device->device, &capture, 100);

        if (result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_image_t colorImage = k4a_capture_get_color_image(capture);
            k4a_image_t depthImage = k4a_capture_get_depth_image(capture);

            if (colorImage && depthImage)
            {
                cv::Mat colorMat = K4AImageToMat(colorImage);
                std::vector<cv::Point2f> corners;
                bool found = DetectCheckerboardCorners(colorMat, corners, patternSize);

                // Update capture data (thread-safe)
                {
                    std::lock_guard<std::mutex> lock(captureData->mutex);

                    // Release previous depth image
                    if (captureData->depthImage)
                    {
                        k4a_image_release(captureData->depthImage);
                    }

                    captureData->colorImage = colorMat;
                    captureData->depthImage = depthImage;
                    k4a_image_reference(depthImage);  // Keep reference
                    captureData->corners = corners;
                    captureData->cornersFound = found;
                    captureData->hasNewData = true;
                }
            }

            if (colorImage) k4a_image_release(colorImage);
            if (depthImage) k4a_image_release(depthImage);
            k4a_capture_release(capture);
        }
        else if (result == K4A_WAIT_RESULT_FAILED)
        {
            std::cerr << "[Device " << device->index << "] Capture failed" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // K4A_WAIT_RESULT_TIMEOUT is normal, just continue
    }

    std::cout << "[Device " << device->index << "] Capture thread stopped" << std::endl;
}

// ============================================================================
// Generate 3D object points for checkerboard
// ============================================================================
std::vector<cv::Point3f> GenerateCheckerboardPoints(int rows, int cols, float squareSize)
{
    std::vector<cv::Point3f> points;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            points.push_back(cv::Point3f(c * squareSize, r * squareSize, 0.0f));
        }
    }
    return points;
}

// ============================================================================
// Save HMD Calibration to JSON (T_checker_to_A.json)
// ============================================================================
void SaveHMDCalibrationJSON(const HMDCalibration& calib, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(9);
    file << "{\n";
    file << "  \"description\": \"Checkerboard to Helmet Camera (A) transformation\",\n";
    file << "  \"method\": \"" << calib.method << "\",\n";
    file << "  \"num_captures\": " << calib.numCaptures << ",\n";
    file << "  \"num_used\": " << calib.numUsed << ",\n";
    file << "  \"consistency\": {\n";
    file << "    \"translation_std_dev_mm\": " << calib.translationStdDev << ",\n";
    file << "    \"rotation_std_dev_deg\": " << calib.rotationStdDev << ",\n";
    file << "    \"max_translation_error_mm\": " << calib.maxTranslationError << ",\n";
    file << "    \"max_rotation_error_deg\": " << calib.maxRotationError << "\n";
    file << "  },\n";
    file << "  \"rotation\": [\n";
    for (int r = 0; r < 3; r++)
    {
        file << "    [";
        for (int c = 0; c < 3; c++)
        {
            file << calib.rotation.at<double>(r, c);
            if (c < 2) file << ", ";
        }
        file << "]";
        if (r < 2) file << ",";
        file << "\n";
    }
    file << "  ],\n";
    file << "  \"translation\": ["
         << calib.translation.at<double>(0, 0) << ", "
         << calib.translation.at<double>(1, 0) << ", "
         << calib.translation.at<double>(2, 0) << "],\n";
    file << "  \"rvec\": ["
         << calib.rvec.at<double>(0, 0) << ", "
         << calib.rvec.at<double>(1, 0) << ", "
         << calib.rvec.at<double>(2, 0) << "]\n";
    file << "}\n";
    file.close();

    std::cout << "Saved HMD calibration to: " << filename << std::endl;
}


// ============================================================================
// HMD Calibration Mode - Method 3: Bridge Mode (Simultaneous Observation)
// Uses two checkerboards: Ground board (visible to all) and Helmet board (external only)
// Computes T_checker_to_A by bridging the spatial relationships
// ============================================================================
struct BridgeCaptureData {
    cv::Mat colorImage;
    k4a_image_t depthImage = nullptr;
    std::vector<cv::Point2f> groundCorners;
    std::vector<cv::Point2f> helmetCorners;
    bool groundFound = false;
    bool helmetFound = false;
    bool hasNewData = false;
    std::mutex mutex;
};

// Capture thread for bridge mode (detects both checkerboards)
void BridgeCaptureThread(DeviceInfo* device, BridgeCaptureData* captureData,
                          cv::Size groundPatternSize, cv::Size helmetPatternSize)
{
    std::cout << "[Device " << device->index << "] Bridge capture thread started" << std::endl;

    while (g_captureRunning)
    {
        k4a_capture_t capture = nullptr;
        k4a_wait_result_t result = k4a_device_get_capture(device->device, &capture, 100);

        if (result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_image_t colorImage = k4a_capture_get_color_image(capture);
            k4a_image_t depthImage = k4a_capture_get_depth_image(capture);

            if (colorImage && depthImage)
            {
                cv::Mat colorMat = K4AImageToMat(colorImage);

                std::vector<cv::Point2f> groundCorners, helmetCorners;
                bool groundFound = DetectCheckerboardCorners(colorMat, groundCorners, groundPatternSize);
                bool helmetFound = DetectCheckerboardCorners(colorMat, helmetCorners, helmetPatternSize);

                // Update capture data (thread-safe)
                {
                    std::lock_guard<std::mutex> lock(captureData->mutex);

                    if (captureData->depthImage)
                    {
                        k4a_image_release(captureData->depthImage);
                    }

                    captureData->colorImage = colorMat;
                    captureData->depthImage = depthImage;
                    k4a_image_reference(depthImage);
                    captureData->groundCorners = groundCorners;
                    captureData->helmetCorners = helmetCorners;
                    captureData->groundFound = groundFound;
                    captureData->helmetFound = helmetFound;
                    captureData->hasNewData = true;
                }
            }

            if (colorImage) k4a_image_release(colorImage);
            if (depthImage) k4a_image_release(depthImage);
            k4a_capture_release(capture);
        }
    }

    std::cout << "[Device " << device->index << "] Bridge capture thread stopped" << std::endl;
}

int RunHMDCalibrationBridge(
    // Ground checkerboard (visible to external + helmet cameras)
    int groundRows, int groundCols, float groundSquare,
    // Helmet checkerboard (visible to external cameras only)
    int helmetRows, int helmetCols, float helmetSquare,
    // Camera serial numbers
    const std::string& helmetSerial,
    const std::string& primarySerial,
    // Output
    const std::string& outputFile,
    // Use depth-based Horn's method instead of solvePnP
    bool useDepth = false)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "HMD Calibration Mode (Bridge Method)" << std::endl;
    std::cout << "========================================\n" << std::endl;

    std::cout << "Ground Checkerboard: " << groundCols << "x" << groundRows
              << " inner corners, " << groundSquare << "mm squares" << std::endl;
    std::cout << "Helmet Checkerboard: " << helmetCols << "x" << helmetRows
              << " inner corners, " << helmetSquare << "mm squares" << std::endl;
    std::cout << "Pose estimation: " << (useDepth ? "Horn's method (depth-based 3D-to-3D)" : "solvePnP (2D-to-3D)") << std::endl;

    cv::Size groundPatternSize(groundCols, groundRows);
    cv::Size helmetPatternSize(helmetCols, helmetRows);

    std::vector<cv::Point3f> groundObjPoints = GenerateCheckerboardPoints(groundRows, groundCols, groundSquare);
    std::vector<cv::Point3f> helmetObjPoints = GenerateCheckerboardPoints(helmetRows, helmetCols, helmetSquare);

    // Find devices
    uint32_t deviceCount = k4a_device_get_installed_count();
    if (deviceCount < 2)
    {
        std::cerr << "Bridge mode requires at least 2 cameras (helmet + external)" << std::endl;
        return -1;
    }

    std::vector<DeviceInfo> devices;
    int helmetDeviceIdx = -1;
    int externalDeviceIdx = -1;

    for (uint32_t i = 0; i < deviceCount; i++)
    {
        k4a_device_t tempDevice = nullptr;
        if (k4a_device_open(i, &tempDevice) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to open device " << i << std::endl;
            continue;
        }

        DeviceInfo info;
        info.device = tempDevice;
        info.serialNumber = GetDeviceSerialNumber(tempDevice);
        info.index = static_cast<int>(devices.size());

        std::cout << "Device " << info.index << ": " << info.serialNumber;

        if (info.serialNumber == helmetSerial)
        {
            helmetDeviceIdx = info.index;
            std::cout << " (HELMET)";
        }
        else if (primarySerial.empty() || info.serialNumber == primarySerial)
        {
            if (externalDeviceIdx < 0)
            {
                externalDeviceIdx = info.index;
                info.isPrimary = true;
                std::cout << " (EXTERNAL/PRIMARY)";
            }
        }
        std::cout << std::endl;

        devices.push_back(info);
    }

    if (helmetDeviceIdx < 0)
    {
        std::cerr << "Helmet camera not found! Specify with --helmet-serial" << std::endl;
        for (auto& dev : devices) { k4a_device_close(dev.device); }
        return -1;
    }

    if (externalDeviceIdx < 0)
    {
        // Use first non-helmet camera
        for (size_t i = 0; i < devices.size(); i++)
        {
            if (static_cast<int>(i) != helmetDeviceIdx)
            {
                externalDeviceIdx = static_cast<int>(i);
                devices[i].isPrimary = true;
                break;
            }
        }
    }

    // Close unused cameras (bridge mode only needs helmet + external)
    for (int i = static_cast<int>(devices.size()) - 1; i >= 0; i--)
    {
        if (i != helmetDeviceIdx && i != externalDeviceIdx)
        {
            std::cout << "Closing unused device " << i << " (" << devices[i].serialNumber << ")" << std::endl;
            k4a_device_close(devices[i].device);
            devices.erase(devices.begin() + i);
            // Adjust indices after removal
            if (helmetDeviceIdx > i) helmetDeviceIdx--;
            if (externalDeviceIdx > i) externalDeviceIdx--;
        }
    }

    std::cout << "\nHelmet camera: Device " << helmetDeviceIdx << std::endl;
    std::cout << "External camera: Device " << externalDeviceIdx << std::endl;

    // Configure and start cameras
    std::cout << "\nStarting cameras..." << std::endl;

    int subordinateCount = 1;  // First subordinate gets 160*1, second gets 160*2, etc.
    for (int i = static_cast<int>(devices.size()) - 1; i >= 0; i--)
    {
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.color_resolution = useDepth ? K4A_COLOR_RESOLUTION_720P : K4A_COLOR_RESOLUTION_1080P;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.synchronized_images_only = true;

        // Subordinate delay prevents IR interference between depth cameras.
        // Each subordinate offsets by 160us * N (N=1,2,...) from master.
        // Reference: Orbbec Femto Bolt multi-device sync documentation
        if (devices.size() > 1)
        {
            if (devices[i].isPrimary)
            {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
            }
            else
            {
                config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
                config.subordinate_delay_off_master_usec = 160 * subordinateCount;
                subordinateCount++;
            }
        }
        else
        {
            config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        }

        if (k4a_device_start_cameras(devices[i].device, &config) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to start cameras on device " << i << std::endl;
            return -1;
        }

        if (k4a_device_get_calibration(devices[i].device, config.depth_mode,
                                        config.color_resolution, &devices[i].calibration) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to get calibration for device " << i << std::endl;
            return -1;
        }

        devices[i].colorWidth = devices[i].calibration.color_camera_calibration.resolution_width;
        devices[i].colorHeight = devices[i].calibration.color_camera_calibration.resolution_height;
        devices[i].transformation = k4a_transformation_create(&devices[i].calibration);

        if (i > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // Extract camera intrinsics
    auto GetCameraMatrix = [](const k4a_calibration_t& calib) -> cv::Mat {
        auto& intrinsics = calib.color_camera_calibration.intrinsics.parameters.param;
        return (cv::Mat_<double>(3, 3) <<
            intrinsics.fx, 0, intrinsics.cx,
            0, intrinsics.fy, intrinsics.cy,
            0, 0, 1);
    };

    auto GetDistCoeffs = [](const k4a_calibration_t& calib) -> cv::Mat {
        auto& intrinsics = calib.color_camera_calibration.intrinsics.parameters.param;
        return (cv::Mat_<double>(8, 1) <<
            intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2,
            intrinsics.k3, intrinsics.k4, intrinsics.k5, intrinsics.k6);
    };

    cv::Mat helmetCamMatrix = GetCameraMatrix(devices[helmetDeviceIdx].calibration);
    cv::Mat helmetDistCoeffs = GetDistCoeffs(devices[helmetDeviceIdx].calibration);
    cv::Mat externalCamMatrix = GetCameraMatrix(devices[externalDeviceIdx].calibration);
    cv::Mat externalDistCoeffs = GetDistCoeffs(devices[externalDeviceIdx].calibration);

    // Start capture threads
    std::vector<BridgeCaptureData> captureData(devices.size());
    std::vector<std::thread> captureThreads;
    g_captureRunning = true;

    for (size_t i = 0; i < devices.size(); i++)
    {
        captureThreads.emplace_back(BridgeCaptureThread, &devices[i], &captureData[i],
                                     groundPatternSize, helmetPatternSize);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create display windows
    cv::namedWindow("Helmet Camera", cv::WINDOW_NORMAL);
    cv::namedWindow("External Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("Helmet Camera", 800, 450);
    cv::resizeWindow("External Camera", 800, 450);

    std::cout << "\n=== Bridge Calibration Instructions ===" << std::endl;
    std::cout << "1. Place GROUND checkerboard (" << groundCols << "x" << groundRows << ") on floor" << std::endl;
    std::cout << "2. Attach HELMET checkerboard (" << helmetCols << "x" << helmetRows << ") to helmet" << std::endl;
    std::cout << "3. Ensure GROUND board is visible to BOTH cameras" << std::endl;
    std::cout << "4. Ensure HELMET board is visible to EXTERNAL camera" << std::endl;
    std::cout << "5. Press SPACE to capture when all boards detected" << std::endl;
    std::cout << "6. Press 'C' to compute calibration" << std::endl;
    std::cout << "7. Press 'S' to save" << std::endl;
    std::cout << "8. Press ESC to quit" << std::endl;

    // Storage for captures
    struct BridgeCapture {
        cv::Mat T_ground_to_external;   // Ground board pose in external camera
        cv::Mat T_helmet_to_external;   // Helmet board pose in external camera
        cv::Mat T_ground_to_helmet;     // Ground board pose in helmet camera
    };
    std::vector<BridgeCapture> captures;
    HMDCalibration hmdCalib;

    bool running = true;
    while (running)
    {
        cv::Mat helmetColorImg, externalColorImg;
        k4a_image_t helmetDepthImg = nullptr, externalDepthImg = nullptr;
        std::vector<cv::Point2f> helmetGroundCorners, externalGroundCorners, externalHelmetCorners;
        bool helmetGroundFound = false, externalGroundFound = false, externalHelmetFound = false;

        // Get latest data from cameras
        {
            std::lock_guard<std::mutex> lock(captureData[helmetDeviceIdx].mutex);
            if (captureData[helmetDeviceIdx].hasNewData)
            {
                helmetColorImg = captureData[helmetDeviceIdx].colorImage.clone();
                if (captureData[helmetDeviceIdx].depthImage)
                {
                    helmetDepthImg = captureData[helmetDeviceIdx].depthImage;
                    k4a_image_reference(helmetDepthImg);
                }
                helmetGroundCorners = captureData[helmetDeviceIdx].groundCorners;
                helmetGroundFound = captureData[helmetDeviceIdx].groundFound;
            }
        }
        {
            std::lock_guard<std::mutex> lock(captureData[externalDeviceIdx].mutex);
            if (captureData[externalDeviceIdx].hasNewData)
            {
                externalColorImg = captureData[externalDeviceIdx].colorImage.clone();
                if (captureData[externalDeviceIdx].depthImage)
                {
                    externalDepthImg = captureData[externalDeviceIdx].depthImage;
                    k4a_image_reference(externalDepthImg);
                }
                externalGroundCorners = captureData[externalDeviceIdx].groundCorners;
                externalHelmetCorners = captureData[externalDeviceIdx].helmetCorners;
                externalGroundFound = captureData[externalDeviceIdx].groundFound;
                externalHelmetFound = captureData[externalDeviceIdx].helmetFound;
            }
        }

        // Display helmet camera
        if (!helmetColorImg.empty())
        {
            cv::Mat display;
            cv::cvtColor(helmetColorImg, display, cv::COLOR_BGRA2BGR);

            if (helmetGroundFound)
            {
                cv::drawChessboardCorners(display, groundPatternSize, helmetGroundCorners, true);
                cv::putText(display, "GROUND BOARD FOUND", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            }
            else
            {
                cv::putText(display, "Ground board NOT found", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }

            int hYPos = 60;
            if (useDepth)
            {
                if (helmetDepthImg)
                    cv::putText(display, "DEPTH OK", cv::Point(10, hYPos),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                else
                    cv::putText(display, "NO DEPTH", cv::Point(10, hYPos),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                hYPos += 30;
            }

            cv::putText(display, "Captures: " + std::to_string(captures.size()),
                        cv::Point(10, hYPos), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

            cv::imshow("Helmet Camera", display);
        }

        // Display external camera
        if (!externalColorImg.empty())
        {
            cv::Mat display;
            cv::cvtColor(externalColorImg, display, cv::COLOR_BGRA2BGR);

            int yPos = 30;
            if (externalGroundFound)
            {
                cv::drawChessboardCorners(display, groundPatternSize, externalGroundCorners, true);
                cv::putText(display, "GROUND BOARD FOUND", cv::Point(10, yPos),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            }
            else
            {
                cv::putText(display, "Ground board NOT found", cv::Point(10, yPos),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
            yPos += 30;

            if (externalHelmetFound)
            {
                cv::drawChessboardCorners(display, helmetPatternSize, externalHelmetCorners, true);
                cv::putText(display, "HELMET BOARD FOUND", cv::Point(10, yPos),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            }
            else
            {
                cv::putText(display, "Helmet board NOT found", cv::Point(10, yPos),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 128, 255), 2);
            }
            yPos += 30;

            if (useDepth)
            {
                if (externalDepthImg)
                    cv::putText(display, "DEPTH OK", cv::Point(10, yPos),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                else
                    cv::putText(display, "NO DEPTH", cv::Point(10, yPos),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                yPos += 30;
            }

            // Ready to capture?
            bool canCapture = helmetGroundFound && externalGroundFound && externalHelmetFound;
            if (canCapture)
            {
                cv::putText(display, "READY - Press SPACE", cv::Point(10, yPos),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            }

            if (hmdCalib.isValid)
            {
                yPos += 30;
                cv::putText(display, "CALIBRATED - Press S to save", cv::Point(10, yPos),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            }

            cv::imshow("External Camera", display);
        }

        int key = cv::waitKey(30);

        if (key == 27)  // ESC
        {
            running = false;
        }
        else if (key == ' ')  // SPACE - Capture
        {
            bool canCapture = helmetGroundFound && externalGroundFound && externalHelmetFound;
            if (canCapture)
            {
                BridgeCapture cap;
                bool captureOk = false;

                if (useDepth)
                {
                    // Depth-based Horn's method: Convert 2D corners to 3D via depth,
                    // then compute pose using 3D-to-3D registration
                    do {
                        if (!externalDepthImg || !helmetDepthImg)
                        {
                            std::cout << "No depth images available for depth-based pose" << std::endl;
                            break;
                        }

                        double rms = 0.0;

                        // T_ground_to_external: Ground board 3D points in external camera
                        std::vector<cv::Point3f> extGroundPts3D;
                        if (!Convert2DTo3D(devices[externalDeviceIdx], externalDepthImg, nullptr,
                                           externalGroundCorners, extGroundPts3D))
                        {
                            std::cout << "Failed to convert ground corners to 3D (external)" << std::endl;
                            break;
                        }
                        if (!ComputePose3DTo3D(groundObjPoints, extGroundPts3D,
                                              cap.T_ground_to_external, rms))
                        {
                            std::cout << "Failed Horn's method for ground->external" << std::endl;
                            break;
                        }
                        std::cout << "  Ground->External RMS: " << std::fixed << std::setprecision(2) << rms << " mm" << std::endl;

                        // T_helmet_to_external: Helmet board 3D points in external camera
                        std::vector<cv::Point3f> extHelmetPts3D;
                        if (!Convert2DTo3D(devices[externalDeviceIdx], externalDepthImg, nullptr,
                                           externalHelmetCorners, extHelmetPts3D))
                        {
                            std::cout << "Failed to convert helmet corners to 3D (external)" << std::endl;
                            break;
                        }
                        if (!ComputePose3DTo3D(helmetObjPoints, extHelmetPts3D,
                                              cap.T_helmet_to_external, rms))
                        {
                            std::cout << "Failed Horn's method for helmet->external" << std::endl;
                            break;
                        }
                        std::cout << "  Helmet->External RMS: " << std::fixed << std::setprecision(2) << rms << " mm" << std::endl;

                        // T_ground_to_helmet: Ground board 3D points in helmet camera
                        std::vector<cv::Point3f> helGroundPts3D;
                        if (!Convert2DTo3D(devices[helmetDeviceIdx], helmetDepthImg, nullptr,
                                           helmetGroundCorners, helGroundPts3D))
                        {
                            std::cout << "Failed to convert ground corners to 3D (helmet)" << std::endl;
                            break;
                        }
                        if (!ComputePose3DTo3D(groundObjPoints, helGroundPts3D,
                                              cap.T_ground_to_helmet, rms))
                        {
                            std::cout << "Failed Horn's method for ground->helmet" << std::endl;
                            break;
                        }
                        std::cout << "  Ground->Helmet RMS: " << std::fixed << std::setprecision(2) << rms << " mm" << std::endl;

                        captureOk = true;
                    } while (false);
                }
                else
                {
                    // solvePnP: 2D corners + known geometry + camera intrinsics
                    cv::Mat rvec, tvec;

                    // T_ground_to_external: Ground board in external camera frame
                    cv::solvePnP(groundObjPoints, externalGroundCorners,
                                 externalCamMatrix, externalDistCoeffs, rvec, tvec);
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    cap.T_ground_to_external = cv::Mat::eye(4, 4, CV_64F);
                    R.copyTo(cap.T_ground_to_external(cv::Rect(0, 0, 3, 3)));
                    tvec.copyTo(cap.T_ground_to_external(cv::Rect(3, 0, 1, 3)));

                    // T_helmet_to_external: Helmet board in external camera frame
                    cv::solvePnP(helmetObjPoints, externalHelmetCorners,
                                 externalCamMatrix, externalDistCoeffs, rvec, tvec);
                    cv::Rodrigues(rvec, R);
                    cap.T_helmet_to_external = cv::Mat::eye(4, 4, CV_64F);
                    R.copyTo(cap.T_helmet_to_external(cv::Rect(0, 0, 3, 3)));
                    tvec.copyTo(cap.T_helmet_to_external(cv::Rect(3, 0, 1, 3)));

                    // T_ground_to_helmet: Ground board in helmet camera frame
                    cv::solvePnP(groundObjPoints, helmetGroundCorners,
                                 helmetCamMatrix, helmetDistCoeffs, rvec, tvec);
                    cv::Rodrigues(rvec, R);
                    cap.T_ground_to_helmet = cv::Mat::eye(4, 4, CV_64F);
                    R.copyTo(cap.T_ground_to_helmet(cv::Rect(0, 0, 3, 3)));
                    tvec.copyTo(cap.T_ground_to_helmet(cv::Rect(3, 0, 1, 3)));

                    captureOk = true;
                }

                if (captureOk)
                {
                    captures.push_back(cap);
                    std::cout << "Capture " << captures.size() << " recorded" << std::endl;
                }
            }
            else
            {
                std::cout << "Cannot capture: not all boards detected" << std::endl;
            }
        }
        else if ((key == 'c' || key == 'C') && !captures.empty())  // Compute
        {
            std::cout << "\n=== Computing Bridge Calibration ===" << std::endl;
            std::cout << "Total captures: " << captures.size() << std::endl;

            // Step 1: Compute T_checker_to_A independently for each capture
            struct PerCaptureResult {
                cv::Mat R;          // 3x3 rotation
                cv::Mat t;          // 3x1 translation
                double tNorm;       // ||t|| in mm
                double angle;       // rotation angle in degrees
                bool isOutlier = false;
            };

            std::vector<PerCaptureResult> results;
            results.reserve(captures.size());

            for (const auto& cap : captures)
            {
                cv::Mat T_ext_to_helmet = cap.T_helmet_to_external.inv();
                cv::Mat T_helmet_to_ground = cap.T_ground_to_helmet.inv();
                cv::Mat T_result = T_ext_to_helmet * cap.T_ground_to_external * T_helmet_to_ground;

                PerCaptureResult res;
                res.R = T_result(cv::Rect(0, 0, 3, 3)).clone();
                res.t = T_result(cv::Rect(3, 0, 1, 3)).clone();
                res.tNorm = cv::norm(res.t);

                // Rotation angle from axis-angle representation
                cv::Mat rvec;
                cv::Rodrigues(res.R, rvec);
                res.angle = cv::norm(rvec) * 180.0 / CV_PI;

                results.push_back(res);
            }

            // Step 2: Compute per-capture statistics
            double meanTNorm = 0.0, meanAngle = 0.0;
            for (const auto& res : results)
            {
                meanTNorm += res.tNorm;
                meanAngle += res.angle;
            }
            meanTNorm /= results.size();
            meanAngle /= results.size();

            double varTNorm = 0.0, varAngle = 0.0;
            for (const auto& res : results)
            {
                varTNorm += (res.tNorm - meanTNorm) * (res.tNorm - meanTNorm);
                varAngle += (res.angle - meanAngle) * (res.angle - meanAngle);
            }
            double stdTNorm = std::sqrt(varTNorm / results.size());
            double stdAngle = std::sqrt(varAngle / results.size());

            // Step 3: Print per-capture results
            std::cout << "\n--- Per-Capture Results ---" << std::endl;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "  #   ||t|| (mm)   angle (deg)   status" << std::endl;
            std::cout << "  --  ----------   -----------   ------" << std::endl;

            // Step 4: Flag outliers (>2 sigma from mean in either translation or rotation)
            const double outlierSigma = 2.0;
            for (size_t i = 0; i < results.size(); i++)
            {
                auto& res = results[i];
                double tDev = std::abs(res.tNorm - meanTNorm);
                double aDev = std::abs(res.angle - meanAngle);

                // Only flag outliers if we have enough captures and nonzero std dev
                if (results.size() >= 4)
                {
                    if ((stdTNorm > 0.01 && tDev > outlierSigma * stdTNorm) ||
                        (stdAngle > 0.01 && aDev > outlierSigma * stdAngle))
                    {
                        res.isOutlier = true;
                    }
                }

                std::cout << "  " << std::setw(2) << (i + 1)
                          << "  " << std::setw(10) << res.tNorm
                          << "   " << std::setw(11) << res.angle
                          << "   " << (res.isOutlier ? "OUTLIER" : "OK") << std::endl;
            }

            std::cout << "  --  ----------   -----------" << std::endl;
            std::cout << "  Mean: " << std::setw(8) << meanTNorm
                      << "   " << std::setw(11) << meanAngle << std::endl;
            std::cout << "  StdDev: " << std::setw(6) << stdTNorm
                      << "   " << std::setw(11) << stdAngle << std::endl;

            // Step 5: Average only non-outlier captures
            cv::Mat avgR = cv::Mat::zeros(3, 3, CV_64F);
            cv::Mat avgT = cv::Mat::zeros(3, 1, CV_64F);
            int usedCount = 0;

            for (const auto& res : results)
            {
                if (!res.isOutlier)
                {
                    avgR += res.R;
                    avgT += res.t;
                    usedCount++;
                }
            }

            if (usedCount == 0)
            {
                std::cout << "\nWARNING: All captures flagged as outliers. Using all captures." << std::endl;
                for (const auto& res : results)
                {
                    avgR += res.R;
                    avgT += res.t;
                }
                usedCount = static_cast<int>(results.size());
            }

            int outlierCount = static_cast<int>(results.size()) - usedCount;
            if (outlierCount > 0)
            {
                std::cout << "\nRemoved " << outlierCount << " outlier(s), using "
                          << usedCount << "/" << results.size() << " captures" << std::endl;
            }

            avgR /= static_cast<double>(usedCount);
            avgT /= static_cast<double>(usedCount);

            // Step 6: Recompute std dev on used captures only
            double finalMeanTNorm = 0.0, finalMeanAngle = 0.0;
            double maxTErr = 0.0, maxAErr = 0.0;
            int finalCount = 0;
            for (const auto& res : results)
            {
                if (!res.isOutlier)
                {
                    finalMeanTNorm += res.tNorm;
                    finalMeanAngle += res.angle;
                    finalCount++;
                }
            }
            finalMeanTNorm /= finalCount;
            finalMeanAngle /= finalCount;

            double finalVarT = 0.0, finalVarA = 0.0;
            for (const auto& res : results)
            {
                if (!res.isOutlier)
                {
                    double tErr = std::abs(res.tNorm - finalMeanTNorm);
                    double aErr = std::abs(res.angle - finalMeanAngle);
                    finalVarT += tErr * tErr;
                    finalVarA += aErr * aErr;
                    if (tErr > maxTErr) maxTErr = tErr;
                    if (aErr > maxAErr) maxAErr = aErr;
                }
            }
            double finalStdT = std::sqrt(finalVarT / finalCount);
            double finalStdA = std::sqrt(finalVarA / finalCount);

            // Re-orthogonalize R using SVD
            cv::Mat U, S, Vt;
            cv::SVD::compute(avgR, S, U, Vt);
            cv::Mat finalR = U * Vt;

            // Ensure proper rotation (det = +1)
            if (cv::determinant(finalR) < 0)
            {
                finalR.col(2) *= -1.0;
            }

            cv::Mat rvec;
            cv::Rodrigues(finalR, rvec);

            hmdCalib.rotation = finalR;
            hmdCalib.translation = avgT;
            hmdCalib.rvec = rvec;
            hmdCalib.numCaptures = static_cast<int>(captures.size());
            hmdCalib.numUsed = usedCount;
            hmdCalib.translationStdDev = finalStdT;
            hmdCalib.rotationStdDev = finalStdA;
            hmdCalib.maxTranslationError = maxTErr;
            hmdCalib.maxRotationError = maxAErr;
            hmdCalib.method = useDepth ? "horn_3d_depth" : "solvePnP";
            hmdCalib.isValid = true;

            // Step 7: Print final results with quality assessment
            std::cout << "\n=== Bridge Calibration Complete ===" << std::endl;
            std::cout << "Rotation matrix:\n" << finalR << std::endl;
            std::cout << "Translation (mm): " << avgT.t() << std::endl;
            std::cout << "||t|| = " << cv::norm(avgT) << " mm" << std::endl;

            std::cout << "\n--- Consistency Metrics (after outlier removal) ---" << std::endl;
            std::cout << "  Captures used:      " << usedCount << "/" << results.size() << std::endl;
            std::cout << "  ||t|| std dev:      " << finalStdT << " mm" << std::endl;
            std::cout << "  ||t|| max error:    " << maxTErr << " mm" << std::endl;
            std::cout << "  Angle std dev:      " << finalStdA << " deg" << std::endl;
            std::cout << "  Angle max error:    " << maxAErr << " deg" << std::endl;

            // Quality assessment
            std::cout << "\n--- Quality Assessment ---" << std::endl;
            if (finalStdT < 2.0 && finalStdA < 1.0)
                std::cout << "  EXCELLENT: Very consistent across captures" << std::endl;
            else if (finalStdT < 5.0 && finalStdA < 2.0)
                std::cout << "  GOOD: Reasonably consistent" << std::endl;
            else if (finalStdT < 10.0 && finalStdA < 5.0)
                std::cout << "  FAIR: Consider recapturing with better poses" << std::endl;
            else
                std::cout << "  POOR: High variance, recapture recommended" << std::endl;

            std::cout << "  TIP: Physically measure checkerboard-to-lens distance and compare to ||t|| = "
                      << std::setprecision(1) << cv::norm(avgT) << " mm" << std::endl;
        }
        else if ((key == 's' || key == 'S') && hmdCalib.isValid)  // Save
        {
            SaveHMDCalibrationJSON(hmdCalib, outputFile);
        }

        // Release depth images acquired this iteration
        if (helmetDepthImg) k4a_image_release(helmetDepthImg);
        if (externalDepthImg) k4a_image_release(externalDepthImg);
    }

    // Cleanup
    g_captureRunning = false;
    for (auto& thread : captureThreads)
    {
        if (thread.joinable()) thread.join();
    }

    for (auto& data : captureData)
    {
        if (data.depthImage) k4a_image_release(data.depthImage);
    }

    cv::destroyAllWindows();

    for (auto& device : devices)
    {
        if (device.transformation) k4a_transformation_destroy(device.transformation);
        if (device.device)
        {
            k4a_device_stop_cameras(device.device);
            k4a_device_close(device.device);
        }
    }

    return 0;
}

// ============================================================================
// Print Usage
// ============================================================================
void PrintUsage()
{
    std::cout << "\n=== Multi-Device Extrinsic Calibration Tool ===\n"
              << "USAGE: multi_device_calibration.exe [OPTIONS]\n\n"
              << "=== Mode 1: Multi-Camera Calibration (default) ===\n"
              << "Calibrates extrinsics between multiple fixed cameras.\n\n"
              << "Options:\n"
              << "  --rows N         Checkerboard inner corners (rows), default: " << CHECKERBOARD_ROWS << "\n"
              << "  --cols N         Checkerboard inner corners (cols), default: " << CHECKERBOARD_COLS << "\n"
              << "  --square N       Square size in mm, default: " << SQUARE_SIZE_MM << "\n"
              << "  --output FILE    Output filename prefix, default: calibration\n"
              << "  --primary SERIAL Serial number of PRIMARY camera (sync hub master port)\n"
              << "  --exclude SERIAL Exclude camera by serial number (can be used multiple times)\n"
              << "\n=== Mode 2: HMD Bridge Calibration (T_checker_to_A) ===\n"
              << "Calibrates helmet camera using two checkerboards (bridge method).\n"
              << "Ground board: visible to BOTH external and helmet cameras\n"
              << "Helmet board: visible ONLY to external camera (attached to helmet)\n\n"
              << "Options:\n"
              << "  --hmd-bridge              Enable HMD bridge calibration mode\n"
              << "  --bridge-depth            Use depth-based Horn's method instead of solvePnP\n"
              << "  --helmet-serial SERIAL    Helmet camera serial number (required)\n"
              << "  --ground-rows N           Ground checkerboard rows (default: " << CHECKERBOARD_ROWS << ")\n"
              << "  --ground-cols N           Ground checkerboard cols (default: " << CHECKERBOARD_COLS << ")\n"
              << "  --ground-square N         Ground square size in mm (default: " << SQUARE_SIZE_MM << ")\n"
              << "  --helmet-rows N           Helmet checkerboard rows (default: " << HEAD_CB_ROWS << ")\n"
              << "  --helmet-cols N           Helmet checkerboard cols (default: " << HEAD_CB_COLS << ")\n"
              << "  --helmet-square N         Helmet square size in mm (default: " << HEAD_CB_SQUARE_MM << ")\n"
              << "\nInstructions (Multi-Camera):\n"
              << "  1. Place checkerboard visible to ALL cameras\n"
              << "  2. Press SPACE to capture and calibrate\n"
              << "  3. Press 'S' to save calibration\n"
              << "  4. Press ESC to quit\n"
              << "\nInstructions (HMD Bridge Calibration):\n"
              << "  1. Place GROUND checkerboard on floor (visible to all cameras)\n"
              << "  2. Attach HELMET checkerboard to helmet (visible to external camera)\n"
              << "  3. Position so external camera sees BOTH boards, helmet camera sees GROUND board\n"
              << "  4. Press SPACE to capture when all boards detected\n"
              << "  5. Press 'C' to compute calibration\n"
              << "  6. Press 'S' to save T_checker_to_A.json\n"
              << "\nExamples:\n"
              << "  # Multi-camera calibration (exclude helmet camera)\n"
              << "  multi_device_calibration.exe --primary CL8T75400DC --exclude CL8T75400GD\n"
              << "\n  # HMD bridge calibration\n"
              << "  multi_device_calibration.exe --hmd-bridge --helmet-serial CL8T75400GD \\\n"
              << "      --ground-rows 6 --ground-cols 9 --ground-square 25 \\\n"
              << "      --helmet-rows 4 --helmet-cols 5 --helmet-square 30 \\\n"
              << "      --output T_checker_to_A\n"
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
    std::string primarySerial = "";  // Serial number of PRIMARY camera (sync hub master port)
    std::vector<std::string> excludeSerials;  // Serial numbers to exclude from calibration

    // HMD bridge calibration options
    bool hmdBridgeMode = false;
    bool bridgeDepth = false;
    std::string helmetSerial = "";
    int groundRows = CHECKERBOARD_ROWS;
    int groundCols = CHECKERBOARD_COLS;
    float groundSquare = SQUARE_SIZE_MM;
    int helmetRows = HEAD_CB_ROWS;
    int helmetCols = HEAD_CB_COLS;
    float helmetSquare = HEAD_CB_SQUARE_MM;

    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "--rows" && i + 1 < argc) checkerboardRows = std::atoi(argv[++i]);
        else if (arg == "--cols" && i + 1 < argc) checkerboardCols = std::atoi(argv[++i]);
        else if (arg == "--square" && i + 1 < argc) squareSize = static_cast<float>(std::atof(argv[++i]));
        else if (arg == "--output" && i + 1 < argc) outputPrefix = argv[++i];
        else if (arg == "--primary" && i + 1 < argc) primarySerial = argv[++i];
        else if (arg == "--exclude" && i + 1 < argc) excludeSerials.push_back(argv[++i]);
        // HMD bridge mode options
        else if (arg == "--hmd-bridge") hmdBridgeMode = true;
        else if (arg == "--bridge-depth") bridgeDepth = true;
        else if (arg == "--helmet-serial" && i + 1 < argc) helmetSerial = argv[++i];
        else if (arg == "--ground-rows" && i + 1 < argc) groundRows = std::atoi(argv[++i]);
        else if (arg == "--ground-cols" && i + 1 < argc) groundCols = std::atoi(argv[++i]);
        else if (arg == "--ground-square" && i + 1 < argc) groundSquare = static_cast<float>(std::atof(argv[++i]));
        else if (arg == "--helmet-rows" && i + 1 < argc) helmetRows = std::atoi(argv[++i]);
        else if (arg == "--helmet-cols" && i + 1 < argc) helmetCols = std::atoi(argv[++i]);
        else if (arg == "--helmet-square" && i + 1 < argc) helmetSquare = static_cast<float>(std::atof(argv[++i]));
        else if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return 0;
        }
    }

    // Dispatch to HMD bridge calibration mode if requested
    if (hmdBridgeMode)
    {
        std::string hmdOutputFile = outputPrefix;
        if (hmdOutputFile.find(".json") == std::string::npos)
        {
            hmdOutputFile += ".json";
        }

        return RunHMDCalibrationBridge(
            groundRows, groundCols, groundSquare,
            helmetRows, helmetCols, helmetSquare,
            helmetSerial, primarySerial,
            hmdOutputFile, bridgeDepth);
    }

    cv::Size patternSize(checkerboardCols, checkerboardRows);
    std::cout << "Checkerboard: " << checkerboardCols << "x" << checkerboardRows
              << " inner corners, " << squareSize << "mm squares" << std::endl;

    // Print excluded cameras if any
    if (!excludeSerials.empty())
    {
        std::cout << "Excluding cameras: ";
        for (size_t i = 0; i < excludeSerials.size(); i++)
        {
            std::cout << excludeSerials[i];
            if (i < excludeSerials.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }

    // Get device count
    uint32_t installedDeviceCount = k4a_device_get_installed_count();
    std::cout << "Found " << installedDeviceCount << " device(s)" << std::endl;

    if (installedDeviceCount == 0)
    {
        std::cerr << "No devices found!" << std::endl;
        return -1;
    }

    // Helper lambda to check if a serial number should be excluded
    auto isExcluded = [&excludeSerials](const std::string& serial) -> bool {
        return std::find(excludeSerials.begin(), excludeSerials.end(), serial) != excludeSerials.end();
    };

    // Open all devices, filtering out excluded ones
    std::vector<DeviceInfo> devices;
    devices.reserve(installedDeviceCount);

    for (uint32_t i = 0; i < installedDeviceCount; i++)
    {
        k4a_device_t tempDevice = nullptr;

        if (k4a_device_open(i, &tempDevice) != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to open device " << i << std::endl;
            return -1;
        }

        std::string serial = GetDeviceSerialNumber(tempDevice);

        if (isExcluded(serial))
        {
            std::cout << "Device " << i << " (SN=" << serial << "): EXCLUDED" << std::endl;
            k4a_device_close(tempDevice);
            continue;
        }

        DeviceInfo info;
        info.device = tempDevice;
        info.serialNumber = serial;
        info.index = static_cast<int>(devices.size());  // New index after filtering
        devices.push_back(info);
    }

    uint32_t deviceCount = static_cast<uint32_t>(devices.size());
    std::cout << "Using " << deviceCount << " device(s) for calibration" << std::endl;

    if (deviceCount == 0)
    {
        std::cerr << "No devices available after exclusion!" << std::endl;
        return -1;
    }

    if (deviceCount < 2)
    {
        std::cout << "Warning: Only 1 device available. Multi-device calibration requires 2+ cameras." << std::endl;
    }

    // Determine PRIMARY camera based on serial number
    int primaryIndex = -1;
    if (!primarySerial.empty())
    {
        for (uint32_t i = 0; i < deviceCount; i++)
        {
            if (devices[i].serialNumber == primarySerial)
            {
                primaryIndex = i;
                break;
            }
        }
        if (primaryIndex < 0)
        {
            std::cerr << "WARNING: Primary serial '" << primarySerial << "' not found!" << std::endl;
            std::cerr << "Available devices:" << std::endl;
            for (uint32_t i = 0; i < deviceCount; i++)
            {
                std::cerr << "  Device " << i << ": " << devices[i].serialNumber << std::endl;
            }
            std::cerr << "Falling back to device 0 as primary." << std::endl;
            primaryIndex = 0;
        }
    }
    else
    {
        // No primary specified, use device 0
        primaryIndex = 0;
        std::cout << "No --primary specified, using device 0 as primary." << std::endl;
        std::cout << "Use --primary <serial> to specify sync hub master camera." << std::endl;
    }

    for (uint32_t i = 0; i < deviceCount; i++)
    {
        devices[i].isPrimary = (static_cast<int>(i) == primaryIndex);
        std::cout << "Device " << i << ": SN=" << devices[i].serialNumber
                  << " (" << (devices[i].isPrimary ? "PRIMARY/MASTER" : "SECONDARY/SUBORDINATE") << ")" << std::endl;
    }

    // Configure and start cameras
    // Need COLOR for checkerboard detection, DEPTH for 3D conversion
    std::cout << "\nStarting cameras..." << std::endl;

    int mode1SubordinateCount = 1;  // First subordinate gets 160*1, second gets 160*2, etc.
    for (int i = deviceCount - 1; i >= 0; i--)
    {
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.synchronized_images_only = true;

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
                config.subordinate_delay_off_master_usec = 160 * mode1SubordinateCount;
                mode1SubordinateCount++;
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

    // Create capture data and start capture threads for each device
    std::vector<CaptureData> captureData(deviceCount);
    std::vector<std::thread> captureThreads;
    g_captureRunning = true;

    for (uint32_t i = 0; i < deviceCount; i++)
    {
        captureThreads.emplace_back(CaptureThread, &devices[i], &captureData[i], patternSize);
    }

    // Wait a moment for threads to start capturing
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::vector<ExtrinsicCalibration> calibrations(deviceCount);
    bool calibrationDone = false;
    bool running = true;

    while (running)
    {
        std::vector<cv::Mat> colorImages(deviceCount);
        std::vector<k4a_image_t> depthImages(deviceCount, nullptr);
        std::vector<std::vector<cv::Point2f>> allCorners(deviceCount);
        std::vector<bool> cornersFound(deviceCount, false);

        // Get latest capture data from all devices (thread-safe)
        for (uint32_t i = 0; i < deviceCount; i++)
        {
            std::lock_guard<std::mutex> lock(captureData[i].mutex);

            if (captureData[i].hasNewData)
            {
                colorImages[i] = captureData[i].colorImage.clone();
                if (captureData[i].depthImage)
                {
                    depthImages[i] = captureData[i].depthImage;
                    k4a_image_reference(depthImages[i]);  // Keep reference
                }
                allCorners[i] = captureData[i].corners;
                cornersFound[i] = captureData[i].cornersFound;
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

                // Find primary camera index
                int primaryIdx = 0;
                for (uint32_t i = 0; i < deviceCount; i++)
                {
                    if (devices[i].isPrimary)
                    {
                        primaryIdx = i;
                        break;
                    }
                }

                // Fix corner orientation for secondary cameras relative to primary
                // This ensures consistent corner ordering even if checkerboard appears rotated
                std::cout << "Checking corner orientation..." << std::endl;
                for (uint32_t i = 0; i < deviceCount; i++)
                {
                    if (!devices[i].isPrimary)
                    {
                        FixCornerOrientation(allCorners[primaryIdx], allCorners[i]);
                    }
                }

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

    // Stop capture threads
    g_captureRunning = false;
    for (auto& thread : captureThreads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    // Release depth images from capture data
    for (auto& data : captureData)
    {
        if (data.depthImage)
        {
            k4a_image_release(data.depthImage);
        }
    }

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
