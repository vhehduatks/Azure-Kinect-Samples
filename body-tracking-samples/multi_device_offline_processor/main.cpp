// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Multi-device offline body tracking processor
// Processes MKV recordings from multiple cameras with skeleton fusion
// Supports ego-view processing with helmet-mounted camera

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <filesystem>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4abt.h>
#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>

using namespace std;
using json = nlohmann::json;
namespace fs = std::filesystem;

// ============================================================================
// Calibration Data Structures
// ============================================================================
struct CameraExtrinsics {
    string serialNumber;
    int deviceIndex;
    bool isValid;
    float rotation[3][3];
    float translation[3];
};

struct CalibrationData {
    int numDevices;
    vector<CameraExtrinsics> cameras;
    bool isLoaded;
};

// ============================================================================
// Body/Joint Structures for Fusion
// ============================================================================
struct FusedJoint {
    k4a_float3_t position;
    k4a_quaternion_t orientation;
    k4abt_joint_confidence_level_t confidence;
    int sourceDeviceIndex;
};

struct FusedBody {
    uint32_t id;
    FusedJoint joints[K4ABT_JOINT_COUNT];
    int matchCount = 1;  // Number of cameras that contributed to this body
};

struct FrameData {
    uint64_t timestamp_usec;
    int deviceIndex;
    vector<k4abt_body_t> bodies;
};

// ============================================================================
// Ego-View Data Structures
// ============================================================================
struct Transform {
    cv::Mat rotation;       // 3x3 CV_64F
    cv::Mat translation;    // 3x1 CV_64F
    bool valid = false;
};

struct WeightedPose {
    Transform pose;
    float weight;
    int cameraIndex;
};

struct HelmetCBConfig {
    int rows = 4;
    int cols = 5;
    float squareMm = 30.0f;
    cv::Size patternSize() const { return cv::Size(cols, rows); }
};

struct Joint3D {
    float x, y, z;
    int confidence;
    string name;
};

struct CameraHelmetSkeleton {
    int cameraIndex;
    Joint3D joints[K4ABT_JOINT_COUNT];
    float weight;
};

struct Joint2D {
    float u, v;
    int confidence;
    bool visible;
    string name;
};

// ============================================================================
// Global Configuration
// ============================================================================
CalibrationData g_calibration = {0, {}, false};
const float BODY_MATCH_THRESHOLD_MM = 500.0f;

// Forward declarations
float ConfidenceToWeight(k4abt_joint_confidence_level_t conf);

// ============================================================================
// Coordinate Transformation Functions
// ============================================================================
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

k4a_quaternion_t TransformOrientation(const k4a_quaternion_t& orientation, const CameraExtrinsics& ext)
{
    k4a_quaternion_t rotQuat = RotationMatrixToQuaternion(ext.rotation);
    return QuaternionMultiply(rotQuat, orientation);
}

// ============================================================================
// Calibration Loading
// ============================================================================
bool LoadCalibration(const string& path, CalibrationData& cal)
{
    ifstream file(path);
    if (!file.is_open()) {
        cerr << "Failed to open calibration file: " << path << endl;
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
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        cam.rotation[r][c] = camJson["rotation"][r][c];
                    }
                }
                for (int i = 0; i < 3; i++) {
                    cam.translation[i] = camJson["translation"][i];
                }
            } else {
                // Identity for primary camera
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
        cout << "Loaded calibration for " << cal.numDevices << " cameras" << endl;
        return true;
    }
    catch (const exception& e) {
        cerr << "Error parsing calibration JSON: " << e.what() << endl;
        return false;
    }
}

// ============================================================================
// Ego-View: Transform Loading
// ============================================================================
bool LoadTransform(const string& path, Transform& t)
{
    ifstream file(path);
    if (!file.is_open()) {
        cerr << "Failed to open transform: " << path << endl;
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
    }
    catch (const exception& e) {
        cerr << "Error parsing transform: " << e.what() << endl;
        return false;
    }
}

// ============================================================================
// Ego-View: Checkerboard Detection
// ============================================================================
bool DetectCheckerboardCorners(const cv::Mat& colorImage,
                                vector<cv::Point2f>& corners,
                                cv::Size patternSize)
{
    cv::Mat gray;
    if (colorImage.channels() == 4) {
        cv::cvtColor(colorImage, gray, cv::COLOR_BGRA2GRAY);
    } else if (colorImage.channels() == 3) {
        cv::cvtColor(colorImage, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = colorImage;
    }

    bool found = cv::findChessboardCorners(gray, patternSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH |
        cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    return found;
}

// Convert 2D corners to 3D using depth image
bool Convert2DTo3DOffline(const k4a_calibration_t& calibration,
                           k4a_transformation_t transformation,
                           const cv::Mat& depthImage,
                           const vector<cv::Point2f>& corners2D,
                           vector<cv::Point3f>& points3D)
{
    points3D.clear();

    int colorWidth = calibration.color_camera_calibration.resolution_width;
    int colorHeight = calibration.color_camera_calibration.resolution_height;

    // Transform depth to color space
    k4a_image_t depthK4a = nullptr;
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        depthImage.cols, depthImage.rows,
        depthImage.cols * (int)sizeof(uint16_t), &depthK4a);
    memcpy(k4a_image_get_buffer(depthK4a), depthImage.data,
        depthImage.total() * sizeof(uint16_t));

    k4a_image_t transformedDepth = nullptr;
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        colorWidth, colorHeight,
        colorWidth * (int)sizeof(uint16_t), &transformedDepth);

    k4a_result_t result = k4a_transformation_depth_image_to_color_camera(
        transformation, depthK4a, transformedDepth);

    if (result != K4A_RESULT_SUCCEEDED) {
        k4a_image_release(depthK4a);
        k4a_image_release(transformedDepth);
        return false;
    }

    uint16_t* depthBuffer = reinterpret_cast<uint16_t*>(
        k4a_image_get_buffer(transformedDepth));

    for (const auto& corner : corners2D) {
        int x = static_cast<int>(round(corner.x));
        int y = static_cast<int>(round(corner.y));

        // Sample 3x3 neighborhood for robustness
        float depthSum = 0;
        int validCount = 0;

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int nx = x + dx;
                int ny = y + dy;

                if (nx >= 0 && nx < colorWidth && ny >= 0 && ny < colorHeight) {
                    uint16_t d = depthBuffer[ny * colorWidth + nx];
                    if (d > 0) {
                        depthSum += d;
                        validCount++;
                    }
                }
            }
        }

        if (validCount == 0) {
            k4a_image_release(depthK4a);
            k4a_image_release(transformedDepth);
            return false;
        }

        float depthMm = depthSum / validCount;

        k4a_float2_t point2d = { corner.x, corner.y };
        k4a_float3_t point3d;
        int valid = 0;

        k4a_calibration_2d_to_3d(&calibration, &point2d, depthMm,
            K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR,
            &point3d, &valid);

        if (!valid) {
            k4a_image_release(depthK4a);
            k4a_image_release(transformedDepth);
            return false;
        }

        points3D.push_back(cv::Point3f(point3d.xyz.x, point3d.xyz.y, point3d.xyz.z));
    }

    k4a_image_release(depthK4a);
    k4a_image_release(transformedDepth);
    return true;
}

// Transform 3D points from camera space to world space
void TransformPointsToWorld(vector<cv::Point3f>& points, const CameraExtrinsics& ext)
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

// Compute checkerboard pose from 3D points in world space
Transform ComputeCheckerboardPose(const vector<cv::Point3f>& points3D,
                                   cv::Size patternSize)
{
    Transform result;

    if ((int)points3D.size() < patternSize.width * patternSize.height) return result;

    // Compute centroid
    cv::Point3f centroid(0, 0, 0);
    for (const auto& p : points3D) {
        centroid += p;
    }
    centroid *= (1.0f / points3D.size());

    // Use first row direction as X axis
    cv::Point3f x_axis = points3D[patternSize.width - 1] - points3D[0];
    float x_norm = (float)cv::norm(x_axis);
    if (x_norm < 1e-6f) return result;
    x_axis /= x_norm;

    // Use first column direction as Y axis
    cv::Point3f y_axis = points3D[(patternSize.height - 1) * patternSize.width] - points3D[0];
    float y_norm = (float)cv::norm(y_axis);
    if (y_norm < 1e-6f) return result;
    y_axis /= y_norm;

    // Z axis from cross product
    cv::Point3f z_axis = x_axis.cross(y_axis);
    float z_norm = (float)cv::norm(z_axis);
    if (z_norm < 1e-6f) return result;
    z_axis /= z_norm;

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

// ============================================================================
// Weighted Multi-Camera Helmet Pose Fusion
// ============================================================================
Transform FuseHelmetPoses(const vector<WeightedPose>& candidates)
{
    if (candidates.empty()) {
        Transform t;
        return t;
    }
    if (candidates.size() == 1) {
        return candidates[0].pose;
    }

    // --- Outlier rejection: median-based filtering (3+ candidates) ---
    vector<WeightedPose> filtered = candidates;
    if (filtered.size() >= 3) {
        const double OUTLIER_THRESHOLD_MM = 100.0;

        // Compute per-axis median of translations
        vector<double> xs, ys, zs;
        for (const auto& c : filtered) {
            xs.push_back(c.pose.translation.at<double>(0));
            ys.push_back(c.pose.translation.at<double>(1));
            zs.push_back(c.pose.translation.at<double>(2));
        }
        sort(xs.begin(), xs.end());
        sort(ys.begin(), ys.end());
        sort(zs.begin(), zs.end());
        size_t mid = xs.size() / 2;
        double medX = xs[mid], medY = ys[mid], medZ = zs[mid];

        // Compute distance from median and reject outliers
        double minDist = numeric_limits<double>::max();
        int closestIdx = 0;
        for (size_t i = 0; i < filtered.size(); i++) {
            double dx = filtered[i].pose.translation.at<double>(0) - medX;
            double dy = filtered[i].pose.translation.at<double>(1) - medY;
            double dz = filtered[i].pose.translation.at<double>(2) - medZ;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < minDist) { minDist = dist; closestIdx = static_cast<int>(i); }
            if (dist > OUTLIER_THRESHOLD_MM) {
                filtered[i].weight = 0.0f;
            }
        }

        // If all rejected, fall back to closest to median
        bool anyValid = false;
        for (const auto& c : filtered) {
            if (c.weight > 0.0f) { anyValid = true; break; }
        }
        if (!anyValid) {
            filtered[closestIdx].weight = 1.0f;
        }

        // Remove zero-weight candidates
        vector<WeightedPose> survivors;
        for (const auto& c : filtered) {
            if (c.weight > 0.0f) survivors.push_back(c);
        }
        filtered = survivors;
    }

    if (filtered.size() == 1) {
        return filtered[0].pose;
    }

    // Weighted average of translations and rotation matrices.
    // Although each camera has a systematic calibration bias, weighted averaging
    // blends these biases and produces smoother results than winner-takes-all,
    // which exposes the full per-camera bias at every switch.
    float totalWeight = 0.0f;
    cv::Mat tFused = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat mSum = cv::Mat::zeros(3, 3, CV_64F);

    for (const auto& c : filtered) {
        double w = static_cast<double>(c.weight);
        totalWeight += c.weight;
        tFused += w * c.pose.translation;
        mSum += w * c.pose.rotation;
    }

    tFused /= static_cast<double>(totalWeight);
    mSum /= static_cast<double>(totalWeight);

    // SVD re-orthogonalization of the averaged rotation matrix
    cv::SVD svd(mSum, cv::SVD::FULL_UV);
    cv::Mat rFused = svd.u * svd.vt;

    // Ensure proper rotation (det = +1, not reflection)
    if (cv::determinant(rFused) < 0) {
        cv::Mat uFixed = svd.u.clone();
        uFixed.col(2) *= -1.0;
        rFused = uFixed * svd.vt;
    }

    Transform result;
    result.rotation = rFused;
    result.translation = tFused;
    result.valid = true;
    return result;
}

// ============================================================================
// EMA Temporal Smoothing for Helmet Pose
// ============================================================================
Transform SmoothPose(const Transform& curr, const Transform& prev, double alpha)
{
    // Blend translations
    cv::Mat tSmooth = alpha * curr.translation + (1.0 - alpha) * prev.translation;

    // Blend rotations via weighted sum + SVD re-orthogonalization
    cv::Mat mBlend = alpha * curr.rotation + (1.0 - alpha) * prev.rotation;
    cv::SVD svd(mBlend, cv::SVD::FULL_UV);
    cv::Mat rSmooth = svd.u * svd.vt;
    if (cv::determinant(rSmooth) < 0) {
        cv::Mat uFixed = svd.u.clone();
        uFixed.col(2) *= -1.0;
        rSmooth = uFixed * svd.vt;
    }

    Transform result;
    result.rotation = rSmooth;
    result.translation = tSmooth;
    result.valid = true;
    return result;
}

// Adaptive per-joint smoothing alpha: core joints get heavy smoothing for stability,
// extremities get light smoothing for responsiveness during fast motion (e.g. boxing).
float GetJointSmoothAlpha(int jointId) {
    switch (jointId) {
        // Core: heavy smoothing (stability anchor)
        case K4ABT_JOINT_PELVIS:
        case K4ABT_JOINT_SPINE_NAVEL:
        case K4ABT_JOINT_SPINE_CHEST:
        case K4ABT_JOINT_NECK:
        case K4ABT_JOINT_CLAVICLE_LEFT:
        case K4ABT_JOINT_CLAVICLE_RIGHT:
        case K4ABT_JOINT_HIP_LEFT:
        case K4ABT_JOINT_HIP_RIGHT:
            return 0.30f;

        // Head/face: moderate smoothing
        case K4ABT_JOINT_HEAD:
        case K4ABT_JOINT_NOSE:
        case K4ABT_JOINT_EYE_LEFT:
        case K4ABT_JOINT_EYE_RIGHT:
        case K4ABT_JOINT_EAR_LEFT:
        case K4ABT_JOINT_EAR_RIGHT:
            return 0.50f;

        // Mid-limb: balanced
        case K4ABT_JOINT_SHOULDER_LEFT:
        case K4ABT_JOINT_SHOULDER_RIGHT:
        case K4ABT_JOINT_ELBOW_LEFT:
        case K4ABT_JOINT_ELBOW_RIGHT:
        case K4ABT_JOINT_KNEE_LEFT:
        case K4ABT_JOINT_KNEE_RIGHT:
        case K4ABT_JOINT_ANKLE_LEFT:
        case K4ABT_JOINT_ANKLE_RIGHT:
            return 0.65f;

        // Extremities: light smoothing (responsiveness for hands/feet)
        case K4ABT_JOINT_WRIST_LEFT:
        case K4ABT_JOINT_WRIST_RIGHT:
        case K4ABT_JOINT_HAND_LEFT:
        case K4ABT_JOINT_HAND_RIGHT:
        case K4ABT_JOINT_HANDTIP_LEFT:
        case K4ABT_JOINT_HANDTIP_RIGHT:
        case K4ABT_JOINT_THUMB_LEFT:
        case K4ABT_JOINT_THUMB_RIGHT:
        case K4ABT_JOINT_FOOT_LEFT:
        case K4ABT_JOINT_FOOT_RIGHT:
            return 0.85f;

        default:
            return 0.50f;
    }
}

// Select the best body from fused bodies for ego-view.
// Filters out phantom bodies (confidence=0), then prioritizes multi-camera bodies
// (matchCount >= 2) over single-camera detections. Among candidates, uses spatial
// continuity (closest to previous pelvis).
int SelectBestBody(const vector<FusedBody>& fusedBodies, const k4a_float3_t& prevPelvis, bool hasPrev)
{
    if (fusedBodies.empty()) return -1;
    if (fusedBodies.size() == 1) return 0;

    // Filter out phantom bodies where pelvis has zero confidence
    vector<int> validBodies;
    for (size_t i = 0; i < fusedBodies.size(); i++) {
        if (fusedBodies[i].joints[K4ABT_JOINT_PELVIS].confidence != K4ABT_JOINT_CONFIDENCE_NONE)
            validBodies.push_back((int)i);
    }
    // Fall back to unfiltered list only if ALL bodies have zero confidence
    if (validBodies.empty()) {
        for (size_t i = 0; i < fusedBodies.size(); i++)
            validBodies.push_back((int)i);
    }

    if (validBodies.size() == 1) return validBodies[0];

    // Separate multi-camera and single-camera bodies
    vector<int> multiCam, singleCam;
    for (int i : validBodies) {
        if (fusedBodies[i].matchCount >= 2)
            multiCam.push_back(i);
        else
            singleCam.push_back(i);
    }

    // Prefer multi-camera bodies; fall back to single-camera only if none exist
    const vector<int>& candidates = multiCam.empty() ? singleCam : multiCam;

    if (candidates.size() == 1) return candidates[0];

    if (hasPrev) {
        float minDist = numeric_limits<float>::max();
        int bestIdx = candidates[0];
        for (int idx : candidates) {
            float dx = fusedBodies[idx].joints[K4ABT_JOINT_PELVIS].position.xyz.x - prevPelvis.xyz.x;
            float dy = fusedBodies[idx].joints[K4ABT_JOINT_PELVIS].position.xyz.y - prevPelvis.xyz.y;
            float dz = fusedBodies[idx].joints[K4ABT_JOINT_PELVIS].position.xyz.z - prevPelvis.xyz.z;
            float dist = sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = idx;
            }
        }
        return bestIdx;
    }

    // No previous: use first multi-camera body
    return candidates[0];
}

// Transform fused skeleton joints from world to helmet camera frame
vector<Joint3D> TransformSkeletonToCamera(const vector<FusedBody>& fusedBodies,
                                           const Transform& helmetPose,
                                           int bodyIdx = 0)
{
    vector<Joint3D> result;
    if (fusedBodies.empty()) return result;

    const FusedBody& body = fusedBodies[min(bodyIdx, (int)fusedBodies.size() - 1)];

    cv::Mat R_inv = helmetPose.rotation.t();

    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        cv::Mat p_world = (cv::Mat_<double>(3, 1) <<
            (double)body.joints[j].position.xyz.x,
            (double)body.joints[j].position.xyz.y,
            (double)body.joints[j].position.xyz.z);

        cv::Mat p_offset = p_world - helmetPose.translation;
        cv::Mat p_cam = R_inv * p_offset;

        Joint3D jt;
        jt.x = (float)p_cam.at<double>(0);
        jt.y = (float)p_cam.at<double>(1);
        jt.z = (float)p_cam.at<double>(2);
        jt.confidence = (int)body.joints[j].confidence;

        auto it = g_jointNames.find(static_cast<k4abt_joint_id_t>(j));
        jt.name = (it != g_jointNames.end()) ? it->second : "UNKNOWN";

        result.push_back(jt);
    }

    return result;
}

// Transform a single body from camera space to helmet-local space.
// Each camera independently computes its helmet pose from checkerboard detection,
// so this uses only camera-local measurements (no world-frame extrinsics).
void TransformBodyToHelmetLocal(const k4abt_body_t& body_cam,
                                 const Transform& helmetPose_cam,
                                 Joint3D* joints_out)
{
    cv::Mat R_inv = helmetPose_cam.rotation.t();
    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        cv::Mat p_cam = (cv::Mat_<double>(3, 1) <<
            (double)body_cam.skeleton.joints[j].position.xyz.x,
            (double)body_cam.skeleton.joints[j].position.xyz.y,
            (double)body_cam.skeleton.joints[j].position.xyz.z);
        cv::Mat p_helmet = R_inv * (p_cam - helmetPose_cam.translation);
        joints_out[j].x = (float)p_helmet.at<double>(0);
        joints_out[j].y = (float)p_helmet.at<double>(1);
        joints_out[j].z = (float)p_helmet.at<double>(2);
        joints_out[j].confidence = (int)body_cam.skeleton.joints[j].confidence_level;
        auto it = g_jointNames.find(static_cast<k4abt_joint_id_t>(j));
        joints_out[j].name = (it != g_jointNames.end()) ? it->second : "UNKNOWN";
    }
}

// Confidence-weighted average of helmet-local skeletons from multiple cameras.
// Same fusion logic as FuseBodiesAtTimestamp() but operates in helmet-local coords.
vector<Joint3D> FuseHelmetLocalSkeletons(const vector<CameraHelmetSkeleton>& skeletons)
{
    vector<Joint3D> result(K4ABT_JOINT_COUNT);

    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        float totalWeight = 0.0f;
        float avgX = 0, avgY = 0, avgZ = 0;
        int maxConf = 0;

        for (const auto& skel : skeletons) {
            float confWeight = ConfidenceToWeight(
                static_cast<k4abt_joint_confidence_level_t>(skel.joints[j].confidence));
            if (confWeight <= 0.0f) continue;

            float w = skel.weight * confWeight;
            avgX += skel.joints[j].x * w;
            avgY += skel.joints[j].y * w;
            avgZ += skel.joints[j].z * w;
            totalWeight += w;

            if (skel.joints[j].confidence > maxConf)
                maxConf = skel.joints[j].confidence;
        }

        auto it = g_jointNames.find(static_cast<k4abt_joint_id_t>(j));
        result[j].name = (it != g_jointNames.end()) ? it->second : "UNKNOWN";

        if (totalWeight > 0) {
            result[j].x = avgX / totalWeight;
            result[j].y = avgY / totalWeight;
            result[j].z = avgZ / totalWeight;
            result[j].confidence = maxConf;
        } else {
            result[j].x = 0;
            result[j].y = 0;
            result[j].z = 0;
            result[j].confidence = 0;
        }
    }

    return result;
}

// Select the best body from camera-local bodies using pelvis continuity.
// Analogous to SelectBestBody() but operates on raw k4abt_body_t in camera space.
int SelectBestBodyLocal(const vector<k4abt_body_t>& bodies,
                         const k4a_float3_t& prevPelvis, bool hasPrev)
{
    if (bodies.empty()) return -1;
    if (bodies.size() == 1) return 0;

    // Filter out phantom bodies where pelvis has zero confidence
    vector<int> validBodies;
    for (size_t i = 0; i < bodies.size(); i++) {
        if (bodies[i].skeleton.joints[K4ABT_JOINT_PELVIS].confidence_level != K4ABT_JOINT_CONFIDENCE_NONE)
            validBodies.push_back((int)i);
    }
    if (validBodies.empty()) {
        for (size_t i = 0; i < bodies.size(); i++)
            validBodies.push_back((int)i);
    }

    if (validBodies.size() == 1) return validBodies[0];

    if (hasPrev) {
        float minDist = numeric_limits<float>::max();
        int bestIdx = validBodies[0];
        for (int idx : validBodies) {
            float dx = bodies[idx].skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.x - prevPelvis.xyz.x;
            float dy = bodies[idx].skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.y - prevPelvis.xyz.y;
            float dz = bodies[idx].skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.z - prevPelvis.xyz.z;
            float dist = sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = idx;
            }
        }
        return bestIdx;
    }

    return validBodies[0];
}

// Project 3D joints (in helmet camera frame) to 2D image coordinates
vector<Joint2D> ProjectSkeleton(const vector<Joint3D>& skeleton3D,
                                 const k4a_calibration_t& calibration)
{
    vector<Joint2D> result(skeleton3D.size());

    int width = calibration.color_camera_calibration.resolution_width;
    int height = calibration.color_camera_calibration.resolution_height;

    for (size_t i = 0; i < skeleton3D.size(); i++) {
        const auto& joint = skeleton3D[i];
        result[i].name = joint.name;
        result[i].confidence = joint.confidence;

        // Reject points behind or too close to the camera — near-zero depth
        // causes extreme values from the lens distortion model
        if (joint.z < 50.0f) {
            result[i].u = 0;
            result[i].v = 0;
            result[i].visible = false;
            continue;
        }

        k4a_float3_t point3d = { joint.x, joint.y, joint.z };
        k4a_float2_t point2d;
        int valid = 0;

        k4a_calibration_3d_to_2d(&calibration, &point3d,
            K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR,
            &point2d, &valid);

        if (valid) {
            result[i].u = point2d.xy.x;
            result[i].v = point2d.xy.y;
            result[i].visible = (point2d.xy.x >= 0 && point2d.xy.x < width &&
                                  point2d.xy.y >= 0 && point2d.xy.y < height);
            // Clamp off-screen projections to avoid extreme distorted values in JSON
            if (!result[i].visible) {
                result[i].u = max(-1.0f * width, min(result[i].u, 2.0f * width));
                result[i].v = max(-1.0f * height, min(result[i].v, 2.0f * height));
            }
        } else {
            result[i].u = 0;
            result[i].v = 0;
            result[i].visible = false;
        }
    }

    return result;
}

// ============================================================================
// Ego-View: Output Writers
// ============================================================================
void WriteEgoFrameJson(const string& path, int frameId, uint64_t timestamp,
                       bool checkerboardDetected, int detectionCamera,
                       const Transform& helmetPose,
                       const vector<Joint3D>& joints3D,
                       const vector<Joint2D>& joints2D,
                       const string& imageFile,
                       int numBodies,
                       const k4a_calibration_t* calibration = nullptr)
{
    json j;
    j["frame_id"] = frameId;
    j["timestamp_usec"] = timestamp;
    j["image_file"] = imageFile;
    j["checkerboard_detected"] = checkerboardDetected;
    j["detection_camera"] = detectionCamera;
    j["num_bodies"] = numBodies;

    // Write camera intrinsics (pinhole model from K4A calibration)
    if (calibration) {
        const auto& params = calibration->color_camera_calibration.intrinsics.parameters.param;
        j["camera_intrinsics"] = {
            {"fx", params.fx},
            {"fy", params.fy},
            {"cx", params.cx},
            {"cy", params.cy},
            {"width", calibration->color_camera_calibration.resolution_width},
            {"height", calibration->color_camera_calibration.resolution_height}
        };
    }

    if (helmetPose.valid) {
        j["camera_pose"]["R"] = json::array();
        for (int r = 0; r < 3; r++) {
            j["camera_pose"]["R"].push_back({
                helmetPose.rotation.at<double>(r, 0),
                helmetPose.rotation.at<double>(r, 1),
                helmetPose.rotation.at<double>(r, 2)
            });
        }
        j["camera_pose"]["t"] = {
            helmetPose.translation.at<double>(0),
            helmetPose.translation.at<double>(1),
            helmetPose.translation.at<double>(2)
        };
    }

    j["skeleton_3d"] = json::array();
    for (size_t i = 0; i < joints3D.size(); i++) {
        j["skeleton_3d"].push_back({
            {"joint_id", i},
            {"name", joints3D[i].name},
            {"x", joints3D[i].x},
            {"y", joints3D[i].y},
            {"z", joints3D[i].z},
            {"confidence", joints3D[i].confidence}
        });
    }

    j["skeleton_2d"] = json::array();
    for (size_t i = 0; i < joints2D.size(); i++) {
        j["skeleton_2d"].push_back({
            {"joint_id", i},
            {"name", joints2D[i].name},
            {"u", joints2D[i].u},
            {"v", joints2D[i].v},
            {"confidence", joints2D[i].confidence},
            {"visible", joints2D[i].visible}
        });
    }

    ofstream file(path);
    file << setw(2) << j << endl;
}

// ============================================================================
// Body Matching and Fusion
// ============================================================================
float CalculateDistance(const k4a_float3_t& p1, const k4a_float3_t& p2)
{
    float dx = p1.xyz.x - p2.xyz.x;
    float dy = p1.xyz.y - p2.xyz.y;
    float dz = p1.xyz.z - p2.xyz.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

float ConfidenceToWeight(k4abt_joint_confidence_level_t conf)
{
    switch (conf) {
        case K4ABT_JOINT_CONFIDENCE_NONE:   return 0.0f;
        case K4ABT_JOINT_CONFIDENCE_LOW:    return 0.1f;
        case K4ABT_JOINT_CONFIDENCE_MEDIUM: return 0.3f;
        case K4ABT_JOINT_CONFIDENCE_HIGH:   return 1.0f;
        default: return 0.0f;
    }
}

// ============================================================================
// Per-Camera Interpolation (15 FPS → 30 FPS midpoint synthesis)
// ============================================================================
float QuaternionDot(const k4a_quaternion_t& a, const k4a_quaternion_t& b)
{
    return a.wxyz.w * b.wxyz.w + a.wxyz.x * b.wxyz.x +
           a.wxyz.y * b.wxyz.y + a.wxyz.z * b.wxyz.z;
}

k4a_quaternion_t QuaternionSlerp(const k4a_quaternion_t& a, const k4a_quaternion_t& b, float t)
{
    k4a_quaternion_t bAdj = b;
    float dot = QuaternionDot(a, b);

    // Ensure shortest path
    if (dot < 0.0f) {
        bAdj.wxyz.w = -b.wxyz.w;
        bAdj.wxyz.x = -b.wxyz.x;
        bAdj.wxyz.y = -b.wxyz.y;
        bAdj.wxyz.z = -b.wxyz.z;
        dot = -dot;
    }

    k4a_quaternion_t result;
    if (dot > 0.9995f) {
        // Near-parallel: fallback to LERP + normalize
        result.wxyz.w = a.wxyz.w + t * (bAdj.wxyz.w - a.wxyz.w);
        result.wxyz.x = a.wxyz.x + t * (bAdj.wxyz.x - a.wxyz.x);
        result.wxyz.y = a.wxyz.y + t * (bAdj.wxyz.y - a.wxyz.y);
        result.wxyz.z = a.wxyz.z + t * (bAdj.wxyz.z - a.wxyz.z);
    } else {
        float theta = acosf(dot);
        float sinTheta = sinf(theta);
        float wA = sinf((1.0f - t) * theta) / sinTheta;
        float wB = sinf(t * theta) / sinTheta;
        result.wxyz.w = wA * a.wxyz.w + wB * bAdj.wxyz.w;
        result.wxyz.x = wA * a.wxyz.x + wB * bAdj.wxyz.x;
        result.wxyz.y = wA * a.wxyz.y + wB * bAdj.wxyz.y;
        result.wxyz.z = wA * a.wxyz.z + wB * bAdj.wxyz.z;
    }

    // Normalize
    float len = sqrtf(result.wxyz.w * result.wxyz.w + result.wxyz.x * result.wxyz.x +
                      result.wxyz.y * result.wxyz.y + result.wxyz.z * result.wxyz.z);
    if (len > 1e-6f) {
        result.wxyz.w /= len;
        result.wxyz.x /= len;
        result.wxyz.y /= len;
        result.wxyz.z /= len;
    }
    return result;
}

vector<k4abt_body_t> InterpolateBodies(const vector<k4abt_body_t>& bodiesA,
                                        const vector<k4abt_body_t>& bodiesB, float t)
{
    vector<k4abt_body_t> result;

    // Track which bodies in B have been matched
    vector<bool> matchedB(bodiesB.size(), false);

    for (const auto& bodyA : bodiesA) {
        // Find matching body in B by ID
        int matchIdx = -1;
        for (size_t i = 0; i < bodiesB.size(); i++) {
            if (bodiesB[i].id == bodyA.id) {
                matchIdx = (int)i;
                break;
            }
        }

        k4abt_body_t interp;
        interp.id = bodyA.id;

        if (matchIdx >= 0) {
            matchedB[matchIdx] = true;
            const auto& bodyB = bodiesB[matchIdx];

            for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                auto confA = bodyA.skeleton.joints[j].confidence_level;
                auto confB = bodyB.skeleton.joints[j].confidence_level;
                bool validA = (confA != K4ABT_JOINT_CONFIDENCE_NONE);
                bool validB = (confB != K4ABT_JOINT_CONFIDENCE_NONE);

                if (validA && validB) {
                    // LERP position
                    interp.skeleton.joints[j].position.xyz.x =
                        bodyA.skeleton.joints[j].position.xyz.x * (1.0f - t) +
                        bodyB.skeleton.joints[j].position.xyz.x * t;
                    interp.skeleton.joints[j].position.xyz.y =
                        bodyA.skeleton.joints[j].position.xyz.y * (1.0f - t) +
                        bodyB.skeleton.joints[j].position.xyz.y * t;
                    interp.skeleton.joints[j].position.xyz.z =
                        bodyA.skeleton.joints[j].position.xyz.z * (1.0f - t) +
                        bodyB.skeleton.joints[j].position.xyz.z * t;
                    // SLERP orientation
                    interp.skeleton.joints[j].orientation =
                        QuaternionSlerp(bodyA.skeleton.joints[j].orientation,
                                        bodyB.skeleton.joints[j].orientation, t);
                    // Min confidence
                    interp.skeleton.joints[j].confidence_level =
                        (confA < confB) ? confA : confB;
                } else if (validA) {
                    interp.skeleton.joints[j] = bodyA.skeleton.joints[j];
                } else if (validB) {
                    interp.skeleton.joints[j] = bodyB.skeleton.joints[j];
                } else {
                    interp.skeleton.joints[j] = bodyA.skeleton.joints[j]; // both NONE
                }
            }
        } else {
            // Unmatched body from A (exiting): pass through as-is
            interp = bodyA;
        }
        result.push_back(interp);
    }

    // Unmatched bodies from B (entering): include as-is
    for (size_t i = 0; i < bodiesB.size(); i++) {
        if (!matchedB[i]) {
            result.push_back(bodiesB[i]);
        }
    }

    return result;
}

// Transform bodies from camera space to primary camera space
vector<k4abt_body_t> TransformBodies(const vector<k4abt_body_t>& bodies, int deviceIndex)
{
    vector<k4abt_body_t> transformed;

    const CameraExtrinsics* ext = nullptr;
    for (const auto& cam : g_calibration.cameras) {
        if (cam.deviceIndex == deviceIndex) {
            ext = &cam;
            break;
        }
    }

    for (const auto& body : bodies) {
        k4abt_body_t tb;
        tb.id = body.id;

        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
            if (ext && ext->isValid) {
                tb.skeleton.joints[j].position = TransformPoint(body.skeleton.joints[j].position, *ext);
                tb.skeleton.joints[j].orientation = TransformOrientation(body.skeleton.joints[j].orientation, *ext);
            } else {
                tb.skeleton.joints[j].position = body.skeleton.joints[j].position;
                tb.skeleton.joints[j].orientation = body.skeleton.joints[j].orientation;
            }
            tb.skeleton.joints[j].confidence_level = body.skeleton.joints[j].confidence_level;
        }

        transformed.push_back(tb);
    }

    return transformed;
}

// Fuse bodies from multiple cameras at same timestamp
vector<FusedBody> FuseBodiesAtTimestamp(const vector<FrameData>& frames)
{
    vector<FusedBody> fused;
    if (frames.empty()) return fused;

    // Transform all bodies to primary space
    vector<vector<k4abt_body_t>> transformedPerCamera(frames.size());
    for (size_t i = 0; i < frames.size(); i++) {
        transformedPerCamera[i] = TransformBodies(frames[i].bodies, frames[i].deviceIndex);
    }

    // Track which bodies have been matched
    vector<vector<bool>> used(frames.size());
    for (size_t i = 0; i < frames.size(); i++) {
        used[i].resize(transformedPerCamera[i].size(), false);
    }

    // Find anchor camera (first with bodies)
    int anchorCam = -1;
    for (size_t i = 0; i < frames.size(); i++) {
        if (!transformedPerCamera[i].empty()) {
            anchorCam = (int)i;
            break;
        }
    }

    if (anchorCam < 0) return fused;

    // Match bodies starting from anchor camera
    uint32_t fusedId = 0;
    for (size_t bi = 0; bi < transformedPerCamera[anchorCam].size(); bi++) {
        FusedBody fb;
        fb.id = fusedId++;

        // Collect matching bodies from all cameras
        vector<pair<int, int>> matches; // (cameraIdx, bodyIdx)
        matches.push_back({anchorCam, (int)bi});
        used[anchorCam][bi] = true;

        const k4a_float3_t& anchorPelvis =
            transformedPerCamera[anchorCam][bi].skeleton.joints[K4ABT_JOINT_PELVIS].position;

        for (size_t ci = 0; ci < frames.size(); ci++) {
            if ((int)ci == anchorCam) continue;

            float minDist = BODY_MATCH_THRESHOLD_MM;
            int bestMatch = -1;

            for (size_t bj = 0; bj < transformedPerCamera[ci].size(); bj++) {
                if (used[ci][bj]) continue;

                const k4a_float3_t& pelvis =
                    transformedPerCamera[ci][bj].skeleton.joints[K4ABT_JOINT_PELVIS].position;

                float dist = CalculateDistance(anchorPelvis, pelvis);
                if (dist < minDist) {
                    minDist = dist;
                    bestMatch = (int)bj;
                }
            }

            if (bestMatch >= 0) {
                matches.push_back({(int)ci, bestMatch});
                used[ci][bestMatch] = true;
            }
        }

        fb.matchCount = (int)matches.size();

        // Fuse joints using weighted average
        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
            float totalWeight = 0.0f;
            k4a_float3_t avgPos = {0, 0, 0};
            k4a_quaternion_t bestOri = {1, 0, 0, 0};
            float bestOriWeight = 0.0f;
            k4abt_joint_confidence_level_t maxConf = K4ABT_JOINT_CONFIDENCE_NONE;
            int bestCamera = -1;

            for (const auto& m : matches) {
                const auto& joint = transformedPerCamera[m.first][m.second].skeleton.joints[j];
                float w = ConfidenceToWeight(joint.confidence_level);

                if (w > 0) {
                    avgPos.xyz.x += joint.position.xyz.x * w;
                    avgPos.xyz.y += joint.position.xyz.y * w;
                    avgPos.xyz.z += joint.position.xyz.z * w;
                    totalWeight += w;

                    if (w > bestOriWeight) {
                        bestOriWeight = w;
                        bestOri = joint.orientation;
                        bestCamera = m.first;
                    }
                    if (joint.confidence_level > maxConf) {
                        maxConf = joint.confidence_level;
                    }
                }
            }

            if (totalWeight > 0) {
                fb.joints[j].position.xyz.x = avgPos.xyz.x / totalWeight;
                fb.joints[j].position.xyz.y = avgPos.xyz.y / totalWeight;
                fb.joints[j].position.xyz.z = avgPos.xyz.z / totalWeight;
                fb.joints[j].orientation = bestOri;
                fb.joints[j].confidence = maxConf;
                fb.joints[j].sourceDeviceIndex = bestCamera;
            } else {
                fb.joints[j].position = {0, 0, 0};
                fb.joints[j].orientation = {1, 0, 0, 0};
                fb.joints[j].confidence = K4ABT_JOINT_CONFIDENCE_NONE;
                fb.joints[j].sourceDeviceIndex = -1;
            }
        }

        fused.push_back(fb);
    }

    // Add unmatched bodies from other cameras
    for (size_t ci = 0; ci < frames.size(); ci++) {
        for (size_t bi = 0; bi < transformedPerCamera[ci].size(); bi++) {
            if (used[ci][bi]) continue;

            FusedBody fb;
            fb.id = fusedId++;
            fb.matchCount = 1;  // Single-camera detection only

            const auto& body = transformedPerCamera[ci][bi];
            for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                fb.joints[j].position = body.skeleton.joints[j].position;
                fb.joints[j].orientation = body.skeleton.joints[j].orientation;
                fb.joints[j].confidence = body.skeleton.joints[j].confidence_level;
                fb.joints[j].sourceDeviceIndex = (int)ci;
            }

            fused.push_back(fb);
        }
    }

    return fused;
}

// ============================================================================
// MKV Processing
// ============================================================================
struct MkvProcessor {
    k4a_playback_t playback = nullptr;
    k4abt_tracker_t tracker = nullptr;
    int deviceIndex = 0;
    string serialNumber;
    string filepath;
    bool isEOF = false;
    uint64_t lastTimestamp = 0;
    vector<k4abt_body_t> lastBodies;

    // Previous frame for interpolation (15 FPS → 30 FPS midpoint synthesis)
    vector<k4abt_body_t> prevBodies;
    uint64_t prevTimestamp = 0;
    bool hasPrevFrame = false;

    // Ego-view fields
    bool isHelmet = false;
    k4a_calibration_t calibration = {};
    k4a_transformation_t transformation = nullptr;
    cv::Mat lastColorImage;
    cv::Mat lastDepthImage;
    bool hasNewFrame = false;

    // Per-camera temporal smoothing (local-frame mode)
    k4abt_body_t prevSmoothedBodyLocal = {};
    bool hasPrevSmoothedLocal = false;
    uint64_t prevSmoothedTimestampLocal = 0;
    k4a_float3_t prevPelvisLocal = {0, 0, 0};
    bool hasPrevPelvisLocal = false;
    int jointCarryCountLocal[K4ABT_JOINT_COUNT] = {};
    int bodyCarryCountLocal = 0;
    int stableTrackCountLocal = 0;
};

bool InitMkvProcessor(MkvProcessor& proc, const string& filepath, int deviceIndex,
                       k4abt_tracker_configuration_t trackerConfig, float smoothingFactor = 0.0f)
{
    proc.filepath = filepath;
    proc.deviceIndex = deviceIndex;

    // Open playback
    k4a_result_t result = k4a_playback_open(filepath.c_str(), &proc.playback);
    if (result != K4A_RESULT_SUCCEEDED) {
        cerr << "Failed to open: " << filepath << endl;
        return false;
    }

    // Get calibration (always stored for ego-view)
    result = k4a_playback_get_calibration(proc.playback, &proc.calibration);
    if (result != K4A_RESULT_SUCCEEDED) {
        cerr << "Failed to get calibration from: " << filepath << endl;
        k4a_playback_close(proc.playback);
        return false;
    }

    // Try to get serial number from recording tag
    char serial[64] = {0};
    size_t serialSize = sizeof(serial);
    if (k4a_playback_get_tag(proc.playback, "K4A_DEVICE_SERIAL_NUMBER", serial, &serialSize) == K4A_BUFFER_RESULT_SUCCEEDED) {
        proc.serialNumber = serial;
    } else {
        proc.serialNumber = "unknown_" + to_string(deviceIndex);
    }

    if (proc.isHelmet) {
        // Helmet camera: no body tracker, no depth→color transformation needed
        cout << "Initialized HELMET processor for device " << deviceIndex
             << " (SN: " << proc.serialNumber << "): " << filepath << endl;
    } else {
        // Fixed camera: create body tracker and depth→color transformation
        result = k4abt_tracker_create(&proc.calibration, trackerConfig, &proc.tracker);
        if (result != K4A_RESULT_SUCCEEDED) {
            cerr << "Failed to create tracker for: " << filepath << endl;
            k4a_playback_close(proc.playback);
            return false;
        }
        k4abt_tracker_set_temporal_smoothing(proc.tracker, smoothingFactor);

        // Create transformation handle for depth→color (needed for ego-view checkerboard detection)
        proc.transformation = k4a_transformation_create(&proc.calibration);

        cout << "Initialized processor for device " << deviceIndex
             << " (SN: " << proc.serialNumber << "): " << filepath << endl;
    }

    return true;
}

bool ProcessNextFrame(MkvProcessor& proc)
{
    if (proc.isEOF) return false;

    proc.hasNewFrame = false;

    k4a_capture_t capture = nullptr;
    k4a_stream_result_t streamResult = k4a_playback_get_next_capture(proc.playback, &capture);

    if (streamResult == K4A_STREAM_RESULT_EOF) {
        proc.isEOF = true;
        return false;
    }

    if (streamResult != K4A_STREAM_RESULT_SUCCEEDED) {
        cerr << "Stream error on device " << proc.deviceIndex << endl;
        proc.isEOF = true;
        return false;
    }

    if (proc.isHelmet) {
        // Helmet camera: extract color image only (no body tracking)
        k4a_image_t color = k4a_capture_get_color_image(capture);
        if (color) {
            proc.lastTimestamp = k4a_image_get_device_timestamp_usec(color);

            int width = k4a_image_get_width_pixels(color);
            int height = k4a_image_get_height_pixels(color);
            k4a_image_format_t format = k4a_image_get_format(color);

            if (format == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
                proc.lastColorImage = cv::Mat(height, width, CV_8UC4,
                    k4a_image_get_buffer(color)).clone();
            } else if (format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
                vector<uint8_t> buffer(k4a_image_get_buffer(color),
                    k4a_image_get_buffer(color) + k4a_image_get_size(color));
                proc.lastColorImage = cv::imdecode(buffer, cv::IMREAD_COLOR);
            }

            k4a_image_release(color);
            proc.hasNewFrame = true;
        }

        k4a_capture_release(capture);
        return proc.hasNewFrame;
    }

    // Fixed camera: body tracking + image extraction
    k4a_image_t depth = k4a_capture_get_depth_image(capture);
    if (depth == nullptr) {
        k4a_capture_release(capture);
        return true; // Skip frame, but continue processing
    }

    // Extract color and depth images for ego-view checkerboard detection
    k4a_image_t color = k4a_capture_get_color_image(capture);
    if (color) {
        int width = k4a_image_get_width_pixels(color);
        int height = k4a_image_get_height_pixels(color);
        k4a_image_format_t format = k4a_image_get_format(color);

        if (format == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
            proc.lastColorImage = cv::Mat(height, width, CV_8UC4,
                k4a_image_get_buffer(color)).clone();
        } else if (format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
            vector<uint8_t> buffer(k4a_image_get_buffer(color),
                k4a_image_get_buffer(color) + k4a_image_get_size(color));
            proc.lastColorImage = cv::imdecode(buffer, cv::IMREAD_COLOR);
        }
        k4a_image_release(color);
    }

    {
        int dw = k4a_image_get_width_pixels(depth);
        int dh = k4a_image_get_height_pixels(depth);
        proc.lastDepthImage = cv::Mat(dh, dw, CV_16UC1,
            k4a_image_get_buffer(depth)).clone();
    }

    // Enqueue capture for body tracking
    k4a_wait_result_t queueResult = k4abt_tracker_enqueue_capture(proc.tracker, capture, K4A_WAIT_INFINITE);
    k4a_image_release(depth);
    k4a_capture_release(capture);

    if (queueResult != K4A_WAIT_RESULT_SUCCEEDED) {
        cerr << "Failed to enqueue capture for device " << proc.deviceIndex << endl;
        return false;
    }

    // Pop result
    k4abt_frame_t bodyFrame = nullptr;
    k4a_wait_result_t popResult = k4abt_tracker_pop_result(proc.tracker, &bodyFrame, K4A_WAIT_INFINITE);

    if (popResult != K4A_WAIT_RESULT_SUCCEEDED) {
        cerr << "Failed to pop body frame for device " << proc.deviceIndex << endl;
        return false;
    }

    // Save previous frame for interpolation (sliding [prev, last] window)
    if (!proc.lastBodies.empty()) {
        proc.prevBodies = proc.lastBodies;
        proc.prevTimestamp = proc.lastTimestamp;
        proc.hasPrevFrame = true;
    }

    // Extract data
    proc.lastTimestamp = k4abt_frame_get_device_timestamp_usec(bodyFrame);
    proc.lastBodies.clear();

    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    for (uint32_t i = 0; i < numBodies; i++) {
        k4abt_body_t body;
        if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) == K4A_RESULT_SUCCEEDED) {
            body.id = k4abt_frame_get_body_id(bodyFrame, i);

            // Body tracker outputs joints in DEPTH camera space, but extrinsic
            // calibration and checkerboard detection operate in COLOR camera space.
            // Convert each joint from depth→color to eliminate the depth-to-color
            // baseline offset (~25-30mm) that would otherwise cause sideways shift.
            for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                k4a_float3_t colorPos;
                if (k4a_calibration_3d_to_3d(&proc.calibration,
                        &body.skeleton.joints[j].position,
                        K4A_CALIBRATION_TYPE_DEPTH,
                        K4A_CALIBRATION_TYPE_COLOR,
                        &colorPos) == K4A_RESULT_SUCCEEDED) {
                    body.skeleton.joints[j].position = colorPos;
                }
            }

            proc.lastBodies.push_back(body);
        }
    }

    k4abt_frame_release(bodyFrame);
    proc.hasNewFrame = true;
    return true;
}

void CloseMkvProcessor(MkvProcessor& proc)
{
    if (proc.tracker) {
        k4abt_tracker_shutdown(proc.tracker);
        k4abt_tracker_destroy(proc.tracker);
        proc.tracker = nullptr;
    }
    if (proc.transformation) {
        k4a_transformation_destroy(proc.transformation);
        proc.transformation = nullptr;
    }
    if (proc.playback) {
        k4a_playback_close(proc.playback);
        proc.playback = nullptr;
    }
}

// ============================================================================
// CSV Output
// ============================================================================
void WriteCSVHeader(ofstream& file)
{
    file << "timestamp_usec,body_id";
    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        file << ",J" << j << "_x,J" << j << "_y,J" << j << "_z,J" << j << "_conf";
    }
    file << "\n";
}

void WriteFusedBodyToCSV(ofstream& file, uint64_t timestamp, const FusedBody& body)
{
    file << timestamp << "," << body.id;
    for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
        file << fixed << setprecision(3)
             << "," << body.joints[j].position.xyz.x
             << "," << body.joints[j].position.xyz.y
             << "," << body.joints[j].position.xyz.z
             << "," << (int)body.joints[j].confidence;
    }
    file << "\n";
}

// ============================================================================
// Main Processing
// ============================================================================
void PrintUsage()
{
    cout << "\n=== Multi-Device Offline Body Tracking Processor ===\n"
         << "Processes MKV recordings from multiple cameras with skeleton fusion.\n"
         << "Supports ego-view processing with helmet-mounted camera.\n\n"
         << "USAGE:\n"
         << "  multi_device_offline_processor.exe [OPTIONS] <mkv1> <mkv2> ...\n\n"
         << "OPTIONS:\n"
         << "  --calibration FILE       - Calibration JSON file (required for fusion)\n"
         << "  --output FILE            - Output CSV file (default: output.csv)\n"
         << "  --mode MODE              - Processing mode: CPU, CUDA, DirectML (default), TensorRT\n"
         << "  --sync-threshold MS      - Max timestamp difference for sync (default: 33ms)\n"
         << "  --help                   - Show this help\n\n"
         << "EGO-VIEW OPTIONS:\n"
         << "  --helmet-serial SERIAL   - Serial number of helmet camera MKV\n"
         << "  --t-checker-to-a PATH    - Path to T_checker_to_A.json\n"
         << "  --helmet-cb-rows N       - Checkerboard inner rows (default: 4)\n"
         << "  --helmet-cb-cols N       - Checkerboard inner cols (default: 5)\n"
         << "  --helmet-cb-square N     - Square size in mm (default: 30)\n"
         << "  --ego-output DIR         - Output directory for ego-view data (default: ego_output/)\n"
         << "  --ego-fusion-mode MODE   - Ego skeleton fusion: world (default), local\n"
         << "                             'local' fuses in helmet-local coords (avoids extrinsic errors)\n\n"
         << "BODY TRACKING:\n"
         << "  --sensor-orientation ORI - Sensor orientation: default, cw90, ccw90, flip180\n"
         << "  --smoothing FACTOR       - Temporal smoothing factor 0.0-1.0 (default: 0.0)\n"
         << "  --max-body-distance MM   - Ignore bodies with pelvis farther than this (default: 4000mm)\n\n"
         << "EXAMPLE (standard):\n"
         << "  multi_device_offline_processor.exe --calibration calib.json \\\n"
         << "      --output skeleton.csv recording_cam0.mkv recording_cam1.mkv\n\n"
         << "EXAMPLE (ego-view):\n"
         << "  multi_device_offline_processor.exe --calibration calib.json \\\n"
         << "      --helmet-serial CL3FC3100HN --t-checker-to-a T_checker_to_A.json \\\n"
         << "      --ego-output ego_dataset/ \\\n"
         << "      cam0.mkv cam1.mkv cam2.mkv helmet.mkv\n"
         << endl;
}

int main(int argc, char** argv)
{
    cout << "\n========================================" << endl;
    cout << "Multi-Device Offline Processor" << endl;
    cout << "========================================\n" << endl;

    // Parse arguments
    string calibrationPath;
    string outputPath = "output.csv";
    vector<string> mkvPaths;
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
    k4abt_sensor_orientation_t sensorOrientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
    float smoothingFactor = 0.0f;
    uint64_t syncThresholdUs = 33000; // 33ms default

    // Ego-view arguments
    string helmetSerial;
    string tCheckerToAPath;
    string egoOutputDir = "ego_output";
    HelmetCBConfig helmetCBConfig;
    string egoFusionMode = "world";  // "world" (default) or "local"
    float maxBodyDistanceMm = 4000.0f;  // ignore bodies farther than this from origin

    for (int i = 1; i < argc; i++) {
        string arg(argv[i]);

        if (arg == "--calibration" && i + 1 < argc) {
            calibrationPath = argv[++i];
        }
        else if (arg == "--output" && i + 1 < argc) {
            outputPath = argv[++i];
        }
        else if (arg == "--mode" && i + 1 < argc) {
            string mode(argv[++i]);
            if (mode == "CPU") processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
            else if (mode == "CUDA") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
            else if (mode == "DirectML") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
            else if (mode == "TensorRT") processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT;
        }
        else if (arg == "--sync-threshold" && i + 1 < argc) {
            syncThresholdUs = (uint64_t)(stod(argv[++i]) * 1000);
        }
        else if (arg == "--helmet-serial" && i + 1 < argc) {
            helmetSerial = argv[++i];
        }
        else if (arg == "--t-checker-to-a" && i + 1 < argc) {
            tCheckerToAPath = argv[++i];
        }
        else if (arg == "--helmet-cb-rows" && i + 1 < argc) {
            helmetCBConfig.rows = stoi(argv[++i]);
        }
        else if (arg == "--helmet-cb-cols" && i + 1 < argc) {
            helmetCBConfig.cols = stoi(argv[++i]);
        }
        else if (arg == "--helmet-cb-square" && i + 1 < argc) {
            helmetCBConfig.squareMm = stof(argv[++i]);
        }
        else if (arg == "--ego-output" && i + 1 < argc) {
            egoOutputDir = argv[++i];
        }
        else if (arg == "--ego-fusion-mode" && i + 1 < argc) {
            egoFusionMode = argv[++i];
            if (egoFusionMode != "world" && egoFusionMode != "local") {
                cerr << "Unknown ego-fusion-mode: " << egoFusionMode
                     << " (use 'world' or 'local')" << endl;
                return -1;
            }
        }
        else if (arg == "--sensor-orientation" && i + 1 < argc) {
            string ori(argv[++i]);
            if (ori == "default") sensorOrientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
            else if (ori == "cw90") sensorOrientation = K4ABT_SENSOR_ORIENTATION_CLOCKWISE90;
            else if (ori == "ccw90") sensorOrientation = K4ABT_SENSOR_ORIENTATION_COUNTERCLOCKWISE90;
            else if (ori == "flip180") sensorOrientation = K4ABT_SENSOR_ORIENTATION_FLIP180;
            else {
                cerr << "Unknown sensor orientation: " << ori << endl;
                PrintUsage();
                return -1;
            }
        }
        else if (arg == "--smoothing" && i + 1 < argc) {
            smoothingFactor = stof(argv[++i]);
            if (smoothingFactor < 0.0f || smoothingFactor > 1.0f) {
                cerr << "Smoothing factor must be between 0.0 and 1.0" << endl;
                return -1;
            }
        }
        else if (arg == "--max-body-distance" && i + 1 < argc) {
            maxBodyDistanceMm = stof(argv[++i]);
        }
        else if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return 0;
        }
        else if (arg[0] != '-') {
            mkvPaths.push_back(arg);
        }
        else {
            cerr << "Unknown argument: " << arg << endl;
            PrintUsage();
            return -1;
        }
    }

    if (mkvPaths.empty()) {
        cerr << "Error: No MKV files specified!" << endl;
        PrintUsage();
        return -1;
    }

    bool egoMode = !helmetSerial.empty();

    if (egoMode) {
        if (tCheckerToAPath.empty()) {
            cerr << "Error: --t-checker-to-a is required for ego-view mode!" << endl;
            return -1;
        }
        if (calibrationPath.empty()) {
            cerr << "Error: --calibration is required for ego-view mode!" << endl;
            return -1;
        }
    }

    cout << "Input MKV files: " << mkvPaths.size() << endl;
    for (size_t i = 0; i < mkvPaths.size(); i++) {
        cout << "  [" << i << "] " << mkvPaths[i] << endl;
    }

    if (egoMode) {
        cout << "\nEgo-view mode ENABLED" << endl;
        cout << "  Helmet serial: " << helmetSerial << endl;
        cout << "  T_checker_to_A: " << tCheckerToAPath << endl;
        cout << "  Checkerboard: " << helmetCBConfig.rows << "x" << helmetCBConfig.cols
             << " (" << helmetCBConfig.squareMm << "mm)" << endl;
        cout << "  Ego output: " << egoOutputDir << endl;
        cout << "  Ego fusion mode: " << egoFusionMode << endl;
    }

    // Load calibration
    if (!calibrationPath.empty()) {
        cout << "\nLoading calibration: " << calibrationPath << endl;
        if (!LoadCalibration(calibrationPath, g_calibration)) {
            if (egoMode) {
                cerr << "Error: Calibration required for ego-view mode!" << endl;
                return -1;
            }
            cerr << "Warning: Failed to load calibration, fusion disabled" << endl;
        }
    } else if (mkvPaths.size() > 1) {
        cout << "\nWarning: Multiple MKV files but no calibration specified!" << endl;
        cout << "Bodies will NOT be fused. Use --calibration for fusion." << endl;
    }

    // Load T_checker_to_A for ego-view
    Transform tCheckerToA;
    if (egoMode) {
        cout << "Loading T_checker_to_A: " << tCheckerToAPath << endl;
        if (!LoadTransform(tCheckerToAPath, tCheckerToA)) {
            cerr << "Error: Failed to load T_checker_to_A!" << endl;
            return -1;
        }
        cout << "Loaded T_checker_to_A transform" << endl;

        // Create output directories
        fs::create_directories(egoOutputDir + "/images");
        fs::create_directories(egoOutputDir + "/annotations");
        cout << "Created ego output directories: " << egoOutputDir << endl;
    }

    // Initialize processors — first pass to identify helmet camera
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = processingMode;
    trackerConfig.sensor_orientation = sensorOrientation;

    vector<MkvProcessor> processors(mkvPaths.size());

    // If ego mode, we need to identify the helmet camera before initialization.
    // We do a preliminary open to read serial numbers, then close and re-open properly.
    int helmetIdx = -1;
    if (egoMode) {
        for (size_t i = 0; i < mkvPaths.size(); i++) {
            k4a_playback_t tempPlayback = nullptr;
            if (k4a_playback_open(mkvPaths[i].c_str(), &tempPlayback) == K4A_RESULT_SUCCEEDED) {
                char serial[64] = {0};
                size_t serialSize = sizeof(serial);
                if (k4a_playback_get_tag(tempPlayback, "K4A_DEVICE_SERIAL_NUMBER", serial, &serialSize)
                    == K4A_BUFFER_RESULT_SUCCEEDED) {
                    if (string(serial) == helmetSerial) {
                        helmetIdx = (int)i;
                    }
                }
                k4a_playback_close(tempPlayback);
            }

            if (helmetIdx >= 0) break;
        }

        if (helmetIdx < 0) {
            cerr << "Error: Helmet serial '" << helmetSerial << "' not found in any MKV file!" << endl;
            cerr << "Available serials:" << endl;
            for (size_t i = 0; i < mkvPaths.size(); i++) {
                k4a_playback_t tempPlayback = nullptr;
                if (k4a_playback_open(mkvPaths[i].c_str(), &tempPlayback) == K4A_RESULT_SUCCEEDED) {
                    char serial[64] = {0};
                    size_t serialSize = sizeof(serial);
                    if (k4a_playback_get_tag(tempPlayback, "K4A_DEVICE_SERIAL_NUMBER", serial, &serialSize)
                        == K4A_BUFFER_RESULT_SUCCEEDED) {
                        cerr << "  [" << i << "] " << serial << " (" << mkvPaths[i] << ")" << endl;
                    }
                    k4a_playback_close(tempPlayback);
                }
            }
            return -1;
        }

        cout << "Helmet camera found at index " << helmetIdx << endl;
        processors[helmetIdx].isHelmet = true;
    }

    for (size_t i = 0; i < mkvPaths.size(); i++) {
        if (!InitMkvProcessor(processors[i], mkvPaths[i], (int)i, trackerConfig, smoothingFactor)) {
            cerr << "Failed to initialize processor " << i << endl;
            return -1;
        }
    }

    // Remap calibration device indices to match processor ordering by serial number.
    // The calibration file uses its own device_index numbering (from multi_device_calibration),
    // but the offline processor assigns indices sequentially from the MKV command-line order.
    // These may differ (e.g., helmet camera at MKV index 0 shifts all fixed camera indices).
    if (g_calibration.isLoaded) {
        cout << "\nRemapping calibration to processor indices by serial number..." << endl;
        for (auto& cam : g_calibration.cameras) {
            int oldIdx = cam.deviceIndex;
            bool matched = false;
            for (size_t i = 0; i < processors.size(); i++) {
                if (processors[i].serialNumber == cam.serialNumber) {
                    cam.deviceIndex = (int)i;
                    matched = true;
                    cout << "  Calibration '" << cam.serialNumber << "': device_index "
                         << oldIdx << " -> " << cam.deviceIndex << endl;
                    break;
                }
            }
            if (!matched) {
                cout << "  Warning: Calibration camera '" << cam.serialNumber
                     << "' (index " << oldIdx << ") not found in any MKV file" << endl;
            }
        }
    }

    // Store helmet calibration for 3D→2D projection
    k4a_calibration_t helmetCalibration = {};
    if (egoMode) {
        helmetCalibration = processors[helmetIdx].calibration;
    }

    // Open output file
    ofstream csvFile(outputPath);
    if (!csvFile.is_open()) {
        cerr << "Failed to open output file: " << outputPath << endl;
        return -1;
    }
    WriteCSVHeader(csvFile);

    // Process frames
    cout << "\nProcessing..." << endl;

    uint64_t frameCount = 0;
    bool allEOF = false;
    int egoFrameCount = 0;
    int cbDetectedCount = 0;

    // EMA temporal smoothing state for helmet pose
    const double EMA_ALPHA = 0.75;
    const double EMA_ALPHA_CAM_SWITCH = 0.15; // much stronger smoothing on camera switch
    const uint64_t STALENESS_THRESHOLD_US = 200000; // 200ms in microseconds
    Transform prevHelmetPose;
    bool hasPrevPose = false;
    uint64_t prevPoseTimestamp = 0;
    int prevDetectionCamera = -1;

    // Latest fused body data (updated when fixed cameras produce new fusion)
    vector<FusedBody> latestFused;
    uint64_t latestFusedTimestamp = 0;
    uint64_t lastCsvTimestamp = 0;

    // Body tracking state: spatial continuity across frames
    k4a_float3_t prevPelvisWorld = {0, 0, 0};
    bool hasPrevPelvis = false;

    // Whole-skeleton temporal smoothing (applied uniformly to all joints)
    FusedBody prevSmoothedBody;
    bool hasPrevSmoothed = false;
    uint64_t prevSmoothedTimestamp = 0;
    const uint64_t SKEL_STALENESS_US = 100000;   // 100ms
    const float BODY_SWITCH_THRESHOLD_MM = 200.0f; // if pelvis jumps > this, carry forward previous

    // Confidence carry-forward TTL: stop holding stale joint positions after this many frames
    const int MAX_CARRY_FRAMES = 5;              // ~167ms at 30fps
    int jointCarryCount[K4ABT_JOINT_COUNT] = {};

    // Whole-body carry-forward limit: prevents permanent skeleton freeze when the
    // initial body selection lands on a mis-fused phantom body.  After this many
    // consecutive carry-forward frames, force-accept the new body.
    const int MAX_BODY_CARRY_FRAMES = 10;        // ~333ms at 30fps
    int bodyCarryCount = 0;

    // Warm-up: don't enable carry-forward until we've had this many consecutive
    // frames of normal EMA tracking (pelvisDist < threshold).  This prevents a
    // transient mis-fused body on the very first frame from locking in via
    // carry-forward for MAX_BODY_CARRY_FRAMES before correcting.
    const int CARRY_FORWARD_WARMUP = 3;
    int stableTrackCount = 0;  // consecutive frames of normal EMA (Case 2)

    // Post-fusion temporal EMA for local mode: smooths the fused helmet-local
    // skeleton across frames.  In world mode, cross-camera averaging in
    // FuseBodiesAtTimestamp() provides natural noise reduction before the EMA.
    // Local mode lacks this pre-averaging, so we add a second temporal pass
    // after FuseHelmetLocalSkeletons() to match world mode's smoothness.
    vector<Joint3D> prevFusedLocalSkeleton;
    bool hasPrevFusedLocal = false;
    uint64_t prevFusedLocalTimestamp = 0;

    // Seed all processors with their first frame
    for (auto& proc : processors) {
        if (!proc.isEOF) {
            ProcessNextFrame(proc);
        }
    }

    uint64_t currentMinTimestamp = 0;

    while (!allEOF) {
      try {
        bool wroteCsvRow = false;

        // Find the processor with the OLDEST timestamp (most behind)
        int nextIdx = -1;
        uint64_t oldestTs = UINT64_MAX;
        for (size_t i = 0; i < processors.size(); i++) {
            if (!processors[i].isEOF && processors[i].lastTimestamp < oldestTs) {
                oldestTs = processors[i].lastTimestamp;
                nextIdx = (int)i;
            }
        }

        if (nextIdx < 0) break; // all EOF

        bool isHelmetFrame = processors[nextIdx].isHelmet;

        // ================================================================
        // Fixed camera frame: update fusion
        // ================================================================
        if (!isHelmetFrame) {
            // Find minimum timestamp among fixed cameras
            uint64_t minTimestamp = UINT64_MAX;
            for (const auto& proc : processors) {
                if (!proc.isEOF && !proc.isHelmet && proc.lastTimestamp < minTimestamp) {
                    minTimestamp = proc.lastTimestamp;
                }
            }

            if (minTimestamp == UINT64_MAX) break;

            // Collect frames within sync threshold (fixed cameras only)
            // Cameras within the sync window contribute directly.
            // Cameras whose [prev, last] interval straddles minTimestamp
            // contribute via interpolation (LERP position, SLERP orientation).
            vector<FrameData> syncFrames;
            for (auto& proc : processors) {
                if (proc.isEOF || proc.isHelmet) continue;
                if (proc.lastTimestamp <= minTimestamp + syncThresholdUs) {
                    // Direct match: camera's latest frame is within sync window
                    FrameData fd;
                    fd.timestamp_usec = proc.lastTimestamp;
                    fd.deviceIndex = proc.deviceIndex;
                    fd.bodies = proc.lastBodies;
                    syncFrames.push_back(fd);
                } else if (proc.hasPrevFrame &&
                           proc.prevTimestamp <= minTimestamp &&
                           proc.lastTimestamp > minTimestamp + syncThresholdUs) {
                    // Interpolation: camera's frame interval straddles the target
                    float t = 0.5f;
                    uint64_t dt = proc.lastTimestamp - proc.prevTimestamp;
                    if (dt > 0) {
                        t = (float)(minTimestamp - proc.prevTimestamp) / (float)dt;
                    }
                    FrameData fd;
                    fd.timestamp_usec = minTimestamp;
                    fd.deviceIndex = proc.deviceIndex;
                    fd.bodies = InterpolateBodies(proc.prevBodies, proc.lastBodies, t);
                    syncFrames.push_back(fd);
                }
            }

            // Fuse bodies and update latest result
            if (!syncFrames.empty() && minTimestamp != lastCsvTimestamp) {
                vector<FusedBody> fused;
                if (g_calibration.isLoaded && syncFrames.size() > 1) {
                    fused = FuseBodiesAtTimestamp(syncFrames);
                } else {
                    for (const auto& body : syncFrames[0].bodies) {
                        FusedBody fb;
                        fb.id = body.id;
                        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                            fb.joints[j].position = body.skeleton.joints[j].position;
                            fb.joints[j].orientation = body.skeleton.joints[j].orientation;
                            fb.joints[j].confidence = body.skeleton.joints[j].confidence_level;
                            fb.joints[j].sourceDeviceIndex = syncFrames[0].deviceIndex;
                        }
                        fused.push_back(fb);
                    }
                }

                // Distance filter: remove bodies whose pelvis exceeds maxBodyDistanceMm
                // from the world origin.  This eliminates phantom/ghost detections
                // at the far end of the room (e.g. reflections at ~5m).
                if (maxBodyDistanceMm > 0) {
                    fused.erase(
                        remove_if(fused.begin(), fused.end(), [&](const FusedBody& fb) {
                            const auto& p = fb.joints[K4ABT_JOINT_PELVIS].position;
                            float dist = sqrtf(p.xyz.x * p.xyz.x +
                                               p.xyz.y * p.xyz.y +
                                               p.xyz.z * p.xyz.z);
                            return dist > maxBodyDistanceMm;
                        }),
                        fused.end());
                }

                latestFused = fused;
                latestFusedTimestamp = minTimestamp;

                // Write to CSV (post-filter, so CSV also omits phantom bodies)
                for (const auto& body : fused) {
                    WriteFusedBodyToCSV(csvFile, minTimestamp, body);
                }
                lastCsvTimestamp = minTimestamp;
                wroteCsvRow = true;
                currentMinTimestamp = minTimestamp;

                frameCount++;
                if (frameCount % 100 == 0) {
                    cout << "Processed " << frameCount << " frames (ego: " << egoFrameCount
                         << ", cb: " << cbDetectedCount << ")..." << endl;
                    cout.flush();
                }
            }
        }

        // ================================================================
        // Helmet camera frame: generate ego-view output
        // ================================================================
        // Only generate ego frames when a new physical helmet image arrives.
        // The helmet camera's native timestamps are the master reference.
        bool egoTrigger = false;
        uint64_t egoTimestamp = 0;

        if (isHelmetFrame && egoMode && processors[helmetIdx].hasNewFrame) {
            egoTrigger = true;
            egoTimestamp = processors[helmetIdx].lastTimestamp;
        }

        if (egoTrigger) {
            cv::Size patternSize = helmetCBConfig.patternSize();

            Transform helmetPose;  // world-frame pose (used for JSON metadata in both modes)
            bool detectionSuccess = false;
            int detectionCamera = -1;
            vector<Joint3D> joints3D;
            vector<Joint2D> joints2D;
            int numBodies = (int)latestFused.size();

            if (egoFusionMode == "local") {
                // ============================================================
                // LOCAL-FRAME FUSION: transform joints to helmet-local coords
                // per-camera using only camera-local measurements, then fuse.
                // Eliminates extrinsic calibration errors from skeleton path.
                // ============================================================
                vector<CameraHelmetSkeleton> localCandidates;
                vector<WeightedPose> poseCandidates;  // world-frame poses for JSON metadata only

                for (auto& proc : processors) {
                    if (proc.isHelmet || proc.isEOF) continue;
                    if (proc.lastColorImage.empty() || proc.lastDepthImage.empty()) continue;
                    if (proc.lastBodies.empty()) continue;

                    // Detect checkerboard corners
                    vector<cv::Point2f> corners;
                    if (!DetectCheckerboardCorners(proc.lastColorImage, corners, patternSize))
                        continue;

                    // Convert 2D corners to 3D in CAMERA space (no world transform)
                    vector<cv::Point3f> points3D_cam;
                    if (!Convert2DTo3DOffline(proc.calibration, proc.transformation,
                                               proc.lastDepthImage, corners, points3D_cam))
                        continue;

                    // Average depth for confidence weighting
                    float avgDepth = 0.0f;
                    for (const auto& p : points3D_cam)
                        avgDepth += sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                    avgDepth /= (float)points3D_cam.size();

                    // Checkerboard pose in CAMERA space (NOT world)
                    Transform checkerPose_cam = ComputeCheckerboardPose(points3D_cam, patternSize);
                    if (!checkerPose_cam.valid) continue;

                    // Helmet pose in camera space: helmetPose_cam = checkerPose_cam ∘ T_checker_to_A
                    Transform helmetPose_cam;
                    helmetPose_cam.rotation = checkerPose_cam.rotation * tCheckerToA.rotation;
                    helmetPose_cam.translation = checkerPose_cam.rotation * tCheckerToA.translation
                                                + checkerPose_cam.translation;
                    helmetPose_cam.valid = true;

                    // Select best body from this camera using camera-local pelvis continuity
                    int bodyIdx = SelectBestBodyLocal(proc.lastBodies,
                                                      proc.prevPelvisLocal, proc.hasPrevPelvisLocal);
                    if (bodyIdx < 0) continue;

                    const k4abt_body_t& selectedBody = proc.lastBodies[bodyIdx];
                    uint64_t curTs = processors[helmetIdx].lastTimestamp;

                    // Per-camera temporal smoothing in camera-local space
                    k4abt_body_t smoothedBody = selectedBody;
                    float pelvisDist = 0.0f;
                    if (proc.hasPrevSmoothedLocal) {
                        float dx = selectedBody.skeleton.joints[0].position.xyz.x
                                 - proc.prevSmoothedBodyLocal.skeleton.joints[0].position.xyz.x;
                        float dy = selectedBody.skeleton.joints[0].position.xyz.y
                                 - proc.prevSmoothedBodyLocal.skeleton.joints[0].position.xyz.y;
                        float dz = selectedBody.skeleton.joints[0].position.xyz.z
                                 - proc.prevSmoothedBodyLocal.skeleton.joints[0].position.xyz.z;
                        pelvisDist = sqrt(dx*dx + dy*dy + dz*dz);
                    }

                    if (proc.hasPrevSmoothedLocal && pelvisDist > BODY_SWITCH_THRESHOLD_MM &&
                        (curTs - proc.prevSmoothedTimestampLocal) < SKEL_STALENESS_US &&
                        proc.bodyCarryCountLocal < MAX_BODY_CARRY_FRAMES &&
                        proc.stableTrackCountLocal >= CARRY_FORWARD_WARMUP) {
                        // Large jump (after warm-up): carry forward previous skeleton
                        smoothedBody = proc.prevSmoothedBodyLocal;
                        proc.bodyCarryCountLocal++;
                    } else if (proc.hasPrevSmoothedLocal && pelvisDist <= BODY_SWITCH_THRESHOLD_MM &&
                               (curTs - proc.prevSmoothedTimestampLocal) < SKEL_STALENESS_US) {
                        // Normal motion: adaptive per-joint smoothing
                        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                            float alpha = GetJointSmoothAlpha(j);
                            // Confidence-adaptive smoothing: reduce alpha for
                            // low-confidence joints to suppress noisy positions.
                            if (selectedBody.skeleton.joints[j].confidence_level == K4ABT_JOINT_CONFIDENCE_NONE) {
                                alpha = min(alpha, 0.15f);
                            } else if (selectedBody.skeleton.joints[j].confidence_level == K4ABT_JOINT_CONFIDENCE_LOW) {
                                alpha = min(alpha, 0.30f);
                            }
                            smoothedBody.skeleton.joints[j].position.xyz.x =
                                alpha * selectedBody.skeleton.joints[j].position.xyz.x +
                                (1.0f - alpha) * proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.x;
                            smoothedBody.skeleton.joints[j].position.xyz.y =
                                alpha * selectedBody.skeleton.joints[j].position.xyz.y +
                                (1.0f - alpha) * proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.y;
                            smoothedBody.skeleton.joints[j].position.xyz.z =
                                alpha * selectedBody.skeleton.joints[j].position.xyz.z +
                                (1.0f - alpha) * proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.z;
                        }
                        proc.prevPelvisLocal = selectedBody.skeleton.joints[K4ABT_JOINT_PELVIS].position;
                        proc.hasPrevPelvisLocal = true;
                        proc.bodyCarryCountLocal = 0;
                        proc.stableTrackCountLocal++;
                    } else if (proc.hasPrevSmoothedLocal) {
                        // Carry-forward limit exceeded, stale, or warm-up period:
                        // gradual transition to avoid sudden jumps.
                        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                            float alpha = 0.30f;  // heavy smoothing for transition
                            smoothedBody.skeleton.joints[j].position.xyz.x =
                                alpha * selectedBody.skeleton.joints[j].position.xyz.x +
                                (1.0f - alpha) * proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.x;
                            smoothedBody.skeleton.joints[j].position.xyz.y =
                                alpha * selectedBody.skeleton.joints[j].position.xyz.y +
                                (1.0f - alpha) * proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.y;
                            smoothedBody.skeleton.joints[j].position.xyz.z =
                                alpha * selectedBody.skeleton.joints[j].position.xyz.z +
                                (1.0f - alpha) * proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.z;
                        }
                        proc.prevPelvisLocal = selectedBody.skeleton.joints[K4ABT_JOINT_PELVIS].position;
                        proc.hasPrevPelvisLocal = true;
                        proc.bodyCarryCountLocal = 0;
                        proc.stableTrackCountLocal = 0;
                        memset(proc.jointCarryCountLocal, 0, sizeof(proc.jointCarryCountLocal));
                    } else {
                        // First frame: accept as-is
                        proc.prevPelvisLocal = selectedBody.skeleton.joints[K4ABT_JOINT_PELVIS].position;
                        proc.hasPrevPelvisLocal = true;
                        proc.bodyCarryCountLocal = 0;
                        proc.stableTrackCountLocal = 0;
                        memset(proc.jointCarryCountLocal, 0, sizeof(proc.jointCarryCountLocal));
                    }

                    // Carry forward low-confidence joints
                    if (proc.hasPrevSmoothedLocal) {
                        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                            // Zero-position guard: (0,0,0) with conf=NONE is a sentinel
                            bool isZeroSentinel =
                                selectedBody.skeleton.joints[j].confidence_level == K4ABT_JOINT_CONFIDENCE_NONE &&
                                fabsf(selectedBody.skeleton.joints[j].position.xyz.x) < 0.01f &&
                                fabsf(selectedBody.skeleton.joints[j].position.xyz.y) < 0.01f &&
                                fabsf(selectedBody.skeleton.joints[j].position.xyz.z) < 0.01f;
                            bool prevHasRealPos =
                                fabsf(proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.x) > 0.01f ||
                                fabsf(proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.y) > 0.01f ||
                                fabsf(proc.prevSmoothedBodyLocal.skeleton.joints[j].position.xyz.z) > 0.01f;

                            if (isZeroSentinel && prevHasRealPos) {
                                // Unlimited carry-forward for zero-position sentinels
                                smoothedBody.skeleton.joints[j].position =
                                    proc.prevSmoothedBodyLocal.skeleton.joints[j].position;
                                smoothedBody.skeleton.joints[j].orientation =
                                    proc.prevSmoothedBodyLocal.skeleton.joints[j].orientation;
                            } else if (smoothedBody.skeleton.joints[j].confidence_level == K4ABT_JOINT_CONFIDENCE_NONE &&
                                proc.prevSmoothedBodyLocal.skeleton.joints[j].confidence_level != K4ABT_JOINT_CONFIDENCE_NONE &&
                                proc.jointCarryCountLocal[j] < MAX_CARRY_FRAMES) {
                                // Regular carry-forward (up to MAX_CARRY_FRAMES)
                                smoothedBody.skeleton.joints[j] = proc.prevSmoothedBodyLocal.skeleton.joints[j];
                                proc.jointCarryCountLocal[j]++;
                            } else {
                                proc.jointCarryCountLocal[j] = 0;
                            }
                        }
                    }

                    proc.prevSmoothedBodyLocal = smoothedBody;
                    proc.hasPrevSmoothedLocal = true;
                    proc.prevSmoothedTimestampLocal = curTs;

                    // Transform smoothed body to helmet-local coordinates
                    CameraHelmetSkeleton camSkel;
                    camSkel.cameraIndex = proc.deviceIndex;
                    TransformBodyToHelmetLocal(smoothedBody, helmetPose_cam, camSkel.joints);
                    float weight = 1.0f / (avgDepth * avgDepth);
                    if (prevDetectionCamera >= 0 && proc.deviceIndex == prevDetectionCamera)
                        weight *= 2.0f;
                    camSkel.weight = weight;
                    localCandidates.push_back(camSkel);

                    // Compute world-frame pose for JSON metadata only:
                    // R_world = R_ext * R_helmet_cam, t_world = R_ext * t_helmet_cam + t_ext
                    for (const auto& cam : g_calibration.cameras) {
                        if (cam.deviceIndex == proc.deviceIndex && cam.isValid) {
                            cv::Mat R_ext(3, 3, CV_64F), t_ext(3, 1, CV_64F);
                            for (int r = 0; r < 3; r++) {
                                for (int c = 0; c < 3; c++)
                                    R_ext.at<double>(r, c) = (double)cam.rotation[r][c];
                                t_ext.at<double>(r, 0) = (double)cam.translation[r];
                            }
                            Transform worldPose;
                            worldPose.rotation = R_ext * helmetPose_cam.rotation;
                            worldPose.translation = R_ext * helmetPose_cam.translation + t_ext;
                            worldPose.valid = true;
                            poseCandidates.push_back({worldPose, weight, proc.deviceIndex});
                            break;
                        }
                    }
                }

                if (!localCandidates.empty()) {
                    // Outlier rejection: check pelvis agreement across cameras
                    if (localCandidates.size() >= 2) {
                        vector<float> pxs, pys, pzs;
                        for (const auto& cs : localCandidates) {
                            pxs.push_back(cs.joints[K4ABT_JOINT_PELVIS].x);
                            pys.push_back(cs.joints[K4ABT_JOINT_PELVIS].y);
                            pzs.push_back(cs.joints[K4ABT_JOINT_PELVIS].z);
                        }
                        sort(pxs.begin(), pxs.end());
                        sort(pys.begin(), pys.end());
                        sort(pzs.begin(), pzs.end());
                        size_t mid = pxs.size() / 2;
                        float medX = pxs[mid], medY = pys[mid], medZ = pzs[mid];

                        const float PELVIS_OUTLIER_MM = 150.0f;
                        vector<CameraHelmetSkeleton> filtered;
                        for (const auto& cs : localCandidates) {
                            float dx = cs.joints[K4ABT_JOINT_PELVIS].x - medX;
                            float dy = cs.joints[K4ABT_JOINT_PELVIS].y - medY;
                            float dz = cs.joints[K4ABT_JOINT_PELVIS].z - medZ;
                            if (sqrt(dx*dx + dy*dy + dz*dz) <= PELVIS_OUTLIER_MM)
                                filtered.push_back(cs);
                        }
                        if (!filtered.empty())
                            localCandidates = filtered;
                    }

                    joints3D = FuseHelmetLocalSkeletons(localCandidates);

                    // Post-fusion temporal EMA: smooth the fused helmet-local
                    // skeleton to compensate for the lack of cross-camera
                    // pre-averaging that world mode gets from FuseBodiesAtTimestamp().
                    uint64_t curTsLocal = processors[helmetIdx].lastTimestamp;
                    if (hasPrevFusedLocal &&
                        (curTsLocal - prevFusedLocalTimestamp) < SKEL_STALENESS_US &&
                        prevFusedLocalSkeleton.size() == joints3D.size()) {
                        for (size_t j = 0; j < joints3D.size(); j++) {
                            float alpha = GetJointSmoothAlpha((int)j);
                            if (joints3D[j].confidence == 0) {
                                alpha = min(alpha, 0.15f);
                            } else if (joints3D[j].confidence == 1) {
                                alpha = min(alpha, 0.30f);
                            }
                            joints3D[j].x = alpha * joints3D[j].x +
                                            (1.0f - alpha) * prevFusedLocalSkeleton[j].x;
                            joints3D[j].y = alpha * joints3D[j].y +
                                            (1.0f - alpha) * prevFusedLocalSkeleton[j].y;
                            joints3D[j].z = alpha * joints3D[j].z +
                                            (1.0f - alpha) * prevFusedLocalSkeleton[j].z;
                        }
                    }
                    prevFusedLocalSkeleton = joints3D;
                    hasPrevFusedLocal = true;
                    prevFusedLocalTimestamp = curTsLocal;

                    joints2D = ProjectSkeleton(joints3D, helmetCalibration);
                    detectionSuccess = true;

                    // World-frame pose for JSON metadata (does NOT affect skeleton accuracy)
                    if (!poseCandidates.empty()) {
                        helmetPose = FuseHelmetPoses(poseCandidates);
                        float bestWeight = 0.0f;
                        for (const auto& c : poseCandidates) {
                            if (c.weight > bestWeight) {
                                bestWeight = c.weight;
                                detectionCamera = c.cameraIndex;
                            }
                        }

                        uint64_t curTs = processors[helmetIdx].lastTimestamp;
                        if (hasPrevPose && (curTs - prevPoseTimestamp) < STALENESS_THRESHOLD_US) {
                            bool camSwitched = (prevDetectionCamera >= 0 && detectionCamera != prevDetectionCamera);
                            double alpha = camSwitched ? EMA_ALPHA_CAM_SWITCH : EMA_ALPHA;
                            helmetPose = SmoothPose(helmetPose, prevHelmetPose, alpha);
                        }
                        prevHelmetPose = helmetPose;
                        hasPrevPose = true;
                        prevPoseTimestamp = curTs;
                        prevDetectionCamera = detectionCamera;
                    }
                }

            } else {
                // ============================================================
                // WORLD-FRAME FUSION (existing pipeline, --ego-fusion-mode world)
                // ============================================================
                vector<WeightedPose> candidates;

                for (auto& proc : processors) {
                    if (proc.isHelmet || proc.isEOF) continue;
                    if (proc.lastColorImage.empty() || proc.lastDepthImage.empty()) continue;

                    vector<cv::Point2f> corners;
                    if (!DetectCheckerboardCorners(proc.lastColorImage, corners, patternSize))
                        continue;

                    vector<cv::Point3f> points3D_cam;
                    if (!Convert2DTo3DOffline(proc.calibration, proc.transformation,
                                               proc.lastDepthImage, corners, points3D_cam))
                        continue;

                    float avgDepth = 0.0f;
                    for (const auto& p : points3D_cam)
                        avgDepth += sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                    avgDepth /= (float)points3D_cam.size();

                    // Transform corners from camera space to world space
                    for (const auto& cam : g_calibration.cameras) {
                        if (cam.deviceIndex == proc.deviceIndex && cam.isValid) {
                            TransformPointsToWorld(points3D_cam, cam);
                            break;
                        }
                    }

                    Transform checkerPose = ComputeCheckerboardPose(points3D_cam, patternSize);
                    if (!checkerPose.valid) continue;

                    Transform candidatePose;
                    candidatePose.rotation = checkerPose.rotation * tCheckerToA.rotation;
                    candidatePose.translation = checkerPose.rotation * tCheckerToA.translation
                                               + checkerPose.translation;
                    candidatePose.valid = true;

                    float weight = 1.0f / (avgDepth * avgDepth);
                    if (prevDetectionCamera >= 0 && proc.deviceIndex == prevDetectionCamera)
                        weight *= 2.0f;
                    candidates.push_back({candidatePose, weight, proc.deviceIndex});
                }

                if (!candidates.empty()) {
                    helmetPose = FuseHelmetPoses(candidates);
                    detectionSuccess = true;

                    float bestWeight = 0.0f;
                    for (const auto& c : candidates) {
                        if (c.weight > bestWeight) {
                            bestWeight = c.weight;
                            detectionCamera = c.cameraIndex;
                        }
                    }

                    uint64_t curTs = processors[helmetIdx].lastTimestamp;
                    if (hasPrevPose && (curTs - prevPoseTimestamp) < STALENESS_THRESHOLD_US) {
                        bool camSwitched = (prevDetectionCamera >= 0 && detectionCamera != prevDetectionCamera);
                        double alpha = camSwitched ? EMA_ALPHA_CAM_SWITCH : EMA_ALPHA;
                        helmetPose = SmoothPose(helmetPose, prevHelmetPose, alpha);
                    }
                    prevHelmetPose = helmetPose;
                    hasPrevPose = true;
                    prevPoseTimestamp = curTs;
                    prevDetectionCamera = detectionCamera;
                }

                // World-frame body selection, smoothing, and projection
                if (detectionSuccess && !latestFused.empty()) {
                    int bodyIdx = SelectBestBody(latestFused, prevPelvisWorld, hasPrevPelvis);
                    if (bodyIdx < 0) bodyIdx = 0;

                    const FusedBody& selectedBody = latestFused[bodyIdx];
                    uint64_t curTs = processors[helmetIdx].lastTimestamp;

                    float pelvisDist = 0.0f;
                    if (hasPrevSmoothed) {
                        float dx = selectedBody.joints[0].position.xyz.x - prevSmoothedBody.joints[0].position.xyz.x;
                        float dy = selectedBody.joints[0].position.xyz.y - prevSmoothedBody.joints[0].position.xyz.y;
                        float dz = selectedBody.joints[0].position.xyz.z - prevSmoothedBody.joints[0].position.xyz.z;
                        pelvisDist = sqrt(dx*dx + dy*dy + dz*dz);
                    }

                    FusedBody smoothedBody;
                    bool useCarryForward = false;

                    if (hasPrevSmoothed && pelvisDist > BODY_SWITCH_THRESHOLD_MM &&
                        (curTs - prevSmoothedTimestamp) < SKEL_STALENESS_US &&
                        bodyCarryCount < MAX_BODY_CARRY_FRAMES &&
                        stableTrackCount >= CARRY_FORWARD_WARMUP) {
                        // Only carry-forward after warm-up: if the tracker hasn't
                        // established stable tracking yet, a large pelvis jump
                        // likely means the initial body was wrong, not that we
                        // should freeze on it.
                        smoothedBody = prevSmoothedBody;
                        useCarryForward = true;
                        bodyCarryCount++;
                        memset(jointCarryCount, 0, sizeof(jointCarryCount));
                    } else if (hasPrevSmoothed && pelvisDist < BODY_SWITCH_THRESHOLD_MM &&
                               (curTs - prevSmoothedTimestamp) < SKEL_STALENESS_US) {
                        smoothedBody = selectedBody;
                        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                            float alpha = GetJointSmoothAlpha(j);
                            // Confidence-adaptive smoothing: reduce alpha for
                            // low-confidence joints to suppress noisy positions.
                            if (selectedBody.joints[j].confidence == K4ABT_JOINT_CONFIDENCE_NONE) {
                                alpha = min(alpha, 0.15f);
                            } else if (selectedBody.joints[j].confidence == K4ABT_JOINT_CONFIDENCE_LOW) {
                                alpha = min(alpha, 0.30f);
                            }
                            smoothedBody.joints[j].position.xyz.x = (float)(
                                alpha * selectedBody.joints[j].position.xyz.x +
                                (1.0f - alpha) * prevSmoothedBody.joints[j].position.xyz.x);
                            smoothedBody.joints[j].position.xyz.y = (float)(
                                alpha * selectedBody.joints[j].position.xyz.y +
                                (1.0f - alpha) * prevSmoothedBody.joints[j].position.xyz.y);
                            smoothedBody.joints[j].position.xyz.z = (float)(
                                alpha * selectedBody.joints[j].position.xyz.z +
                                (1.0f - alpha) * prevSmoothedBody.joints[j].position.xyz.z);
                        }
                        prevPelvisWorld = selectedBody.joints[K4ABT_JOINT_PELVIS].position;
                        hasPrevPelvis = true;
                        bodyCarryCount = 0;
                        stableTrackCount++;
                    } else if (hasPrevSmoothed) {
                        // Carry-forward limit exceeded, stale, or warm-up period:
                        // gradual transition to avoid sudden jumps.
                        smoothedBody = selectedBody;
                        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                            float alpha = 0.30f;  // heavy smoothing for transition
                            smoothedBody.joints[j].position.xyz.x = (float)(
                                alpha * selectedBody.joints[j].position.xyz.x +
                                (1.0f - alpha) * prevSmoothedBody.joints[j].position.xyz.x);
                            smoothedBody.joints[j].position.xyz.y = (float)(
                                alpha * selectedBody.joints[j].position.xyz.y +
                                (1.0f - alpha) * prevSmoothedBody.joints[j].position.xyz.y);
                            smoothedBody.joints[j].position.xyz.z = (float)(
                                alpha * selectedBody.joints[j].position.xyz.z +
                                (1.0f - alpha) * prevSmoothedBody.joints[j].position.xyz.z);
                        }
                        prevPelvisWorld = selectedBody.joints[K4ABT_JOINT_PELVIS].position;
                        hasPrevPelvis = true;
                        bodyCarryCount = 0;
                        stableTrackCount = 0;
                        memset(jointCarryCount, 0, sizeof(jointCarryCount));
                    } else {
                        // First frame: accept as-is
                        smoothedBody = selectedBody;
                        prevPelvisWorld = selectedBody.joints[K4ABT_JOINT_PELVIS].position;
                        hasPrevPelvis = true;
                        bodyCarryCount = 0;
                        stableTrackCount = 0;
                        memset(jointCarryCount, 0, sizeof(jointCarryCount));
                    }

                    if (hasPrevSmoothed) {
                        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
                            // Zero-position guard: (0,0,0) with conf=NONE is a sentinel
                            // meaning "no detection", not a real position.  Always carry
                            // forward the previous position to avoid collapsing the
                            // skeleton toward the world origin.
                            bool isZeroSentinel =
                                selectedBody.joints[j].confidence == K4ABT_JOINT_CONFIDENCE_NONE &&
                                fabsf(selectedBody.joints[j].position.xyz.x) < 0.01f &&
                                fabsf(selectedBody.joints[j].position.xyz.y) < 0.01f &&
                                fabsf(selectedBody.joints[j].position.xyz.z) < 0.01f;
                            bool prevHasRealPos =
                                fabsf(prevSmoothedBody.joints[j].position.xyz.x) > 0.01f ||
                                fabsf(prevSmoothedBody.joints[j].position.xyz.y) > 0.01f ||
                                fabsf(prevSmoothedBody.joints[j].position.xyz.z) > 0.01f;

                            if (isZeroSentinel && prevHasRealPos) {
                                // Unlimited carry-forward for zero-position sentinels
                                smoothedBody.joints[j].position = prevSmoothedBody.joints[j].position;
                                smoothedBody.joints[j].orientation = prevSmoothedBody.joints[j].orientation;
                                // Keep confidence as NONE to indicate this is estimated
                            } else if (smoothedBody.joints[j].confidence == K4ABT_JOINT_CONFIDENCE_NONE &&
                                prevSmoothedBody.joints[j].confidence != K4ABT_JOINT_CONFIDENCE_NONE &&
                                jointCarryCount[j] < MAX_CARRY_FRAMES) {
                                // Regular carry-forward for non-zero bad detections
                                smoothedBody.joints[j].position = prevSmoothedBody.joints[j].position;
                                smoothedBody.joints[j].orientation = prevSmoothedBody.joints[j].orientation;
                                smoothedBody.joints[j].confidence = prevSmoothedBody.joints[j].confidence;
                                jointCarryCount[j]++;
                            } else {
                                jointCarryCount[j] = 0;
                            }
                        }
                    }

                    prevSmoothedBody = smoothedBody;
                    hasPrevSmoothed = true;
                    prevSmoothedTimestamp = curTs;

                    vector<FusedBody> singleBody = {smoothedBody};
                    joints3D = TransformSkeletonToCamera(singleBody, helmetPose, 0);
                    joints2D = ProjectSkeleton(joints3D, helmetCalibration);
                }
            }

            // Generate ego-view output (shared between modes)
            ostringstream filename;
            filename << "frame_" << setw(6) << setfill('0') << egoFrameCount;

            string imagePath = egoOutputDir + "/images/" + filename.str() + ".jpg";
            string jsonPath = egoOutputDir + "/annotations/" + filename.str() + ".json";

            cv::Mat outputImage;
            if (processors[helmetIdx].lastColorImage.channels() == 4) {
                cv::cvtColor(processors[helmetIdx].lastColorImage, outputImage, cv::COLOR_BGRA2BGR);
            } else {
                outputImage = processors[helmetIdx].lastColorImage;
            }

            if (!outputImage.empty()) {
                vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 95};
                cv::imwrite(imagePath, outputImage, jpegParams);
            }

            // Save annotation JSON
            WriteEgoFrameJson(jsonPath, egoFrameCount,
                              egoTimestamp,
                              detectionSuccess, detectionCamera,
                              helmetPose, joints3D, joints2D,
                              filename.str() + ".jpg",
                              numBodies,
                              &helmetCalibration);

            if (detectionSuccess) cbDetectedCount++;
            egoFrameCount++;
        }

        // Advance the most-behind processor to its next frame
        ProcessNextFrame(processors[nextIdx]);

        // Check if all are EOF
        allEOF = true;
        for (const auto& proc : processors) {
            if (!proc.isEOF) allEOF = false;
        }
      } catch (const std::exception& e) {
        cerr << "\n*** EXCEPTION at frame " << frameCount << " (ego: " << egoFrameCount
             << "): " << e.what() << endl;
        cerr.flush();
        break;
      } catch (...) {
        cerr << "\n*** UNKNOWN EXCEPTION at frame " << frameCount << " (ego: " << egoFrameCount << ")" << endl;
        cerr.flush();
        break;
      }
    }

    cout << "\nProcessing loop finished. Fused frames: " << frameCount
         << ", Ego frames: " << egoFrameCount << endl;
    cout.flush();

    // Write ego metadata
    if (egoMode) {
        cout << "Writing ego metadata..." << endl;
        cout.flush();

        json metadata;
        metadata["total_frames"] = egoFrameCount;
        metadata["checkerboard_detected_frames"] = cbDetectedCount;
        metadata["helmet_serial"] = helmetSerial;
        metadata["checkerboard_size"] = {helmetCBConfig.rows, helmetCBConfig.cols};
        metadata["checkerboard_square_mm"] = helmetCBConfig.squareMm;
        metadata["t_checker_to_a_path"] = tCheckerToAPath;
        metadata["calibration_path"] = calibrationPath;
        metadata["ego_fusion_mode"] = egoFusionMode;

        json mkvList = json::array();
        for (size_t i = 0; i < mkvPaths.size(); i++) {
            mkvList.push_back({
                {"index", i},
                {"path", mkvPaths[i]},
                {"serial", processors[i].serialNumber},
                {"is_helmet", processors[i].isHelmet}
            });
        }
        metadata["mkv_files"] = mkvList;

        ofstream metaFile(egoOutputDir + "/metadata.json");
        metaFile << setw(2) << metadata << endl;
        cout << "Ego metadata written." << endl;
        cout.flush();
    }

    // Cleanup
    csvFile.close();

    for (size_t i = 0; i < processors.size(); i++) {
        cout << "Closing processor " << i << " (SN: " << processors[i].serialNumber << ")..." << endl;
        cout.flush();
        CloseMkvProcessor(processors[i]);
    }

    cout << "\n========================================" << endl;
    cout << "Processing complete!" << endl;
    cout << "Total fused frames: " << frameCount << endl;
    cout << "Output: " << outputPath << endl;

    if (egoMode) {
        cout << "Ego frames: " << egoFrameCount << endl;
        cout << "Checkerboard detected: " << cbDetectedCount << " / " << egoFrameCount << " frames" << endl;
        cout << "Ego output: " << egoOutputDir << endl;
    }

    cout << "========================================" << endl;

    return 0;
}
