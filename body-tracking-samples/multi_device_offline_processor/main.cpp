// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Multi-device offline body tracking processor
// Processes MKV recordings from multiple cameras with skeleton fusion

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <iomanip>
#include <cmath>
#include <algorithm>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4abt.h>
#include <nlohmann/json.hpp>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>

using namespace std;
using json = nlohmann::json;

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
};

struct FrameData {
    uint64_t timestamp_usec;
    int deviceIndex;
    vector<k4abt_body_t> bodies;
};

// ============================================================================
// Global Configuration
// ============================================================================
CalibrationData g_calibration = {0, {}, false};
const float BODY_MATCH_THRESHOLD_MM = 500.0f;

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
        case K4ABT_JOINT_CONFIDENCE_LOW:    return 0.25f;
        case K4ABT_JOINT_CONFIDENCE_MEDIUM: return 0.6f;
        case K4ABT_JOINT_CONFIDENCE_HIGH:   return 1.0f;
        default: return 0.0f;
    }
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
};

bool InitMkvProcessor(MkvProcessor& proc, const string& filepath, int deviceIndex,
                       k4abt_tracker_configuration_t trackerConfig)
{
    proc.filepath = filepath;
    proc.deviceIndex = deviceIndex;

    // Open playback
    k4a_result_t result = k4a_playback_open(filepath.c_str(), &proc.playback);
    if (result != K4A_RESULT_SUCCEEDED) {
        cerr << "Failed to open: " << filepath << endl;
        return false;
    }

    // Get calibration
    k4a_calibration_t calibration;
    result = k4a_playback_get_calibration(proc.playback, &calibration);
    if (result != K4A_RESULT_SUCCEEDED) {
        cerr << "Failed to get calibration from: " << filepath << endl;
        k4a_playback_close(proc.playback);
        return false;
    }

    // Create body tracker
    result = k4abt_tracker_create(&calibration, trackerConfig, &proc.tracker);
    if (result != K4A_RESULT_SUCCEEDED) {
        cerr << "Failed to create tracker for: " << filepath << endl;
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

    cout << "Initialized processor for device " << deviceIndex
         << " (SN: " << proc.serialNumber << "): " << filepath << endl;

    return true;
}

bool ProcessNextFrame(MkvProcessor& proc)
{
    if (proc.isEOF) return false;

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

    // Check if capture has depth image
    k4a_image_t depth = k4a_capture_get_depth_image(capture);
    if (depth == nullptr) {
        k4a_capture_release(capture);
        return true; // Skip frame, but continue processing
    }

    // Enqueue capture
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

    // Extract data
    proc.lastTimestamp = k4abt_frame_get_device_timestamp_usec(bodyFrame);
    proc.lastBodies.clear();

    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    for (uint32_t i = 0; i < numBodies; i++) {
        k4abt_body_t body;
        if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) == K4A_RESULT_SUCCEEDED) {
            body.id = k4abt_frame_get_body_id(bodyFrame, i);
            proc.lastBodies.push_back(body);
        }
    }

    k4abt_frame_release(bodyFrame);
    return true;
}

void CloseMkvProcessor(MkvProcessor& proc)
{
    if (proc.tracker) {
        k4abt_tracker_shutdown(proc.tracker);
        k4abt_tracker_destroy(proc.tracker);
        proc.tracker = nullptr;
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
         << "Processes MKV recordings from multiple cameras with skeleton fusion.\n\n"
         << "USAGE:\n"
         << "  multi_device_offline_processor.exe [OPTIONS] <mkv1> <mkv2> ...\n\n"
         << "OPTIONS:\n"
         << "  --calibration FILE   - Calibration JSON file (required for fusion)\n"
         << "  --output FILE        - Output CSV file (default: output.csv)\n"
         << "  --mode MODE          - Processing mode: CPU, CUDA, DirectML (default), TensorRT\n"
         << "  --sync-threshold MS  - Max timestamp difference for sync (default: 33ms)\n"
         << "  --help               - Show this help\n\n"
         << "EXAMPLE:\n"
         << "  multi_device_offline_processor.exe --calibration calib.json \\\n"
         << "      --output skeleton.csv recording_cam0.mkv recording_cam1.mkv\n"
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
    uint64_t syncThresholdUs = 33000; // 33ms default

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

    cout << "Input MKV files: " << mkvPaths.size() << endl;
    for (size_t i = 0; i < mkvPaths.size(); i++) {
        cout << "  [" << i << "] " << mkvPaths[i] << endl;
    }

    // Load calibration
    if (!calibrationPath.empty()) {
        cout << "\nLoading calibration: " << calibrationPath << endl;
        if (!LoadCalibration(calibrationPath, g_calibration)) {
            cerr << "Warning: Failed to load calibration, fusion disabled" << endl;
        }
    } else if (mkvPaths.size() > 1) {
        cout << "\nWarning: Multiple MKV files but no calibration specified!" << endl;
        cout << "Bodies will NOT be fused. Use --calibration for fusion." << endl;
    }

    // Initialize processors
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = processingMode;

    vector<MkvProcessor> processors(mkvPaths.size());
    for (size_t i = 0; i < mkvPaths.size(); i++) {
        if (!InitMkvProcessor(processors[i], mkvPaths[i], (int)i, trackerConfig)) {
            cerr << "Failed to initialize processor " << i << endl;
            return -1;
        }
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

    while (!allEOF) {
        // Advance each processor
        for (auto& proc : processors) {
            if (!proc.isEOF) {
                ProcessNextFrame(proc);
            }
        }

        // Check if all are EOF
        allEOF = true;
        for (const auto& proc : processors) {
            if (!proc.isEOF) allEOF = false;
        }

        if (allEOF) break;

        // Find minimum timestamp among non-EOF processors
        uint64_t minTimestamp = UINT64_MAX;
        for (const auto& proc : processors) {
            if (!proc.isEOF && proc.lastTimestamp < minTimestamp) {
                minTimestamp = proc.lastTimestamp;
            }
        }

        // Collect frames within sync threshold
        vector<FrameData> syncFrames;
        for (auto& proc : processors) {
            if (proc.isEOF) continue;

            // Check if within sync threshold
            if (proc.lastTimestamp <= minTimestamp + syncThresholdUs) {
                FrameData fd;
                fd.timestamp_usec = proc.lastTimestamp;
                fd.deviceIndex = proc.deviceIndex;
                fd.bodies = proc.lastBodies;
                syncFrames.push_back(fd);
            }
        }

        // Fuse bodies
        if (!syncFrames.empty()) {
            vector<FusedBody> fused;

            if (g_calibration.isLoaded && syncFrames.size() > 1) {
                fused = FuseBodiesAtTimestamp(syncFrames);
            } else {
                // No fusion - just use bodies from first frame
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

            // Write to CSV
            for (const auto& body : fused) {
                WriteFusedBodyToCSV(csvFile, minTimestamp, body);
            }

            frameCount++;
            if (frameCount % 100 == 0) {
                cout << "Processed " << frameCount << " frames..." << endl;
            }
        }
    }

    // Cleanup
    csvFile.close();

    for (auto& proc : processors) {
        CloseMkvProcessor(proc);
    }

    cout << "\n========================================" << endl;
    cout << "Processing complete!" << endl;
    cout << "Total frames: " << frameCount << endl;
    cout << "Output: " << outputPath << endl;
    cout << "========================================" << endl;

    return 0;
}
