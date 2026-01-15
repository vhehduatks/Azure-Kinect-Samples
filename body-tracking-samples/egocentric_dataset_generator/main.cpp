// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Egocentric Body Tracking Dataset Generator
// Generates ML training data from helmet-mounted camera with skeleton labels

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <optional>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <BodyTrackingHelpers.h>

using namespace std;
using json = nlohmann::json;
namespace fs = std::filesystem;

// ============================================================================
// Configuration
// ============================================================================
struct Config {
    string camera_a_path;           // Helmet camera MKV
    string camera_b_path;           // Fixed camera B MKV
    string camera_c_path;           // Fixed camera C MKV (optional)
    string skeleton_path;           // skeleton.csv
    string calibration_path;        // calibration.json
    string t_checker_to_a_path;     // T_checker_to_A.json
    string output_dir;              // Output directory

    int checkerboard_rows = 6;      // Inner corners rows
    int checkerboard_cols = 9;      // Inner corners cols
    float checkerboard_square_mm = 25.0f;

    uint64_t sync_threshold_usec = 10000;  // 10ms
    int jpeg_quality = 95;
    int skip_frames = 0;
    int max_frames = 0;             // 0 = all
    bool verbose = false;
};

// ============================================================================
// Data Structures
// ============================================================================
struct Transform {
    cv::Mat rotation;       // 3x3 rotation matrix
    cv::Mat translation;    // 3x1 translation vector
    bool valid = false;
};

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
    bool isLoaded = false;
};

struct Joint3D {
    float x, y, z;
    int confidence;
    string name;
};

struct Joint2D {
    float u, v;
    int confidence;
    bool visible;
    string name;
};

struct SkeletonFrame {
    uint64_t timestamp_usec;
    uint32_t body_id;
    vector<Joint3D> joints;
};

struct CameraFrame {
    uint64_t timestamp_usec;
    cv::Mat color_image;
    cv::Mat depth_image;
    k4a_calibration_t calibration;
    bool valid = false;
};

struct OutputFrame {
    int frame_id;
    uint64_t timestamp_usec;
    string image_file;
    Transform camera_pose;
    vector<Joint3D> skeleton_3d;
    vector<Joint2D> skeleton_2d;
    bool checkerboard_detected;
};

// ============================================================================
// MKV Reader
// ============================================================================
class MkvReader {
public:
    bool Open(const string& path) {
        if (k4a_playback_open(path.c_str(), &playback_) != K4A_RESULT_SUCCEEDED) {
            cerr << "Failed to open: " << path << endl;
            return false;
        }

        if (k4a_playback_get_calibration(playback_, &calibration_) != K4A_RESULT_SUCCEEDED) {
            cerr << "Failed to get calibration from: " << path << endl;
            Close();
            return false;
        }

        // Create transformation handle for depth-to-color
        transformation_ = k4a_transformation_create(&calibration_);

        path_ = path;
        is_open_ = true;
        return true;
    }

    void Close() {
        if (transformation_) {
            k4a_transformation_destroy(transformation_);
            transformation_ = nullptr;
        }
        if (playback_) {
            k4a_playback_close(playback_);
            playback_ = nullptr;
        }
        is_open_ = false;
    }

    bool GetNextFrame(CameraFrame& frame) {
        if (!is_open_) return false;

        k4a_capture_t capture = nullptr;
        k4a_stream_result_t result = k4a_playback_get_next_capture(playback_, &capture);

        if (result == K4A_STREAM_RESULT_EOF) {
            return false;
        }

        if (result != K4A_STREAM_RESULT_SUCCEEDED) {
            return false;
        }

        frame.valid = false;
        frame.calibration = calibration_;

        // Get color image
        k4a_image_t color = k4a_capture_get_color_image(capture);
        if (color) {
            frame.timestamp_usec = k4a_image_get_device_timestamp_usec(color);

            int width = k4a_image_get_width_pixels(color);
            int height = k4a_image_get_height_pixels(color);
            k4a_image_format_t format = k4a_image_get_format(color);

            if (format == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
                frame.color_image = cv::Mat(height, width, CV_8UC4,
                    k4a_image_get_buffer(color)).clone();
            } else if (format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
                // Decode MJPG
                vector<uint8_t> buffer(k4a_image_get_buffer(color),
                    k4a_image_get_buffer(color) + k4a_image_get_size(color));
                frame.color_image = cv::imdecode(buffer, cv::IMREAD_COLOR);
            }

            k4a_image_release(color);
            frame.valid = true;
        }

        // Get depth image
        k4a_image_t depth = k4a_capture_get_depth_image(capture);
        if (depth) {
            int width = k4a_image_get_width_pixels(depth);
            int height = k4a_image_get_height_pixels(depth);
            frame.depth_image = cv::Mat(height, width, CV_16UC1,
                k4a_image_get_buffer(depth)).clone();
            k4a_image_release(depth);
        }

        k4a_capture_release(capture);
        return frame.valid;
    }

    k4a_calibration_t GetCalibration() const { return calibration_; }
    k4a_transformation_t GetTransformation() const { return transformation_; }
    bool IsOpen() const { return is_open_; }

    ~MkvReader() { Close(); }

private:
    k4a_playback_t playback_ = nullptr;
    k4a_calibration_t calibration_;
    k4a_transformation_t transformation_ = nullptr;
    string path_;
    bool is_open_ = false;
};

// ============================================================================
// Skeleton Loader (CSV)
// ============================================================================
class SkeletonLoader {
public:
    bool Load(const string& path) {
        ifstream file(path);
        if (!file.is_open()) {
            cerr << "Failed to open skeleton CSV: " << path << endl;
            return false;
        }

        string line;
        getline(file, line);  // Skip header

        while (getline(file, line)) {
            SkeletonFrame frame;
            if (ParseLine(line, frame)) {
                frames_.push_back(frame);
            }
        }

        cout << "Loaded " << frames_.size() << " skeleton frames" << endl;
        return !frames_.empty();
    }

    optional<SkeletonFrame> GetFrameAtTimestamp(uint64_t timestamp_usec,
                                                 uint64_t threshold_usec) {
        // Find closest frame
        SkeletonFrame* best = nullptr;
        uint64_t best_diff = UINT64_MAX;

        for (auto& frame : frames_) {
            uint64_t diff = (frame.timestamp_usec > timestamp_usec) ?
                (frame.timestamp_usec - timestamp_usec) :
                (timestamp_usec - frame.timestamp_usec);

            if (diff < best_diff && diff <= threshold_usec) {
                best_diff = diff;
                best = &frame;
            }
        }

        if (best) return *best;
        return nullopt;
    }

    // Linear interpolation between two frames
    SkeletonFrame Interpolate(const SkeletonFrame& before,
                               const SkeletonFrame& after,
                               uint64_t target_timestamp) {
        double t = static_cast<double>(target_timestamp - before.timestamp_usec) /
                   static_cast<double>(after.timestamp_usec - before.timestamp_usec);
        t = max(0.0, min(1.0, t));

        SkeletonFrame result;
        result.timestamp_usec = target_timestamp;
        result.body_id = before.body_id;
        result.joints.resize(before.joints.size());

        for (size_t i = 0; i < before.joints.size(); i++) {
            result.joints[i].x = before.joints[i].x * (1.0 - t) + after.joints[i].x * t;
            result.joints[i].y = before.joints[i].y * (1.0 - t) + after.joints[i].y * t;
            result.joints[i].z = before.joints[i].z * (1.0 - t) + after.joints[i].z * t;
            result.joints[i].confidence = min(before.joints[i].confidence,
                                               after.joints[i].confidence);
            result.joints[i].name = before.joints[i].name;
        }

        return result;
    }

private:
    bool ParseLine(const string& line, SkeletonFrame& frame) {
        stringstream ss(line);
        string token;

        // timestamp_usec,body_id,J0_x,J0_y,J0_z,J0_conf,...
        getline(ss, token, ',');
        frame.timestamp_usec = stoull(token);

        getline(ss, token, ',');
        frame.body_id = stoul(token);

        frame.joints.resize(K4ABT_JOINT_COUNT);

        for (int j = 0; j < K4ABT_JOINT_COUNT; j++) {
            getline(ss, token, ',');
            frame.joints[j].x = stof(token);

            getline(ss, token, ',');
            frame.joints[j].y = stof(token);

            getline(ss, token, ',');
            frame.joints[j].z = stof(token);

            getline(ss, token, ',');
            frame.joints[j].confidence = stoi(token);

            auto it = g_jointNames.find(static_cast<k4abt_joint_id_t>(j));
            frame.joints[j].name = (it != g_jointNames.end()) ? it->second : "UNKNOWN";
        }

        return true;
    }

    vector<SkeletonFrame> frames_;
};

// ============================================================================
// Calibration Loader
// ============================================================================
bool LoadCalibration(const string& path, CalibrationData& cal) {
    ifstream file(path);
    if (!file.is_open()) {
        cerr << "Failed to open calibration: " << path << endl;
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
                // Identity
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
        return true;
    } catch (const exception& e) {
        cerr << "Error parsing calibration: " << e.what() << endl;
        return false;
    }
}

bool LoadTransform(const string& path, Transform& t) {
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
    } catch (const exception& e) {
        cerr << "Error parsing transform: " << e.what() << endl;
        return false;
    }
}

// ============================================================================
// Checkerboard Detection
// ============================================================================
bool DetectCheckerboardCorners(const cv::Mat& colorImage,
                                vector<cv::Point2f>& corners,
                                cv::Size patternSize) {
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
        cv::CALIB_CB_NORMALIZE_IMAGE |
        cv::CALIB_CB_FAST_CHECK);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    return found;
}

// Convert 2D corners to 3D using depth image
bool Convert2DTo3D(const cv::Mat& depthImage,
                   const vector<cv::Point2f>& corners2D,
                   const k4a_calibration_t& calibration,
                   k4a_transformation_t transformation,
                   vector<cv::Point3f>& points3D) {
    points3D.clear();

    int colorWidth = calibration.color_camera_calibration.resolution_width;
    int colorHeight = calibration.color_camera_calibration.resolution_height;

    // Transform depth to color space
    k4a_image_t depthK4a = nullptr;
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        depthImage.cols, depthImage.rows,
        depthImage.cols * sizeof(uint16_t), &depthK4a);
    memcpy(k4a_image_get_buffer(depthK4a), depthImage.data,
        depthImage.total() * sizeof(uint16_t));

    k4a_image_t transformedDepth = nullptr;
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        colorWidth, colorHeight,
        colorWidth * sizeof(uint16_t), &transformedDepth);

    k4a_transformation_depth_image_to_color_camera(transformation,
        depthK4a, transformedDepth);

    uint16_t* depthBuffer = reinterpret_cast<uint16_t*>(
        k4a_image_get_buffer(transformedDepth));

    for (const auto& corner : corners2D) {
        int x = static_cast<int>(round(corner.x));
        int y = static_cast<int>(round(corner.y));

        // Sample 3x3 neighborhood
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

// Compute checkerboard pose from 3D points
Transform ComputeCheckerboardPose(const vector<cv::Point3f>& points3D,
                                   cv::Size patternSize) {
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
    x_axis /= cv::norm(x_axis);

    // Use first column direction as Y axis
    cv::Point3f y_axis = points3D[(patternSize.height - 1) * patternSize.width] - points3D[0];
    y_axis /= cv::norm(y_axis);

    // Z axis from cross product
    cv::Point3f z_axis = x_axis.cross(y_axis);
    z_axis /= cv::norm(z_axis);

    // Re-orthogonalize Y
    y_axis = z_axis.cross(x_axis);
    y_axis /= cv::norm(y_axis);

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

// Transform points from camera space to world space
void TransformToWorld(vector<cv::Point3f>& points,
                      const CameraExtrinsics& ext) {
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
// Skeleton Transformation
// ============================================================================
vector<Joint3D> TransformSkeletonToCamera(const vector<Joint3D>& skeleton_world,
                                           const Transform& camera_pose) {
    vector<Joint3D> result(skeleton_world.size());

    // R_A^T * (P_world - t_A)
    cv::Mat R_inv = camera_pose.rotation.t();

    for (size_t i = 0; i < skeleton_world.size(); i++) {
        cv::Mat p_world = (cv::Mat_<double>(3, 1) <<
            skeleton_world[i].x, skeleton_world[i].y, skeleton_world[i].z);

        cv::Mat p_offset = p_world - camera_pose.translation;
        cv::Mat p_cam = R_inv * p_offset;

        result[i].x = p_cam.at<double>(0);
        result[i].y = p_cam.at<double>(1);
        result[i].z = p_cam.at<double>(2);
        result[i].confidence = skeleton_world[i].confidence;
        result[i].name = skeleton_world[i].name;
    }

    return result;
}

// Project 3D skeleton to 2D
vector<Joint2D> ProjectSkeleton(const vector<Joint3D>& skeleton_3d,
                                 const k4a_calibration_t& calibration) {
    vector<Joint2D> result(skeleton_3d.size());

    int width = calibration.color_camera_calibration.resolution_width;
    int height = calibration.color_camera_calibration.resolution_height;

    for (size_t i = 0; i < skeleton_3d.size(); i++) {
        const auto& joint = skeleton_3d[i];

        if (joint.z <= 0) {
            result[i].visible = false;
            result[i].confidence = 0;
            result[i].name = joint.name;
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
            result[i].confidence = joint.confidence;
            result[i].visible = (point2d.xy.x >= 0 && point2d.xy.x < width &&
                                  point2d.xy.y >= 0 && point2d.xy.y < height);
        } else {
            result[i].visible = false;
            result[i].confidence = 0;
        }
        result[i].name = joint.name;
    }

    return result;
}

// ============================================================================
// Output Writer
// ============================================================================
void WriteOutputJson(const string& path, const OutputFrame& frame) {
    json j;

    j["frame_id"] = frame.frame_id;
    j["timestamp_usec"] = frame.timestamp_usec;
    j["image_file"] = frame.image_file;
    j["checkerboard_detected"] = frame.checkerboard_detected;

    // Camera pose
    if (frame.camera_pose.valid) {
        j["camera_pose"]["R"] = json::array();
        for (int r = 0; r < 3; r++) {
            j["camera_pose"]["R"].push_back({
                frame.camera_pose.rotation.at<double>(r, 0),
                frame.camera_pose.rotation.at<double>(r, 1),
                frame.camera_pose.rotation.at<double>(r, 2)
            });
        }
        j["camera_pose"]["t"] = {
            frame.camera_pose.translation.at<double>(0),
            frame.camera_pose.translation.at<double>(1),
            frame.camera_pose.translation.at<double>(2)
        };
    }

    // 3D skeleton
    j["skeleton_3d"] = json::array();
    for (size_t i = 0; i < frame.skeleton_3d.size(); i++) {
        j["skeleton_3d"].push_back({
            {"joint_id", i},
            {"name", frame.skeleton_3d[i].name},
            {"x", frame.skeleton_3d[i].x},
            {"y", frame.skeleton_3d[i].y},
            {"z", frame.skeleton_3d[i].z},
            {"confidence", frame.skeleton_3d[i].confidence}
        });
    }

    // 2D skeleton
    j["skeleton_2d"] = json::array();
    for (size_t i = 0; i < frame.skeleton_2d.size(); i++) {
        j["skeleton_2d"].push_back({
            {"joint_id", i},
            {"name", frame.skeleton_2d[i].name},
            {"u", frame.skeleton_2d[i].u},
            {"v", frame.skeleton_2d[i].v},
            {"confidence", frame.skeleton_2d[i].confidence},
            {"visible", frame.skeleton_2d[i].visible}
        });
    }

    ofstream file(path);
    file << setw(2) << j << endl;
}

// ============================================================================
// CLI Parsing
// ============================================================================
void PrintUsage() {
    cout << R"(
Egocentric Body Tracking Dataset Generator

Usage: egocentric_dataset_generator.exe [OPTIONS]

Required:
  --camera-a <file>        MKV file from helmet camera (egocentric)
  --camera-b <file>        MKV file from fixed camera B
  --skeleton <file>        skeleton.csv from multi_device_offline_processor
  --calibration <file>     calibration.json (B, C extrinsics)
  --t-checker-to-a <file>  Fixed transform from checkerboard to Camera A

Optional:
  --camera-c <file>        MKV file from fixed camera C
  --output <dir>           Output directory (default: ./output)
  --checkerboard-rows <n>  Inner corners rows (default: 6)
  --checkerboard-cols <n>  Inner corners cols (default: 9)
  --sync-threshold <ms>    Max timestamp difference (default: 10)
  --jpeg-quality <n>       JPEG quality 1-100 (default: 95)
  --skip-frames <n>        Skip every N frames (default: 0)
  --max-frames <n>         Maximum frames to process (default: all)
  --verbose                Enable verbose logging
  --help                   Show this help

Example:
  egocentric_dataset_generator.exe \
    --camera-a helmet.mkv \
    --camera-b fixed_b.mkv \
    --skeleton skeleton.csv \
    --calibration calibration.json \
    --t-checker-to-a t_checker_a.json \
    --output ./dataset
)" << endl;
}

bool ParseArgs(int argc, char** argv, Config& config) {
    for (int i = 1; i < argc; i++) {
        string arg(argv[i]);

        if (arg == "--camera-a" && i + 1 < argc) {
            config.camera_a_path = argv[++i];
        } else if (arg == "--camera-b" && i + 1 < argc) {
            config.camera_b_path = argv[++i];
        } else if (arg == "--camera-c" && i + 1 < argc) {
            config.camera_c_path = argv[++i];
        } else if (arg == "--skeleton" && i + 1 < argc) {
            config.skeleton_path = argv[++i];
        } else if (arg == "--calibration" && i + 1 < argc) {
            config.calibration_path = argv[++i];
        } else if (arg == "--t-checker-to-a" && i + 1 < argc) {
            config.t_checker_to_a_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            config.output_dir = argv[++i];
        } else if (arg == "--checkerboard-rows" && i + 1 < argc) {
            config.checkerboard_rows = stoi(argv[++i]);
        } else if (arg == "--checkerboard-cols" && i + 1 < argc) {
            config.checkerboard_cols = stoi(argv[++i]);
        } else if (arg == "--sync-threshold" && i + 1 < argc) {
            config.sync_threshold_usec = stoull(argv[++i]) * 1000;
        } else if (arg == "--jpeg-quality" && i + 1 < argc) {
            config.jpeg_quality = stoi(argv[++i]);
        } else if (arg == "--skip-frames" && i + 1 < argc) {
            config.skip_frames = stoi(argv[++i]);
        } else if (arg == "--max-frames" && i + 1 < argc) {
            config.max_frames = stoi(argv[++i]);
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return false;
        } else {
            cerr << "Unknown argument: " << arg << endl;
            PrintUsage();
            return false;
        }
    }

    // Validate required arguments
    if (config.camera_a_path.empty() || config.camera_b_path.empty() ||
        config.skeleton_path.empty() || config.calibration_path.empty() ||
        config.t_checker_to_a_path.empty()) {
        cerr << "Missing required arguments!" << endl;
        PrintUsage();
        return false;
    }

    if (config.output_dir.empty()) {
        config.output_dir = "./output";
    }

    return true;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
    cout << "\n========================================" << endl;
    cout << "Egocentric Dataset Generator" << endl;
    cout << "========================================\n" << endl;

    Config config;
    if (!ParseArgs(argc, argv, config)) {
        return -1;
    }

    // Create output directories
    fs::create_directories(config.output_dir + "/images");
    fs::create_directories(config.output_dir + "/annotations");

    // Load calibration
    CalibrationData calibration;
    if (!LoadCalibration(config.calibration_path, calibration)) {
        return -1;
    }
    cout << "Loaded calibration for " << calibration.numDevices << " cameras" << endl;

    // Load T_checker_to_A
    Transform t_checker_to_a;
    if (!LoadTransform(config.t_checker_to_a_path, t_checker_to_a)) {
        return -1;
    }
    cout << "Loaded T_checker_to_A transform" << endl;

    // Open MKV files
    MkvReader reader_a, reader_b, reader_c;

    if (!reader_a.Open(config.camera_a_path)) {
        return -1;
    }
    cout << "Opened Camera A: " << config.camera_a_path << endl;

    if (!reader_b.Open(config.camera_b_path)) {
        return -1;
    }
    cout << "Opened Camera B: " << config.camera_b_path << endl;

    bool has_camera_c = !config.camera_c_path.empty();
    if (has_camera_c) {
        if (!reader_c.Open(config.camera_c_path)) {
            has_camera_c = false;
            cout << "Warning: Could not open Camera C" << endl;
        } else {
            cout << "Opened Camera C: " << config.camera_c_path << endl;
        }
    }

    // Load skeleton data
    SkeletonLoader skeleton_loader;
    if (!skeleton_loader.Load(config.skeleton_path)) {
        return -1;
    }

    // Processing loop
    cv::Size pattern_size(config.checkerboard_cols, config.checkerboard_rows);
    int frame_count = 0;
    int output_count = 0;
    int checkerboard_detected_count = 0;

    Transform last_camera_pose;
    int consecutive_misses = 0;
    const int MAX_CONSECUTIVE_MISSES = 30;

    cout << "\nProcessing frames..." << endl;

    CameraFrame frame_a, frame_b, frame_c;

    while (reader_a.GetNextFrame(frame_a)) {
        frame_count++;

        // Skip frames if requested
        if (config.skip_frames > 0 && (frame_count % (config.skip_frames + 1)) != 1) {
            continue;
        }

        // Max frames limit
        if (config.max_frames > 0 && output_count >= config.max_frames) {
            break;
        }

        // Get corresponding B, C frames
        // Note: Simple sequential read - assumes sync hub synchronization
        reader_b.GetNextFrame(frame_b);
        if (has_camera_c) {
            reader_c.GetNextFrame(frame_c);
        }

        // Get skeleton for this timestamp
        auto skeleton_opt = skeleton_loader.GetFrameAtTimestamp(
            frame_a.timestamp_usec, config.sync_threshold_usec * 10);

        if (!skeleton_opt) {
            if (config.verbose) {
                cout << "Frame " << frame_count << ": No skeleton data" << endl;
            }
            continue;
        }

        // Detect checkerboard in B and/or C
        vector<cv::Point2f> corners_b, corners_c;
        bool found_b = false, found_c = false;

        if (frame_b.valid && !frame_b.color_image.empty()) {
            found_b = DetectCheckerboardCorners(frame_b.color_image, corners_b, pattern_size);
        }

        if (has_camera_c && frame_c.valid && !frame_c.color_image.empty()) {
            found_c = DetectCheckerboardCorners(frame_c.color_image, corners_c, pattern_size);
        }

        Transform camera_pose;
        bool checkerboard_detected = false;

        if (found_b || found_c) {
            // Convert 2D to 3D and compute pose
            vector<cv::Point3f> points3D_world;

            if (found_b && !frame_b.depth_image.empty()) {
                vector<cv::Point3f> points3D_b;
                if (Convert2DTo3D(frame_b.depth_image, corners_b,
                                   frame_b.calibration, reader_b.GetTransformation(),
                                   points3D_b)) {
                    // Transform to world coordinates
                    if (calibration.cameras.size() > 1) {
                        TransformToWorld(points3D_b, calibration.cameras[1]);
                    }
                    points3D_world = points3D_b;
                    checkerboard_detected = true;
                }
            }

            if (checkerboard_detected) {
                // Compute checkerboard pose in world
                Transform checker_pose_world = ComputeCheckerboardPose(points3D_world, pattern_size);

                if (checker_pose_world.valid) {
                    // Camera A pose = checker_pose_world * t_checker_to_a
                    camera_pose.rotation = checker_pose_world.rotation * t_checker_to_a.rotation;
                    camera_pose.translation = checker_pose_world.rotation * t_checker_to_a.translation
                                             + checker_pose_world.translation;
                    camera_pose.valid = true;

                    last_camera_pose = camera_pose;
                    consecutive_misses = 0;
                    checkerboard_detected_count++;
                }
            }
        }

        // Use last known pose if checkerboard not detected
        if (!camera_pose.valid) {
            consecutive_misses++;
            if (last_camera_pose.valid && consecutive_misses <= MAX_CONSECUTIVE_MISSES) {
                camera_pose = last_camera_pose;
            } else {
                if (config.verbose) {
                    cout << "Frame " << frame_count << ": No pose (consecutive misses: "
                         << consecutive_misses << ")" << endl;
                }
                continue;
            }
        }

        // Transform skeleton to camera A coordinates
        vector<Joint3D> skeleton_3d = TransformSkeletonToCamera(
            skeleton_opt->joints, camera_pose);

        // Project to 2D
        vector<Joint2D> skeleton_2d = ProjectSkeleton(
            skeleton_3d, frame_a.calibration);

        // Generate output filename
        ostringstream filename;
        filename << "frame_" << setw(6) << setfill('0') << output_count;

        string image_path = config.output_dir + "/images/" + filename.str() + ".jpg";
        string json_path = config.output_dir + "/annotations/" + filename.str() + ".json";

        // Save image
        cv::Mat output_image;
        if (frame_a.color_image.channels() == 4) {
            cv::cvtColor(frame_a.color_image, output_image, cv::COLOR_BGRA2BGR);
        } else {
            output_image = frame_a.color_image;
        }

        vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, config.jpeg_quality};
        cv::imwrite(image_path, output_image, jpeg_params);

        // Save JSON
        OutputFrame output;
        output.frame_id = output_count;
        output.timestamp_usec = frame_a.timestamp_usec;
        output.image_file = filename.str() + ".jpg";
        output.camera_pose = camera_pose;
        output.skeleton_3d = skeleton_3d;
        output.skeleton_2d = skeleton_2d;
        output.checkerboard_detected = checkerboard_detected;

        WriteOutputJson(json_path, output);

        output_count++;

        if (output_count % 100 == 0) {
            cout << "Processed " << output_count << " frames..." << endl;
        }
    }

    // Write metadata
    json metadata;
    metadata["total_frames"] = output_count;
    metadata["checkerboard_detected_frames"] = checkerboard_detected_count;
    metadata["camera_a"] = config.camera_a_path;
    metadata["camera_b"] = config.camera_b_path;
    metadata["skeleton"] = config.skeleton_path;
    metadata["checkerboard_size"] = {config.checkerboard_rows, config.checkerboard_cols};

    ofstream meta_file(config.output_dir + "/metadata.json");
    meta_file << setw(2) << metadata << endl;

    cout << "\n========================================" << endl;
    cout << "Processing complete!" << endl;
    cout << "Total frames processed: " << output_count << endl;
    cout << "Checkerboard detected: " << checkerboard_detected_count << " frames" << endl;
    cout << "Output directory: " << config.output_dir << endl;
    cout << "========================================" << endl;

    return 0;
}
