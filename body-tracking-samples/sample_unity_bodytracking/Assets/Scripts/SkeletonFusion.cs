using System;
using System.Collections.Generic;
using System.Numerics;
using Microsoft.Azure.Kinect.BodyTracking;
using UnityEngine;

public enum FusionMode
{
    WinnerTakesAll,
    WeightedAverage
}

public struct BodyMatch
{
    public int primaryBodyIndex;
    public int[] secondaryBodyIndices;  // Index per device, -1 if not matched
}

public static class SkeletonFusion
{
    // Pelvis joint index (K4ABT_JOINT_PELVIS = 0)
    public const int PELVIS_JOINT = 0;

    // Match threshold in meters (500mm)
    public const float MATCH_THRESHOLD = 0.5f;

    // Confidence weights
    private static readonly Dictionary<JointConfidenceLevel, float> ConfidenceWeights = new Dictionary<JointConfidenceLevel, float>
    {
        { JointConfidenceLevel.None, 0.0f },
        { JointConfidenceLevel.Low, 0.25f },
        { JointConfidenceLevel.Medium, 0.6f },
        { JointConfidenceLevel.High, 1.0f }
    };

    /// <summary>
    /// Transform a 3D point from camera space to primary camera space
    /// P_primary = R * P_camera + t
    /// </summary>
    public static System.Numerics.Vector3 TransformPoint(
        System.Numerics.Vector3 point,
        CameraCalibration calibration)
    {
        if (calibration.rotationFlat == null || calibration.translation == null)
            return point;

        // Convert point from meters to mm for transformation, then back
        float px = point.X * 1000f;
        float py = point.Y * 1000f;
        float pz = point.Z * 1000f;

        float[] R = calibration.rotationFlat;
        float[] t = calibration.translation;

        // R * P + t
        float rx = R[0] * px + R[1] * py + R[2] * pz + t[0];
        float ry = R[3] * px + R[4] * py + R[5] * pz + t[1];
        float rz = R[6] * px + R[7] * py + R[8] * pz + t[2];

        // Convert back to meters
        return new System.Numerics.Vector3(rx / 1000f, ry / 1000f, rz / 1000f);
    }

    /// <summary>
    /// Transform a quaternion orientation from camera space to primary camera space
    /// </summary>
    public static System.Numerics.Quaternion TransformOrientation(
        System.Numerics.Quaternion orientation,
        CameraCalibration calibration)
    {
        if (calibration.rotation == null)
            return orientation;

        // Convert rotation matrix to quaternion
        System.Numerics.Quaternion rotQuat = RotationMatrixToQuaternion(calibration.rotationFlat);

        // Apply rotation: q_result = q_rotation * q_original
        return System.Numerics.Quaternion.Concatenate(orientation, rotQuat);
    }

    /// <summary>
    /// Convert 3x3 rotation matrix (row-major flat array) to quaternion
    /// </summary>
    private static System.Numerics.Quaternion RotationMatrixToQuaternion(float[] R)
    {
        if (R == null || R.Length != 9)
            return System.Numerics.Quaternion.Identity;

        float m00 = R[0], m01 = R[1], m02 = R[2];
        float m10 = R[3], m11 = R[4], m12 = R[5];
        float m20 = R[6], m21 = R[7], m22 = R[8];

        float trace = m00 + m11 + m22;
        float qw, qx, qy, qz;

        if (trace > 0)
        {
            float s = 0.5f / (float)Math.Sqrt(trace + 1.0f);
            qw = 0.25f / s;
            qx = (m21 - m12) * s;
            qy = (m02 - m20) * s;
            qz = (m10 - m01) * s;
        }
        else if (m00 > m11 && m00 > m22)
        {
            float s = 2.0f * (float)Math.Sqrt(1.0f + m00 - m11 - m22);
            qw = (m21 - m12) / s;
            qx = 0.25f * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            float s = 2.0f * (float)Math.Sqrt(1.0f + m11 - m00 - m22);
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = 0.25f * s;
            qz = (m12 + m21) / s;
        }
        else
        {
            float s = 2.0f * (float)Math.Sqrt(1.0f + m22 - m00 - m11);
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25f * s;
        }

        return new System.Numerics.Quaternion(qx, qy, qz, qw);
    }

    /// <summary>
    /// Transform an entire body to primary camera coordinate frame
    /// </summary>
    public static Body TransformBody(Body body, CameraCalibration calibration)
    {
        Body transformed = Body.DeepCopy(body);

        for (int i = 0; i < body.Length; i++)
        {
            transformed.JointPositions3D[i] = TransformPoint(body.JointPositions3D[i], calibration);
            transformed.JointRotations[i] = TransformOrientation(body.JointRotations[i], calibration);
        }

        return transformed;
    }

    /// <summary>
    /// Match bodies across multiple cameras using pelvis proximity
    /// </summary>
    public static List<BodyMatch> MatchBodiesAcrossCameras(
        Body[][] bodiesPerDevice,
        int[] bodyCountsPerDevice,
        int numDevices)
    {
        List<BodyMatch> matches = new List<BodyMatch>();

        if (numDevices == 0 || bodyCountsPerDevice[0] == 0)
            return matches;

        // Track which bodies from secondary devices are already matched
        bool[][] matched = new bool[numDevices][];
        for (int d = 0; d < numDevices; d++)
        {
            matched[d] = new bool[bodyCountsPerDevice[d]];
        }

        // Use primary camera bodies as anchors
        for (int primaryIdx = 0; primaryIdx < bodyCountsPerDevice[0]; primaryIdx++)
        {
            BodyMatch match = new BodyMatch
            {
                primaryBodyIndex = primaryIdx,
                secondaryBodyIndices = new int[numDevices]
            };
            match.secondaryBodyIndices[0] = primaryIdx;

            System.Numerics.Vector3 primaryPelvis = bodiesPerDevice[0][primaryIdx].JointPositions3D[PELVIS_JOINT];

            // Find matching body in each secondary camera
            for (int devIdx = 1; devIdx < numDevices; devIdx++)
            {
                match.secondaryBodyIndices[devIdx] = -1;
                float minDist = MATCH_THRESHOLD;

                for (int bodyIdx = 0; bodyIdx < bodyCountsPerDevice[devIdx]; bodyIdx++)
                {
                    if (matched[devIdx][bodyIdx])
                        continue;

                    System.Numerics.Vector3 secondaryPelvis = bodiesPerDevice[devIdx][bodyIdx].JointPositions3D[PELVIS_JOINT];
                    float dist = System.Numerics.Vector3.Distance(primaryPelvis, secondaryPelvis);

                    if (dist < minDist)
                    {
                        minDist = dist;
                        match.secondaryBodyIndices[devIdx] = bodyIdx;
                    }
                }

                // Mark as matched
                if (match.secondaryBodyIndices[devIdx] >= 0)
                {
                    matched[devIdx][match.secondaryBodyIndices[devIdx]] = true;
                }
            }

            matches.Add(match);
        }

        // Add unmatched bodies from secondary cameras as separate entries
        for (int devIdx = 1; devIdx < numDevices; devIdx++)
        {
            for (int bodyIdx = 0; bodyIdx < bodyCountsPerDevice[devIdx]; bodyIdx++)
            {
                if (!matched[devIdx][bodyIdx])
                {
                    BodyMatch match = new BodyMatch
                    {
                        primaryBodyIndex = -1,
                        secondaryBodyIndices = new int[numDevices]
                    };

                    for (int d = 0; d < numDevices; d++)
                    {
                        match.secondaryBodyIndices[d] = (d == devIdx) ? bodyIdx : -1;
                    }

                    matches.Add(match);
                }
            }
        }

        return matches;
    }

    /// <summary>
    /// Fuse matched bodies using Winner-Takes-All mode
    /// </summary>
    public static Body FuseBodyWinnerTakesAll(
        Body[][] bodiesPerDevice,
        int numDevices,
        BodyMatch match)
    {
        int jointCount = Microsoft.Azure.Kinect.BodyTracking.Skeleton.JointCount;
        Body fusedBody = new Body(jointCount);
        fusedBody.Length = jointCount;
        fusedBody.Id = GetBestBodyId(bodiesPerDevice, numDevices, match);

        for (int j = 0; j < jointCount; j++)
        {
            JointConfidenceLevel bestConfidence = JointConfidenceLevel.None;
            int bestDevice = -1;
            int bestBodyIdx = -1;

            // Find best joint across all devices
            for (int d = 0; d < numDevices; d++)
            {
                int bodyIdx = (d == 0) ? match.primaryBodyIndex : match.secondaryBodyIndices[d];
                if (bodyIdx < 0)
                    continue;

                JointConfidenceLevel conf = bodiesPerDevice[d][bodyIdx].JointPrecisions[j];
                if (conf > bestConfidence)
                {
                    bestConfidence = conf;
                    bestDevice = d;
                    bestBodyIdx = bodyIdx;
                }
            }

            if (bestDevice >= 0)
            {
                fusedBody.JointPositions3D[j] = bodiesPerDevice[bestDevice][bestBodyIdx].JointPositions3D[j];
                fusedBody.JointRotations[j] = bodiesPerDevice[bestDevice][bestBodyIdx].JointRotations[j];
                fusedBody.JointPrecisions[j] = bestConfidence;
            }
            else
            {
                // No valid joint found
                fusedBody.JointPositions3D[j] = System.Numerics.Vector3.Zero;
                fusedBody.JointRotations[j] = System.Numerics.Quaternion.Identity;
                fusedBody.JointPrecisions[j] = JointConfidenceLevel.None;
            }
        }

        return fusedBody;
    }

    /// <summary>
    /// Fuse matched bodies using Weighted Average mode
    /// </summary>
    public static Body FuseBodyWeightedAverage(
        Body[][] bodiesPerDevice,
        int numDevices,
        BodyMatch match)
    {
        int jointCount = Microsoft.Azure.Kinect.BodyTracking.Skeleton.JointCount;
        Body fusedBody = new Body(jointCount);
        fusedBody.Length = jointCount;
        fusedBody.Id = GetBestBodyId(bodiesPerDevice, numDevices, match);

        for (int j = 0; j < jointCount; j++)
        {
            float totalWeight = 0;
            System.Numerics.Vector3 weightedPos = System.Numerics.Vector3.Zero;
            System.Numerics.Quaternion bestQuat = System.Numerics.Quaternion.Identity;
            JointConfidenceLevel maxConfidence = JointConfidenceLevel.None;
            float maxWeight = 0;

            // Accumulate weighted positions
            for (int d = 0; d < numDevices; d++)
            {
                int bodyIdx = (d == 0) ? match.primaryBodyIndex : match.secondaryBodyIndices[d];
                if (bodyIdx < 0)
                    continue;

                JointConfidenceLevel conf = bodiesPerDevice[d][bodyIdx].JointPrecisions[j];
                float weight = GetConfidenceWeight(conf);

                if (weight > 0)
                {
                    weightedPos += bodiesPerDevice[d][bodyIdx].JointPositions3D[j] * weight;
                    totalWeight += weight;

                    // Keep quaternion from highest confidence source
                    if (weight > maxWeight)
                    {
                        maxWeight = weight;
                        bestQuat = bodiesPerDevice[d][bodyIdx].JointRotations[j];
                    }
                }

                if (conf > maxConfidence)
                    maxConfidence = conf;
            }

            if (totalWeight > 0)
            {
                fusedBody.JointPositions3D[j] = weightedPos / totalWeight;
                fusedBody.JointRotations[j] = bestQuat;
                fusedBody.JointPrecisions[j] = maxConfidence;
            }
            else
            {
                fusedBody.JointPositions3D[j] = System.Numerics.Vector3.Zero;
                fusedBody.JointRotations[j] = System.Numerics.Quaternion.Identity;
                fusedBody.JointPrecisions[j] = JointConfidenceLevel.None;
            }
        }

        return fusedBody;
    }

    /// <summary>
    /// Get the body ID from the best available source
    /// </summary>
    private static uint GetBestBodyId(Body[][] bodiesPerDevice, int numDevices, BodyMatch match)
    {
        if (match.primaryBodyIndex >= 0)
            return bodiesPerDevice[0][match.primaryBodyIndex].Id;

        for (int d = 1; d < numDevices; d++)
        {
            if (match.secondaryBodyIndices[d] >= 0)
                return bodiesPerDevice[d][match.secondaryBodyIndices[d]].Id;
        }

        return 0;
    }

    /// <summary>
    /// Get weight for a confidence level
    /// </summary>
    public static float GetConfidenceWeight(JointConfidenceLevel confidence)
    {
        if (ConfidenceWeights.TryGetValue(confidence, out float weight))
            return weight;
        return 0.0f;
    }

    /// <summary>
    /// Perform full skeleton fusion pipeline
    /// </summary>
    public static Body[] FuseSkeletons(
        Body[][] rawBodiesPerDevice,
        int[] bodyCountsPerDevice,
        CalibrationData calibration,
        FusionMode fusionMode,
        out int fusedBodyCount)
    {
        fusedBodyCount = 0;

        if (calibration == null || calibration.calibrations == null)
            return null;

        int numDevices = calibration.num_devices;
        if (numDevices == 0)
            return null;

        // Step 1: Transform all bodies to primary camera frame
        Body[][] transformedBodies = new Body[numDevices][];
        for (int d = 0; d < numDevices; d++)
        {
            transformedBodies[d] = new Body[bodyCountsPerDevice[d]];

            // Find calibration for this device
            CameraCalibration cal = null;
            foreach (var c in calibration.calibrations)
            {
                if (c.device_index == d)
                {
                    cal = c;
                    break;
                }
            }

            for (int b = 0; b < bodyCountsPerDevice[d]; b++)
            {
                if (cal != null && cal.is_valid)
                {
                    transformedBodies[d][b] = TransformBody(rawBodiesPerDevice[d][b], cal);
                }
                else if (d == 0)
                {
                    // Primary camera: identity transform
                    transformedBodies[d][b] = Body.DeepCopy(rawBodiesPerDevice[d][b]);
                }
                else
                {
                    // No calibration: use raw (may be incorrect)
                    transformedBodies[d][b] = Body.DeepCopy(rawBodiesPerDevice[d][b]);
                }
            }
        }

        // Step 2: Match bodies across cameras
        List<BodyMatch> matches = MatchBodiesAcrossCameras(transformedBodies, bodyCountsPerDevice, numDevices);

        // Step 3: Fuse matched bodies
        Body[] fusedBodies = new Body[matches.Count];
        for (int i = 0; i < matches.Count; i++)
        {
            if (fusionMode == FusionMode.WinnerTakesAll)
            {
                fusedBodies[i] = FuseBodyWinnerTakesAll(transformedBodies, numDevices, matches[i]);
            }
            else
            {
                fusedBodies[i] = FuseBodyWeightedAverage(transformedBodies, numDevices, matches[i]);
            }
        }

        fusedBodyCount = matches.Count;
        return fusedBodies;
    }
}
