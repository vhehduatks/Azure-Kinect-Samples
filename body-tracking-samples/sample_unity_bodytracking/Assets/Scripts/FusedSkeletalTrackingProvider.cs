using Microsoft.Azure.Kinect.BodyTracking;
using Microsoft.Azure.Kinect.Sensor;
using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// Multi-camera skeletal tracking provider with skeleton fusion support.
/// Uses calibration data to transform skeletons from secondary cameras to primary camera frame.
/// </summary>
public class FusedSkeletalTrackingProvider : BackgroundDataProvider
{
    private bool readFirstFrame = false;
    private TimeSpan initialTimestamp;

    // Configuration
    private string calibrationFilePath;
    private FusionMode fusionMode;
    private bool fusionEnabled;

    // Calibration data
    private CalibrationData calibrationData;

    // Multi-device state
    private int numDevices;
    private Device[] devices;
    private Tracker[] trackers;
    private Calibration[] deviceCalibrations;
    private string[] serialNumbers;

    // Per-device body data
    private Body[][] rawBodiesPerDevice;
    private int[] bodyCountsPerDevice;
    private object[] deviceLocks;

    // Thread management
    private Task[] captureTasks;
    private CancellationTokenSource captureTokenSource;

    public FusedSkeletalTrackingProvider(
        string calibrationFile,
        FusionMode mode = FusionMode.WeightedAverage,
        bool enableFusion = true) : base(0)
    {
        calibrationFilePath = calibrationFile;
        fusionMode = mode;
        fusionEnabled = enableFusion;
    }

    public void SetFusionEnabled(bool enabled)
    {
        fusionEnabled = enabled;
        Debug.Log($"Skeleton fusion {(enabled ? "enabled" : "disabled")}");
    }

    public void SetFusionMode(FusionMode mode)
    {
        fusionMode = mode;
        Debug.Log($"Fusion mode set to {mode}");
    }

    public void ToggleFusionMode()
    {
        fusionMode = fusionMode == FusionMode.WeightedAverage
            ? FusionMode.WinnerTakesAll
            : FusionMode.WeightedAverage;
        Debug.Log($"Fusion mode switched to {fusionMode}");
    }

    protected override void RunBackgroundThreadAsync(int id, CancellationToken token)
    {
        try
        {
            Debug.Log("Starting fused body tracker background thread.");

            // Load calibration
            calibrationData = CalibrationLoader.LoadFromJson(calibrationFilePath);
            if (calibrationData == null)
            {
                Debug.LogError("Failed to load calibration. Running single-camera mode.");
                RunSingleCameraMode(token);
                return;
            }

            numDevices = calibrationData.num_devices;
            if (numDevices < 1)
            {
                Debug.LogError("No devices in calibration. Running single-camera mode.");
                RunSingleCameraMode(token);
                return;
            }

            Debug.Log($"Calibration loaded for {numDevices} devices");

            // Detect connected devices
            int connectedDevices = Device.GetInstalledCount();
            if (connectedDevices < numDevices)
            {
                Debug.LogWarning($"Expected {numDevices} devices but only {connectedDevices} connected.");
                numDevices = connectedDevices;
            }

            if (numDevices == 0)
            {
                Debug.LogError("No devices connected!");
                return;
            }

            // Initialize arrays
            devices = new Device[numDevices];
            trackers = new Tracker[numDevices];
            deviceCalibrations = new Calibration[numDevices];
            serialNumbers = new string[numDevices];
            rawBodiesPerDevice = new Body[numDevices][];
            bodyCountsPerDevice = new int[numDevices];
            deviceLocks = new object[numDevices];

            for (int i = 0; i < numDevices; i++)
            {
                rawBodiesPerDevice[i] = new Body[20]; // Max 20 bodies per device
                for (int j = 0; j < 20; j++)
                {
                    rawBodiesPerDevice[i][j] = new Body(Skeleton.JointCount);
                }
                bodyCountsPerDevice[i] = 0;
                deviceLocks[i] = new object();
            }

            // Open devices
            if (!OpenDevices())
            {
                Debug.LogError("Failed to open devices.");
                return;
            }

            // Map serial numbers to calibration
            MapCalibrationToDevices();

            // Start capture threads for each device
            captureTokenSource = new CancellationTokenSource();
            captureTasks = new Task[numDevices];

            for (int i = 0; i < numDevices; i++)
            {
                int deviceIndex = i;
                captureTasks[i] = Task.Run(() => CaptureThread(deviceIndex, captureTokenSource.Token));
            }

            // Main fusion loop
            BackgroundData currentFrameData = new BackgroundData();

            while (!token.IsCancellationRequested)
            {
                Thread.Sleep(10); // ~100Hz fusion rate

                // Collect body data from all devices
                Body[][] collectedBodies = new Body[numDevices][];
                int[] collectedCounts = new int[numDevices];
                int primaryDepthDevice = 0;

                for (int d = 0; d < numDevices; d++)
                {
                    lock (deviceLocks[d])
                    {
                        collectedCounts[d] = bodyCountsPerDevice[d];
                        collectedBodies[d] = new Body[collectedCounts[d]];
                        for (int b = 0; b < collectedCounts[d]; b++)
                        {
                            collectedBodies[d][b] = Body.DeepCopy(rawBodiesPerDevice[d][b]);
                        }
                    }
                }

                // Check if any bodies detected
                int totalBodies = 0;
                for (int d = 0; d < numDevices; d++)
                    totalBodies += collectedCounts[d];

                if (totalBodies == 0)
                    continue;

                IsRunning = true;

                if (fusionEnabled && numDevices > 1)
                {
                    // Perform skeleton fusion
                    int fusedCount;
                    Body[] fusedBodies = SkeletonFusion.FuseSkeletons(
                        collectedBodies,
                        collectedCounts,
                        calibrationData,
                        fusionMode,
                        out fusedCount);

                    if (fusedBodies != null && fusedCount > 0)
                    {
                        currentFrameData.NumOfBodies = (ulong)fusedCount;
                        for (int i = 0; i < fusedCount && i < currentFrameData.Bodies.Length; i++)
                        {
                            CopyBodyData(fusedBodies[i], ref currentFrameData.Bodies[i]);
                        }
                    }
                }
                else
                {
                    // Single camera mode or fusion disabled - use primary camera only
                    currentFrameData.NumOfBodies = (ulong)collectedCounts[0];
                    for (int i = 0; i < collectedCounts[0] && i < currentFrameData.Bodies.Length; i++)
                    {
                        CopyBodyData(collectedBodies[0][i], ref currentFrameData.Bodies[i]);
                    }
                }

                // Set timestamp
                if (!readFirstFrame)
                {
                    readFirstFrame = true;
                    currentFrameData.TimestampInMs = 0;
                }
                else
                {
                    currentFrameData.TimestampInMs += 10; // Approximate
                }

                // Update frame data for main thread
                SetCurrentFrameData(ref currentFrameData);
            }

            // Cleanup
            Cleanup();
        }
        catch (Exception e)
        {
            Debug.LogError($"Fused body tracker exception: {e.Message}\n{e.StackTrace}");
            Cleanup();
            token.ThrowIfCancellationRequested();
        }
    }

    private void CaptureThread(int deviceIndex, CancellationToken token)
    {
        try
        {
            Debug.Log($"Device {deviceIndex} ({serialNumbers[deviceIndex]}) capture thread started.");

            while (!token.IsCancellationRequested)
            {
                try
                {
                    using (Capture sensorCapture = devices[deviceIndex].GetCapture(TimeSpan.FromMilliseconds(1000)))
                    {
                        trackers[deviceIndex].EnqueueCapture(sensorCapture);
                    }

                    using (Frame frame = trackers[deviceIndex].PopResult(TimeSpan.FromMilliseconds(100), throwOnTimeout: false))
                    {
                        if (frame != null)
                        {
                            lock (deviceLocks[deviceIndex])
                            {
                                bodyCountsPerDevice[deviceIndex] = (int)frame.NumberOfBodies;
                                for (uint i = 0; i < frame.NumberOfBodies && i < 20; i++)
                                {
                                    rawBodiesPerDevice[deviceIndex][i].CopyFromBodyTrackingSdk(
                                        frame.GetBody(i),
                                        deviceCalibrations[deviceIndex]);
                                }
                            }
                        }
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"Device {deviceIndex} capture error: {ex.Message}");
                    Thread.Sleep(100);
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Device {deviceIndex} capture thread exception: {e.Message}");
        }
    }

    private bool OpenDevices()
    {
        try
        {
            // Open all devices
            for (int i = 0; i < numDevices; i++)
            {
                devices[i] = Device.Open(i);
                serialNumbers[i] = devices[i].SerialNum;
                Debug.Log($"Opened device {i}: {serialNumbers[i]}");
            }

            // Configure devices - first device as primary (Standalone), others as Subordinate
            for (int i = 0; i < numDevices; i++)
            {
                WiredSyncMode syncMode = (i == 0)
                    ? WiredSyncMode.Standalone  // Or Master if using sync
                    : WiredSyncMode.Subordinate;

                // Note: For Orbbec Femto Bolt with sync hub, set appropriately
                // For now, using Standalone for simplicity in Unity

                devices[i].StartCameras(new DeviceConfiguration()
                {
                    CameraFPS = FPS.FPS30,
                    ColorResolution = ColorResolution.Off,
                    DepthMode = DepthMode.NFOV_Unbinned,
                    WiredSyncMode = WiredSyncMode.Standalone,
                    SubordinateDelayOffUsec = (i == 0) ? 0 : 160
                });

                deviceCalibrations[i] = devices[i].GetCalibration();

                trackers[i] = Tracker.Create(
                    deviceCalibrations[i],
                    new TrackerConfiguration()
                    {
                        ProcessingMode = TrackerProcessingMode.Gpu,
                        SensorOrientation = SensorOrientation.Default
                    });

                Debug.Log($"Device {i} ({serialNumbers[i]}) tracker created.");
            }

            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to open devices: {e.Message}");
            return false;
        }
    }

    private void MapCalibrationToDevices()
    {
        // Update calibration device indices based on serial number matching
        for (int i = 0; i < numDevices; i++)
        {
            bool found = false;
            foreach (var cal in calibrationData.calibrations)
            {
                if (cal.serial_number == serialNumbers[i])
                {
                    cal.device_index = i;
                    found = true;
                    Debug.Log($"Mapped calibration for device {i} (SN: {serialNumbers[i]})");
                    break;
                }
            }

            if (!found)
            {
                Debug.LogWarning($"No calibration found for device {i} (SN: {serialNumbers[i]})");
            }
        }
    }

    private void CopyBodyData(Body src, ref Body dst)
    {
        dst.Id = src.Id;
        dst.Length = src.Length;

        for (int j = 0; j < src.Length; j++)
        {
            dst.JointPositions3D[j] = src.JointPositions3D[j];
            dst.JointPositions2D[j] = src.JointPositions2D[j];
            dst.JointRotations[j] = src.JointRotations[j];
            dst.JointPrecisions[j] = src.JointPrecisions[j];
        }
    }

    private void RunSingleCameraMode(CancellationToken token)
    {
        // Fallback to single camera mode
        Debug.Log("Running in single-camera mode (no fusion).");

        BackgroundData currentFrameData = new BackgroundData();

        using (Device device = Device.Open(0))
        {
            device.StartCameras(new DeviceConfiguration()
            {
                CameraFPS = FPS.FPS30,
                ColorResolution = ColorResolution.Off,
                DepthMode = DepthMode.NFOV_Unbinned,
                WiredSyncMode = WiredSyncMode.Standalone,
            });

            var deviceCalibration = device.GetCalibration();

            using (Tracker tracker = Tracker.Create(deviceCalibration, new TrackerConfiguration()
            {
                ProcessingMode = TrackerProcessingMode.Gpu,
                SensorOrientation = SensorOrientation.Default
            }))
            {
                while (!token.IsCancellationRequested)
                {
                    using (Capture sensorCapture = device.GetCapture())
                    {
                        tracker.EnqueueCapture(sensorCapture);
                    }

                    using (Frame frame = tracker.PopResult(TimeSpan.Zero, throwOnTimeout: false))
                    {
                        if (frame != null)
                        {
                            IsRunning = true;
                            currentFrameData.NumOfBodies = frame.NumberOfBodies;

                            for (uint i = 0; i < currentFrameData.NumOfBodies; i++)
                            {
                                currentFrameData.Bodies[i].CopyFromBodyTrackingSdk(frame.GetBody(i), deviceCalibration);
                            }

                            SetCurrentFrameData(ref currentFrameData);
                        }
                    }
                }
            }
        }
    }

    private void Cleanup()
    {
        Debug.Log("Cleaning up fused body tracker...");

        captureTokenSource?.Cancel();

        if (captureTasks != null)
        {
            try
            {
                Task.WaitAll(captureTasks, 2000);
            }
            catch { }
        }

        if (trackers != null)
        {
            foreach (var tracker in trackers)
            {
                try { tracker?.Dispose(); } catch { }
            }
        }

        if (devices != null)
        {
            foreach (var device in devices)
            {
                try { device?.Dispose(); } catch { }
            }
        }

        captureTokenSource?.Dispose();
    }
}
