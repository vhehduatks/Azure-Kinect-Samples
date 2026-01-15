using System;
using System.Collections;
using System.IO;
using UnityEngine;

/// <summary>
/// Records 6DOF (position + rotation) data from HMD and controllers.
/// Uses OVRCameraRig anchors for tracking data.
///
/// Keyboard Controls:
///   R - Start/Stop recording
///   ESC - Quit application
///
/// Output CSV format:
///   timestamp_ms, frame, unity_time,
///   hmd_pos_x, hmd_pos_y, hmd_pos_z, hmd_rot_x, hmd_rot_y, hmd_rot_z, hmd_rot_w,
///   left_pos_x, left_pos_y, left_pos_z, left_rot_x, left_rot_y, left_rot_z, left_rot_w,
///   right_pos_x, right_pos_y, right_pos_z, right_rot_x, right_rot_y, right_rot_z, right_rot_w
/// </summary>
public class HMDDataRecorder : MonoBehaviour
{
    [Header("OVR Camera Rig")]
    [Tooltip("Reference to OVRCameraRig. If null, will auto-find in scene.")]
    public OVRCameraRig cameraRig;

    [Header("Recording Settings")]
    [Tooltip("Participant ID for filename")]
    public string participantID = "P01";

    [Tooltip("Session name for filename")]
    public string sessionName = "Session01";

    [Tooltip("Recording frame rate (Hz)")]
    [Range(1f, 120f)]
    public float frameRate = 30f;

    [Header("Output Settings")]
    [Tooltip("Output folder relative to Application.dataPath")]
    public string outputFolder = "Recordings";

    [Tooltip("Use quaternion (XYZW) instead of Euler angles for rotation")]
    public bool useQuaternion = true;

    // Runtime state
    private bool isRecording = false;
    private StreamWriter csvWriter;
    private Coroutine recordCoroutine;
    private int frameCount = 0;
    private string currentFilePath;

    // Cached transforms
    private Transform hmdTransform;
    private Transform leftControllerTransform;
    private Transform rightControllerTransform;

    void Awake()
    {
        // Auto-find OVRCameraRig if not assigned
        if (cameraRig == null)
        {
            cameraRig = FindObjectOfType<OVRCameraRig>();
            if (cameraRig == null)
            {
                Debug.LogError("[HMDDataRecorder] OVRCameraRig not found in scene!");
            }
        }
    }

    void Start()
    {
        if (cameraRig != null)
        {
            // Cache transform references
            hmdTransform = cameraRig.centerEyeAnchor;
            leftControllerTransform = cameraRig.leftHandAnchor;
            rightControllerTransform = cameraRig.rightHandAnchor;

            Debug.Log("[HMDDataRecorder] Initialized. Press R to start/stop recording.");
        }
    }

    void Update()
    {
        // Toggle recording with R key
        if (Input.GetKeyDown(KeyCode.R))
        {
            if (!isRecording)
                StartRecording();
            else
                StopRecording();
        }

        // Quit with ESC
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            StopRecording();
            Application.Quit();
        }
    }

    void OnDisable()
    {
        StopRecording();
    }

    void OnApplicationQuit()
    {
        StopRecording();
    }

    /// <summary>
    /// Starts recording HMD and controller data to CSV.
    /// </summary>
    public void StartRecording()
    {
        if (isRecording) return;
        if (cameraRig == null)
        {
            Debug.LogError("[HMDDataRecorder] Cannot start recording: OVRCameraRig not found!");
            return;
        }

        // Create output directory
        string folderPath = Path.Combine(Application.dataPath, outputFolder);
        if (!Directory.Exists(folderPath))
        {
            Directory.CreateDirectory(folderPath);
        }

        // Generate filename with timestamp
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string fileName = $"HMD_{participantID}_{sessionName}_{timestamp}.csv";
        currentFilePath = Path.Combine(folderPath, fileName);

        // Open CSV file
        csvWriter = new StreamWriter(currentFilePath);

        // Write header
        WriteHeader();

        // Reset frame counter
        frameCount = 0;
        isRecording = true;

        // Start recording coroutine
        recordCoroutine = StartCoroutine(RecordLoop());

        Debug.Log($"[HMDDataRecorder] Recording started: {currentFilePath}");
    }

    /// <summary>
    /// Stops recording and closes the CSV file.
    /// </summary>
    public void StopRecording()
    {
        if (!isRecording) return;

        isRecording = false;

        // Stop coroutine
        if (recordCoroutine != null)
        {
            StopCoroutine(recordCoroutine);
            recordCoroutine = null;
        }

        // Close file
        if (csvWriter != null)
        {
            csvWriter.Flush();
            csvWriter.Close();
            csvWriter = null;
        }

        Debug.Log($"[HMDDataRecorder] Recording stopped. {frameCount} frames saved to: {currentFilePath}");
    }

    /// <summary>
    /// Returns true if currently recording.
    /// </summary>
    public bool IsRecording => isRecording;

    /// <summary>
    /// Returns the current frame count.
    /// </summary>
    public int FrameCount => frameCount;

    private void WriteHeader()
    {
        string header = "timestamp_ms,frame,unity_time";

        if (useQuaternion)
        {
            // Position (XYZ) + Quaternion (XYZW) for each device
            header += ",hmd_pos_x,hmd_pos_y,hmd_pos_z,hmd_rot_x,hmd_rot_y,hmd_rot_z,hmd_rot_w";
            header += ",left_pos_x,left_pos_y,left_pos_z,left_rot_x,left_rot_y,left_rot_z,left_rot_w";
            header += ",right_pos_x,right_pos_y,right_pos_z,right_rot_x,right_rot_y,right_rot_z,right_rot_w";
        }
        else
        {
            // Position (XYZ) + Euler angles (XYZ) for each device
            header += ",hmd_pos_x,hmd_pos_y,hmd_pos_z,hmd_rot_x,hmd_rot_y,hmd_rot_z";
            header += ",left_pos_x,left_pos_y,left_pos_z,left_rot_x,left_rot_y,left_rot_z";
            header += ",right_pos_x,right_pos_y,right_pos_z,right_rot_x,right_rot_y,right_rot_z";
        }

        csvWriter.WriteLine(header);
    }

    private IEnumerator RecordLoop()
    {
        float interval = 1f / frameRate;
        var wait = new WaitForSecondsRealtime(interval);

        while (isRecording)
        {
            RecordFrame();
            frameCount++;
            yield return wait;
        }
    }

    private void RecordFrame()
    {
        // Get timestamp
        long timestampMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        float unityTime = Time.time;

        // Get poses
        Vector3 hmdPos = hmdTransform != null ? hmdTransform.position : Vector3.zero;
        Quaternion hmdRot = hmdTransform != null ? hmdTransform.rotation : Quaternion.identity;

        Vector3 leftPos = leftControllerTransform != null ? leftControllerTransform.position : Vector3.zero;
        Quaternion leftRot = leftControllerTransform != null ? leftControllerTransform.rotation : Quaternion.identity;

        Vector3 rightPos = rightControllerTransform != null ? rightControllerTransform.position : Vector3.zero;
        Quaternion rightRot = rightControllerTransform != null ? rightControllerTransform.rotation : Quaternion.identity;

        // Build CSV line
        string line = $"{timestampMs},{frameCount},{unityTime:F4}";

        if (useQuaternion)
        {
            line += $",{FormatPose6DOF_Quat(hmdPos, hmdRot)}";
            line += $",{FormatPose6DOF_Quat(leftPos, leftRot)}";
            line += $",{FormatPose6DOF_Quat(rightPos, rightRot)}";
        }
        else
        {
            line += $",{FormatPose6DOF_Euler(hmdPos, hmdRot)}";
            line += $",{FormatPose6DOF_Euler(leftPos, leftRot)}";
            line += $",{FormatPose6DOF_Euler(rightPos, rightRot)}";
        }

        csvWriter.WriteLine(line);
    }

    private string FormatPose6DOF_Quat(Vector3 pos, Quaternion rot)
    {
        return $"{pos.x:F4},{pos.y:F4},{pos.z:F4},{rot.x:F4},{rot.y:F4},{rot.z:F4},{rot.w:F4}";
    }

    private string FormatPose6DOF_Euler(Vector3 pos, Quaternion rot)
    {
        Vector3 euler = rot.eulerAngles;
        return $"{pos.x:F4},{pos.y:F4},{pos.z:F4},{euler.x:F3},{euler.y:F3},{euler.z:F3}";
    }
}
