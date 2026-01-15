using UnityEngine;
using TMPro;

/// <summary>
/// Updates the recording status UI to show recording state and 6DOF data.
/// Keeps the UI fixed in front of the user's view in VR.
/// </summary>
public class RecordingStatusUI : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Reference to HMDDataRecorder. If null, will auto-find in scene.")]
    public HMDDataRecorder dataRecorder;

    [Tooltip("Reference to OVRCameraRig. If null, will auto-find in scene.")]
    public OVRCameraRig cameraRig;

    [Tooltip("The canvas to position in front of user")]
    public Canvas statusCanvas;

    [Header("UI Elements")]
    public TextMeshProUGUI recordingIndicator;
    public TextMeshProUGUI frameCountText;
    public TextMeshProUGUI hmdPositionText;
    public TextMeshProUGUI hmdRotationText;
    public TextMeshProUGUI leftControllerText;
    public TextMeshProUGUI rightControllerText;

    [Header("UI Settings")]
    [Tooltip("Distance from HMD to place the UI")]
    public float uiDistance = 0.5f;

    [Tooltip("Offset below eye level")]
    public float verticalOffset = -0.1f;

    [Tooltip("Scale of the canvas in world space")]
    public float canvasScale = 0.001f;

    [Tooltip("Smoothing for UI follow (0 = instant, 1 = very smooth)")]
    [Range(0f, 0.99f)]
    public float followSmoothing = 0.9f;

    // Cached transforms
    private Transform hmdTransform;
    private Transform leftControllerTransform;
    private Transform rightControllerTransform;
    private Transform canvasTransform;

    // Target position/rotation for smooth follow
    private Vector3 targetPosition;
    private Quaternion targetRotation;

    void Start()
    {
        // Auto-find HMDDataRecorder if not assigned
        if (dataRecorder == null)
        {
            dataRecorder = FindObjectOfType<HMDDataRecorder>();
            if (dataRecorder == null)
            {
                Debug.LogError("[RecordingStatusUI] HMDDataRecorder not found in scene!");
            }
        }

        // Auto-find OVRCameraRig if not assigned
        if (cameraRig == null)
        {
            cameraRig = FindObjectOfType<OVRCameraRig>();
            if (cameraRig == null)
            {
                Debug.LogError("[RecordingStatusUI] OVRCameraRig not found in scene!");
            }
        }

        // Cache transforms
        if (cameraRig != null)
        {
            hmdTransform = cameraRig.centerEyeAnchor;
            leftControllerTransform = cameraRig.leftHandAnchor;
            rightControllerTransform = cameraRig.rightHandAnchor;
        }

        // Auto-find Canvas if not assigned
        if (statusCanvas == null)
        {
            statusCanvas = GetComponentInChildren<Canvas>();
            if (statusCanvas == null)
            {
                // Try to find by name
                var canvasObj = GameObject.Find("RecordingStatusCanvas");
                if (canvasObj != null)
                    statusCanvas = canvasObj.GetComponent<Canvas>();
            }
        }

        if (statusCanvas != null)
        {
            canvasTransform = statusCanvas.transform;
            canvasTransform.localScale = Vector3.one * canvasScale;

            // Initialize position
            if (hmdTransform != null)
            {
                UpdateTargetTransform();
                canvasTransform.position = targetPosition;
                canvasTransform.rotation = targetRotation;
            }
        }

        // Auto-find UI elements if not assigned
        AutoFindUIElements();

        Debug.Log("[RecordingStatusUI] Initialized.");
    }

    void AutoFindUIElements()
    {
        if (recordingIndicator == null)
            recordingIndicator = FindTextByName("RecordingIndicator");
        if (frameCountText == null)
            frameCountText = FindTextByName("FrameCountText");
        if (hmdPositionText == null)
            hmdPositionText = FindTextByName("HMDPositionText");
        if (hmdRotationText == null)
            hmdRotationText = FindTextByName("HMDRotationText");
        if (leftControllerText == null)
            leftControllerText = FindTextByName("LeftControllerText");
        if (rightControllerText == null)
            rightControllerText = FindTextByName("RightControllerText");
    }

    TextMeshProUGUI FindTextByName(string name)
    {
        var go = GameObject.Find(name);
        if (go != null)
            return go.GetComponent<TextMeshProUGUI>();
        return null;
    }

    void LateUpdate()
    {
        UpdateCanvasPosition();
        UpdateUIContent();
    }

    void UpdateTargetTransform()
    {
        if (hmdTransform == null) return;

        // Calculate target position in front of HMD
        Vector3 forward = hmdTransform.forward;
        forward.y = 0; // Keep horizontal
        if (forward.sqrMagnitude < 0.001f)
            forward = Vector3.forward;
        forward.Normalize();

        targetPosition = hmdTransform.position + forward * uiDistance;
        targetPosition.y = hmdTransform.position.y + verticalOffset;

        // Face the user
        targetRotation = Quaternion.LookRotation(forward, Vector3.up);
    }

    void UpdateCanvasPosition()
    {
        if (canvasTransform == null || hmdTransform == null) return;

        UpdateTargetTransform();

        // Smooth follow
        canvasTransform.position = Vector3.Lerp(canvasTransform.position, targetPosition, 1f - followSmoothing);
        canvasTransform.rotation = Quaternion.Slerp(canvasTransform.rotation, targetRotation, 1f - followSmoothing);
    }

    void UpdateUIContent()
    {
        // Update recording indicator
        if (recordingIndicator != null && dataRecorder != null)
        {
            if (dataRecorder.IsRecording)
            {
                recordingIndicator.text = "<color=#FF0000>\u25CF</color> REC";
                recordingIndicator.color = Color.white;
            }
            else
            {
                recordingIndicator.text = "IDLE";
                recordingIndicator.color = new Color(0.67f, 0.67f, 0.67f);
            }
        }

        // Update frame count
        if (frameCountText != null && dataRecorder != null)
        {
            frameCountText.text = $"Frame: {dataRecorder.FrameCount}";
        }

        // Update HMD position
        if (hmdPositionText != null && hmdTransform != null)
        {
            Vector3 pos = hmdTransform.position;
            hmdPositionText.text = $"HMD Pos: ({pos.x:F2}, {pos.y:F2}, {pos.z:F2})";
        }

        // Update HMD rotation
        if (hmdRotationText != null && hmdTransform != null)
        {
            Vector3 rot = hmdTransform.eulerAngles;
            hmdRotationText.text = $"HMD Rot: ({rot.x:F1}, {rot.y:F1}, {rot.z:F1})";
        }

        // Update left controller (6DoF: position + rotation)
        if (leftControllerText != null && leftControllerTransform != null)
        {
            Vector3 pos = leftControllerTransform.position;
            Vector3 rot = leftControllerTransform.eulerAngles;
            leftControllerText.text =
                $"L Ctrl Pos: ({pos.x:F2}, {pos.y:F2}, {pos.z:F2})  Rot: ({rot.x:F1}, {rot.y:F1}, {rot.z:F1})";
        }

        // Update right controller (6DoF: position + rotation)
        if (rightControllerText != null && rightControllerTransform != null)
        {
            Vector3 pos = rightControllerTransform.position;
            Vector3 rot = rightControllerTransform.eulerAngles;
            rightControllerText.text =
                $"R Ctrl Pos: ({pos.x:F2}, {pos.y:F2}, {pos.z:F2})  Rot: ({rot.x:F1}, {rot.y:F1}, {rot.z:F1})";
        }
    }
}
