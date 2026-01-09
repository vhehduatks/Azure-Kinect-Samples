using UnityEngine;

public class main : MonoBehaviour
{
    // Handler for SkeletalTracking thread.
    public GameObject m_tracker;

    [Header("Multi-Camera Fusion Settings")]
    [Tooltip("Enable multi-camera skeleton fusion")]
    public bool enableFusion = false;

    [Tooltip("Path to calibration.json file (relative to StreamingAssets or absolute)")]
    public string calibrationFile = "calibration.json";

    [Tooltip("Fusion mode: WeightedAverage or WinnerTakesAll")]
    public FusionMode fusionMode = FusionMode.WeightedAverage;

    private BackgroundDataProvider m_trackingProvider;
    private FusedSkeletalTrackingProvider m_fusedProvider;
    public BackgroundData m_lastFrameData = new BackgroundData();

    void Start()
    {
        if (enableFusion)
        {
            // Resolve calibration file path
            string calibPath = ResolveCalibrationPath(calibrationFile);
            Debug.Log($"Starting multi-camera fusion with calibration: {calibPath}");

            m_fusedProvider = new FusedSkeletalTrackingProvider(calibPath, fusionMode, true);
            m_trackingProvider = m_fusedProvider;
        }
        else
        {
            // Single camera mode
            const int TRACKER_ID = 0;
            Debug.Log("Starting single-camera body tracking.");
            m_trackingProvider = new SkeletalTrackingProvider(TRACKER_ID);
        }
    }

    void Update()
    {
        if (m_trackingProvider != null && m_trackingProvider.IsRunning)
        {
            if (m_trackingProvider.GetCurrentFrameData(ref m_lastFrameData))
            {
                if (m_lastFrameData.NumOfBodies != 0)
                {
                    m_tracker.GetComponent<TrackerHandler>().updateTracker(m_lastFrameData);
                }
            }
        }

        // Runtime controls
        HandleInput();
    }

    void HandleInput()
    {
        // F key: Toggle fusion on/off
        if (Input.GetKeyDown(KeyCode.F) && m_fusedProvider != null)
        {
            enableFusion = !enableFusion;
            m_fusedProvider.SetFusionEnabled(enableFusion);
        }

        // M key: Switch fusion mode
        if (Input.GetKeyDown(KeyCode.M) && m_fusedProvider != null)
        {
            m_fusedProvider.ToggleFusionMode();
        }
    }

    private string ResolveCalibrationPath(string path)
    {
        // Check if absolute path
        if (System.IO.Path.IsPathRooted(path) && System.IO.File.Exists(path))
        {
            return path;
        }

        // Try StreamingAssets folder
        string streamingPath = System.IO.Path.Combine(Application.streamingAssetsPath, path);
        if (System.IO.File.Exists(streamingPath))
        {
            return streamingPath;
        }

        // Try application data path
        string dataPath = System.IO.Path.Combine(Application.dataPath, path);
        if (System.IO.File.Exists(dataPath))
        {
            return dataPath;
        }

        // Try persistent data path
        string persistentPath = System.IO.Path.Combine(Application.persistentDataPath, path);
        if (System.IO.File.Exists(persistentPath))
        {
            return persistentPath;
        }

        // Return original path, let the loader handle the error
        Debug.LogWarning($"Calibration file not found in common locations. Using: {path}");
        return path;
    }

    void OnApplicationQuit()
    {
        if (m_trackingProvider != null)
        {
            m_trackingProvider.Dispose();
        }
    }
}
