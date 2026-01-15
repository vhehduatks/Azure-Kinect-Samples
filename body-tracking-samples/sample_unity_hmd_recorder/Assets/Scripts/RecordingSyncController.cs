using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Synchronizes recording between Unity and external applications (e.g., multi_device_body_viewer)
/// via UDP commands. When R is pressed in Unity, it sends a command to toggle recording
/// in the external application as well.
///
/// Protocol:
///   - TOGGLE_RECORD: Toggle recording on/off
///   - START_RECORD: Start recording
///   - STOP_RECORD: Stop recording
///   - CYCLE_CAMERA: Cycle camera view
/// </summary>
public class RecordingSyncController : MonoBehaviour
{
    [Header("UDP Settings")]
    [Tooltip("Target IP address (127.0.0.1 for local)")]
    public string targetIP = "127.0.0.1";

    [Tooltip("Port to send commands to (must match multi_device_body_viewer)")]
    public int sendPort = 9000;

    [Tooltip("Port to receive commands from (for bidirectional sync)")]
    public int receivePort = 9001;

    [Tooltip("Enable receiving commands from external app")]
    public bool enableReceive = true;

    [Header("Sync Target")]
    [Tooltip("HMDDataRecorder to control (auto-find if null)")]
    public HMDDataRecorder hmdRecorder;

    [Header("Status")]
    [SerializeField] private bool isConnected = false;
    [SerializeField] private string lastSentCommand = "";
    [SerializeField] private string lastReceivedCommand = "";

    // UDP clients
    private UdpClient sendClient;
    private UdpClient receiveClient;
    private Thread receiveThread;
    private bool isRunning = false;

    // Thread-safe command queue
    private string pendingCommand = null;
    private readonly object commandLock = new object();

    void Start()
    {
        // Auto-find HMDDataRecorder if not assigned
        if (hmdRecorder == null)
        {
            hmdRecorder = FindObjectOfType<HMDDataRecorder>();
            if (hmdRecorder == null)
            {
                Debug.LogWarning("[RecordingSync] HMDDataRecorder not found in scene");
            }
        }

        // Initialize UDP sender
        try
        {
            sendClient = new UdpClient();
            isConnected = true;
            Debug.Log($"[RecordingSync] UDP sender ready -> {targetIP}:{sendPort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[RecordingSync] Failed to create UDP sender: {e.Message}");
            isConnected = false;
        }

        // Initialize UDP receiver
        if (enableReceive)
        {
            StartReceiver();
        }
    }

    void StartReceiver()
    {
        try
        {
            receiveClient = new UdpClient(receivePort);
            isRunning = true;
            receiveThread = new Thread(ReceiveLoop);
            receiveThread.IsBackground = true;
            receiveThread.Start();
            Debug.Log($"[RecordingSync] UDP receiver listening on port {receivePort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[RecordingSync] Failed to start UDP receiver: {e.Message}");
        }
    }

    void ReceiveLoop()
    {
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);

        while (isRunning)
        {
            try
            {
                byte[] data = receiveClient.Receive(ref remoteEP);
                string command = Encoding.UTF8.GetString(data).Trim();

                lock (commandLock)
                {
                    pendingCommand = command;
                }
            }
            catch (SocketException)
            {
                // Socket closed, exit loop
                break;
            }
            catch (Exception e)
            {
                if (isRunning)
                {
                    Debug.LogError($"[RecordingSync] Receive error: {e.Message}");
                }
            }
        }
    }

    void Update()
    {
        // Process received commands on main thread
        ProcessPendingCommand();

        // Send commands on key press
        if (Input.GetKeyDown(KeyCode.R))
        {
            SendCommand("TOGGLE_RECORD");
        }

        if (Input.GetKeyDown(KeyCode.K))
        {
            SendCommand("CYCLE_CAMERA");
        }
    }

    void ProcessPendingCommand()
    {
        string command = null;
        lock (commandLock)
        {
            command = pendingCommand;
            pendingCommand = null;
        }

        if (string.IsNullOrEmpty(command)) return;

        lastReceivedCommand = command;
        Debug.Log($"[RecordingSync] Received: {command}");

        // Handle command
        switch (command)
        {
            case "TOGGLE_RECORD":
                ToggleLocalRecording();
                break;
            case "START_RECORD":
                StartLocalRecording();
                break;
            case "STOP_RECORD":
                StopLocalRecording();
                break;
            default:
                Debug.Log($"[RecordingSync] Unknown command: {command}");
                break;
        }
    }

    /// <summary>
    /// Send a command to the external application.
    /// </summary>
    public void SendCommand(string command)
    {
        if (sendClient == null || !isConnected)
        {
            Debug.LogWarning("[RecordingSync] UDP sender not ready");
            return;
        }

        try
        {
            byte[] data = Encoding.UTF8.GetBytes(command);
            sendClient.Send(data, data.Length, targetIP, sendPort);
            lastSentCommand = command;
            Debug.Log($"[RecordingSync] Sent: {command} -> {targetIP}:{sendPort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[RecordingSync] Failed to send command: {e.Message}");
        }
    }

    void ToggleLocalRecording()
    {
        if (hmdRecorder == null) return;

        if (hmdRecorder.IsRecording)
        {
            hmdRecorder.StopRecording();
            Debug.Log("[RecordingSync] Stopped local recording (external trigger)");
        }
        else
        {
            hmdRecorder.StartRecording();
            Debug.Log("[RecordingSync] Started local recording (external trigger)");
        }
    }

    void StartLocalRecording()
    {
        if (hmdRecorder == null) return;

        if (!hmdRecorder.IsRecording)
        {
            hmdRecorder.StartRecording();
            Debug.Log("[RecordingSync] Started local recording (external trigger)");
        }
    }

    void StopLocalRecording()
    {
        if (hmdRecorder == null) return;

        if (hmdRecorder.IsRecording)
        {
            hmdRecorder.StopRecording();
            Debug.Log("[RecordingSync] Stopped local recording (external trigger)");
        }
    }

    void OnDestroy()
    {
        isRunning = false;

        // Close sender
        if (sendClient != null)
        {
            sendClient.Close();
            sendClient = null;
        }

        // Close receiver
        if (receiveClient != null)
        {
            receiveClient.Close();
            receiveClient = null;
        }

        // Wait for receiver thread
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join(1000);
        }

        Debug.Log("[RecordingSync] Cleaned up");
    }

    void OnApplicationQuit()
    {
        OnDestroy();
    }
}
