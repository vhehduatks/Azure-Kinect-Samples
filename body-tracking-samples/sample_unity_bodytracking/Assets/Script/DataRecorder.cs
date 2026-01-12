using System;
using System.Collections;
using System.IO;
using UnityEngine;
using TMPro;

public class DataRecorder : MonoBehaviour
{
    public enum PostureType { Stand, Walk, Run, Jump, Sit, T_Pose }

    [Header("Orbbec")]
    public Transform pointsParent;

    [Header("Hmd, Controller")]
    public OVRCameraRig rig;
    public GameObject Cube_L, Cube_R;
    public TextMeshProUGUI info_txt;

    [Header("Recording Options")]
    public string ParticipantID = "01";
    public PostureType currentPosture;
    [Tooltip("Record/UI update rate (Hz)")]
    public float frameRate = 30f;

    // ---- runtime ----
    private bool isRecording = false;
    private StreamWriter joint_sw, HMD_sw;
    private float interval;

    // Current (Update���� �� ������ ���ŵǴ� �ֽŰ�)
    private Vector3 curHmdPos, curLPos, curRPos;
    private Quaternion curHmdRot, curLRot, curRRot;

    // Last Recorded (RecordRoutine���� ������ CSV�� �� ��)
    private int recFrame = -1;
    private float recUnityTime = 0f;
    private string recTimestamp = "";
    private Vector3 recHmdPos, recLPos, recRPos;
    private Quaternion recHmdRot, recLRot, recRRot;

    private Coroutine recordRoutine;
    private Coroutine uiRoutine;

    // HMD, Controller Position and Rotation (Euler)
    string Pose6(Vector3 pos, Quaternion rot)
    {
        Vector3 e = rot.eulerAngles;
        return $"{pos.x:F4},{pos.y:F4},{pos.z:F4},{e.x:F3},{e.y:F3},{e.z:F3}";
    }

    // Get system timestamp in milliseconds (epoch time)
    long GetTimestampMs()
    {
        return DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }

    void Awake()
    {
        interval = (frameRate > 0f) ? (1f / frameRate) : (1f / 30f);
    }

    void Start()
    {
        // UI�� ��ȭ ���ο� ������� ��� �����ֱ�
        uiRoutine = StartCoroutine(UIRoutine());
    }

    void OnDisable()
    {
        // ���� ����
        if (uiRoutine != null) StopCoroutine(uiRoutine);
        StopRecording();
    }

    void Update()
    {
        // ��ȭ ���
        if (Input.GetKeyDown(KeyCode.R))
        {
            if (!isRecording) StartRecording();
            else StopRecording();
        }

        // ---- pose ���� (�� ������ �ֽŰ�) ----
        UpdateCurrentPoses();

        // (����) �׽�Ʈ ť�� ����ȭ
        // Cube_L / Cube_R �� �׻� ������� �Ϸ��� �Ʒ� �ּ� ����
        /*
        if (Cube_L != null)
        {
            Cube_L.transform.position = curLPos;
            Cube_L.transform.rotation = curLRot;
        }
        if (Cube_R != null)
        {
            Cube_R.transform.position = curRPos;
            Cube_R.transform.rotation = curRRot;
        }
        */
    }

    private void UpdateCurrentPoses()
    {
        Transform camTf = Camera.main != null ? Camera.main.transform : null;

        // HMD: centerEyeAnchor(����). ������ Camera.main ���
        Transform hmdTf = (rig != null && rig.centerEyeAnchor != null) ? rig.centerEyeAnchor : camTf;

        curHmdPos = hmdTf != null ? hmdTf.position : Vector3.zero;
        curHmdRot = hmdTf != null ? hmdTf.rotation : Quaternion.identity;

        // Controllers: hand anchors(����)
        Transform lTf = (rig != null) ? rig.leftHandAnchor : null;
        Transform rTf = (rig != null) ? rig.rightHandAnchor : null;

        curLPos = lTf != null ? lTf.position : Vector3.zero;
        curLRot = lTf != null ? lTf.rotation : Quaternion.identity;

        curRPos = rTf != null ? rTf.position : Vector3.zero;
        curRRot = rTf != null ? rTf.rotation : Quaternion.identity;
    }

    void StartRecording()
    {
        interval = (frameRate > 0f) ? (1f / frameRate) : (1f / 30f);

        // 1) Joint CSV
        string joint_folderPath = Path.Combine(Application.dataPath, "JointCSV");
        if (!Directory.Exists(joint_folderPath)) Directory.CreateDirectory(joint_folderPath);

        string joint_fileName = $"Joint_{currentPosture}_P{ParticipantID}.csv";
        string joint_path = Path.Combine(joint_folderPath, joint_fileName);

        joint_sw = new StreamWriter(joint_path);

        // 1-1) Joint header
        string header = "timestamp_ms,frame,unity_time";

        string jointsPart = "";
        for (int i = 0; i < 32; i++)
            jointsPart += $",joint{i}_x,joint{i}_y,joint{i}_z";

        joint_sw.WriteLine(header + jointsPart);

        // 2) HMD, Controller CSV
        string hmd_folderPath = Path.Combine(Application.dataPath, "HMD_CSV");
        if (!Directory.Exists(hmd_folderPath)) Directory.CreateDirectory(hmd_folderPath);

        string hmd_fileName = $"HMD_{currentPosture}_P{ParticipantID}.csv";
        string hmd_path = Path.Combine(hmd_folderPath, hmd_fileName);

        HMD_sw = new StreamWriter(hmd_path);

        string hmdsPart = "";
        hmdsPart += ",hmd_pos_x,hmd_pos_y,hmd_pos_z,hmd_rot_x,hmd_rot_y,hmd_rot_z";
        hmdsPart += ",left_pos_x,left_pos_y,left_pos_z,left_rot_x,left_rot_y,left_rot_z";
        hmdsPart += ",right_pos_x,right_pos_y,right_pos_z,right_rot_x,right_rot_y,right_rot_z";

        HMD_sw.WriteLine(header + hmdsPart);

        // ��� ����
        isRecording = true;

        // ������ ī��Ʈ �ʱ�ȭ
        recFrame = -1;

        // �̹� ���� ������ ����
        if (recordRoutine != null) StopCoroutine(recordRoutine);
        recordRoutine = StartCoroutine(RecordRoutine());

        Debug.Log($"<color=blue>��� ����:</color> {joint_path}");
    }

    IEnumerator RecordRoutine()
    {
        int frameCount = 0;
        var wait = new WaitForSecondsRealtime(interval);

        while (isRecording)
        {
            // Get timestamp in milliseconds (epoch time for synchronization)
            long timestampMs = GetTimestampMs();
            float unityTime = Time.time;
            string prefix = $"{timestampMs},{frameCount},{unityTime:F4}";

            // ---- Joint recording ----
            string joints = "";
            if (pointsParent != null)
            {
                foreach (Transform joint in pointsParent)
                {
                    joints += $",{joint.position.x:F4},{joint.position.y:F4},{joint.position.z:F4}";
                }
            }
            else
            {
                // If pointsParent is null, fill with zeros
                for (int i = 0; i < 32; i++) joints += ",0,0,0";
            }

            joint_sw.WriteLine(prefix + joints);
            joint_sw.Flush();  // Ensure data is written immediately

            // ---- HMD/Controller recording ----
            recFrame = frameCount;
            recUnityTime = unityTime;
            recTimestamp = timestampMs.ToString();

            recHmdPos = curHmdPos; recHmdRot = curHmdRot;
            recLPos = curLPos; recLRot = curLRot;
            recRPos = curRPos; recRRot = curRRot;

            string payload =
                "," + Pose6(recHmdPos, recHmdRot) +
                "," + Pose6(recLPos, recLRot) +
                "," + Pose6(recRPos, recRRot);

            HMD_sw.WriteLine(prefix + payload);
            HMD_sw.Flush();  // Ensure data is written immediately

            frameCount++;
            yield return wait;
        }
    }

    IEnumerator UIRoutine()
    {
        while (true)
        {
            if (info_txt != null)
            {
                Vector3 che = curHmdRot.eulerAngles;
                Vector3 cle = curLRot.eulerAngles;
                Vector3 cre = curRRot.eulerAngles;

                Vector3 rhe = recHmdRot.eulerAngles;
                Vector3 rle = recLRot.eulerAngles;
                Vector3 rre = recRRot.eulerAngles;

                // Current: �� ������ ���ŵǴ� ��
                // Recorded: ���� CSV�� ���������� ��ϵ� ��(�����ӷ���Ʈ ���)
                info_txt.text =
                    $"                Position                     Rotation \n" +
                    $"          X        Y         Z         X         Y         Z\n" +
                    $"HMD : {curHmdPos.x:F2}, {curHmdPos.y:F}, {curHmdPos.z:F2}  | {che.x:F1}, {che.y:F1}, {che.z:F1}\n" +
                    $"L       : {curLPos.x:F2}, {curLPos.y:F2}, {curLPos.z:F2}  | {cle.x:F1}, {cle.y:F1}, {cle.z:F1}\n" +
                    $"R      : {curRPos.x:F2}, {curRPos.y:F2}, {curRPos.z:F2}  | {cre.x:F1}, {cre.y:F1}, {cre.z:F1}\n";
            }

            // UI ���ŵ� frameRate(Hz)�� ���߱�
            float uiInterval = (frameRate > 0f) ? (1f / frameRate) : (1f / 30f);
            yield return new WaitForSecondsRealtime(uiInterval);
        }
    }

    void StopRecording()
    {
        if (!isRecording) return;

        isRecording = false;

        if (recordRoutine != null)
        {
            StopCoroutine(recordRoutine);
            recordRoutine = null;
        }

        if (joint_sw != null) { joint_sw.Close(); joint_sw = null; }
        if (HMD_sw != null) { HMD_sw.Close(); HMD_sw = null; }

        Debug.Log("<color=red>��ȭ ����.</color>");
    }
}