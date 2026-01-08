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

    // Current (Update에서 매 프레임 갱신되는 최신값)
    private Vector3 curHmdPos, curLPos, curRPos;
    private Quaternion curHmdRot, curLRot, curRRot;

    // Last Recorded (RecordRoutine에서 실제로 CSV에 쓴 값)
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

    void Awake()
    {
        interval = (frameRate > 0f) ? (1f / frameRate) : (1f / 30f);
    }

    void Start()
    {
        // UI는 녹화 여부와 상관없이 계속 보여주기
        uiRoutine = StartCoroutine(UIRoutine());
    }

    void OnDisable()
    {
        // 안전 종료
        if (uiRoutine != null) StopCoroutine(uiRoutine);
        StopRecording();
    }

    void Update()
    {
        // 녹화 토글
        if (Input.GetKeyDown(KeyCode.R))
        {
            if (!isRecording) StartRecording();
            else StopRecording();
        }

        // ---- pose 갱신 (매 프레임 최신값) ----
        UpdateCurrentPoses();

        // (선택) 테스트 큐브 동기화
        // Cube_L / Cube_R 를 항상 따라오게 하려면 아래 주석 해제
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

        // HMD: centerEyeAnchor(월드). 없으면 Camera.main 사용
        Transform hmdTf = (rig != null && rig.centerEyeAnchor != null) ? rig.centerEyeAnchor : camTf;

        curHmdPos = hmdTf != null ? hmdTf.position : Vector3.zero;
        curHmdRot = hmdTf != null ? hmdTf.rotation : Quaternion.identity;

        // Controllers: hand anchors(월드)
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

        // 1-1) Joint 헤더
        string header = "Frame,Time";
        string last_header = ",Timestamp";

        string jointsPart = "";
        for (int i = 0; i < 32; i++)
            jointsPart += $",P{i}_posX,P{i}_posY,P{i}_posZ";

        joint_sw.WriteLine(header + jointsPart + last_header);

        // 2) HMD, Controller CSV
        string hmd_folderPath = Path.Combine(Application.dataPath, "HMD_CSV");
        if (!Directory.Exists(hmd_folderPath)) Directory.CreateDirectory(hmd_folderPath);

        string hmd_fileName = $"HMD_{currentPosture}_P{ParticipantID}.csv";
        string hmd_path = Path.Combine(hmd_folderPath, hmd_fileName);

        HMD_sw = new StreamWriter(hmd_path);

        string hmdsPart = "";
        hmdsPart += ",HMD_PosX,HMD_PosY,HMD_PosZ,HMD_RotX,HMD_RotY,HMD_RotZ";
        hmdsPart += ",L_ctrl_PosX,L_ctrl_PosY,L_ctrl_PosZ,L_ctrl_RotX,L_ctrl_RotY,L_ctrl_RotZ";
        hmdsPart += ",R_ctrl_PosX,R_ctrl_PosY,R_ctrl_PosZ,R_ctrl_RotX,R_ctrl_RotY,R_ctrl_RotZ";

        HMD_sw.WriteLine(header + hmdsPart + last_header);

        // 기록 시작
        isRecording = true;

        // 프레임 카운트 초기화
        recFrame = -1;

        // 이미 돌고 있으면 정리
        if (recordRoutine != null) StopCoroutine(recordRoutine);
        recordRoutine = StartCoroutine(RecordRoutine());

        Debug.Log($"<color=blue>기록 시작:</color> {joint_path}");
    }

    IEnumerator RecordRoutine()
    {
        int frameCount = 0;
        var wait = new WaitForSecondsRealtime(interval);

        while (isRecording)
        {
            // CSV prefix/suffix
            float unityTime = Time.time;
            string prefix = $"{frameCount},{unityTime}";
            string timestamp = System.DateTime.Now.ToString("MM-dd HH:mm:ss.fff");
            string suffix = $",{timestamp}";

            // ---- Joint 기록 ----
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
                // pointsParent 없으면 32개 0으로 채우기
                for (int i = 0; i < 32; i++) joints += ",0,0,0";
            }

            joint_sw.WriteLine(prefix + joints + suffix);

            // ---- HMD/Controller 기록 (Update의 최신값을 사용) ----
            // "이번 프레임에 CSV에 쓴 값"을 snapshot으로 저장해 UI에 표시
            recFrame = frameCount;
            recUnityTime = unityTime;
            recTimestamp = timestamp;

            recHmdPos = curHmdPos; recHmdRot = curHmdRot;
            recLPos = curLPos; recLRot = curLRot;
            recRPos = curRPos; recRRot = curRRot;

            string payload =
                "," + Pose6(recHmdPos, recHmdRot) +
                "," + Pose6(recLPos, recLRot) +
                "," + Pose6(recRPos, recRRot);

            HMD_sw.WriteLine(prefix + payload + suffix);

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

                // Current: 매 프레임 갱신되는 값
                // Recorded: 실제 CSV에 마지막으로 기록된 값(프레임레이트 기반)
                info_txt.text =
                    $"                Position                     Rotation \n" +
                    $"          X        Y         Z         X         Y         Z\n" +
                    $"HMD : {curHmdPos.x:F2}, {curHmdPos.y:F}, {curHmdPos.z:F2}  | {che.x:F1}, {che.y:F1}, {che.z:F1}\n" +
                    $"L       : {curLPos.x:F2}, {curLPos.y:F2}, {curLPos.z:F2}  | {cle.x:F1}, {cle.y:F1}, {cle.z:F1}\n" +
                    $"R      : {curRPos.x:F2}, {curRPos.y:F2}, {curRPos.z:F2}  | {cre.x:F1}, {cre.y:F1}, {cre.z:F1}\n";
            }

            // UI 갱신도 frameRate(Hz)로 맞추기
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

        Debug.Log("<color=red>녹화 종료.</color>");
    }
}