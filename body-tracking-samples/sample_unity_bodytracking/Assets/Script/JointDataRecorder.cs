using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class JointDataRecorder : MonoBehaviour
{
    public enum PostureType { Stand, Walk, Run, Jump, Sit, T_Pose }

    [Header("Connect to Kinect")]
    public Transform pointsParent;

    [Header("Recording Options")]
    public string ParticipantID = "01";
    public PostureType currentPosture;
    public float frameRate = 30f;

    private bool isRecording = false;
    private StreamWriter sw;
    private float interval;

    void Start()
    {
        interval = 1f / frameRate;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            if (!isRecording) StartRecording();
            else StopRecording();
        }
    }

    void StartRecording()
    {
        string folderPath = Path.Combine(Application.dataPath, "JointCSV");
        if (!Directory.Exists(folderPath)) Directory.CreateDirectory(folderPath);

        string fileName = $"Joint_{currentPosture}_P{ParticipantID}.csv";
        string path = Path.Combine(folderPath, fileName);

        sw = new StreamWriter(path);

        // 헤더 작성
        string header = "Frame,Time";
        for (int i = 0; i < 32; i++)
        {
            header += $",P{i}_posX,P{i}_posY,P{i}_posZ";
        }
        header += ",Timestamp"; // 맨 오른쪽에 추가
        sw.WriteLine(header);

        isRecording = true;
        StartCoroutine(RecordRoutine());
        Debug.Log($"<color=blue>녹화 시작:</color> {path}");
    }

    IEnumerator RecordRoutine()
    {
        int frameCount = 0;
        while (isRecording)
        {

            //bool isDetected = false;
            //Transform pointsParent = null;

            //if (targetStickman != null)
            //{
            //    pointsParent = targetStickman.transform.Find("Points");
            //    if (pointsParent != null && pointsParent.childCount == 32) isDetected = true;
            //}

            // 기본 데이터 (Frame, Unity Time, Detection)
            //string line = $"{frameCount},{Time.time},{(isDetected ? 1 : 0)}";
            string line = $"{frameCount},{Time.time}";

            // Joint 좌표 데이터
            //if (isDetected)
            //{
                foreach (Transform joint in pointsParent)
                {
                    line += $",{joint.position.x:F4},{joint.position.y:F4},{joint.position.z:F4}";
                }
            //}
            //else
            //{
                //for (int i = 0; i < 32 * 3; i++) line += ",0";
            //}

            // CSV 맨 오른쪽에 시간 
            string timestamp = System.DateTime.Now.ToString("MM-dd HH:mm:ss.fff");
            line += $",{timestamp}";

            sw.WriteLine(line);
            frameCount++;

            yield return new WaitForSecondsRealtime(interval);
        }
    }

    void StopRecording()
    {
        isRecording = false;
        if (sw != null) { sw.Close(); sw = null; }
        Debug.Log("<color=red>녹화 종료.</color>");
    }
}