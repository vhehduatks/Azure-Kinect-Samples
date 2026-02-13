using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;
using TMPro;

public class ActionBtn : MonoBehaviour
{
    [System.Serializable]
    public class Item
    {
        public string text = "";        // 버튼 이름 + sessionName
        public VideoClip video;         // 재생할 비디오
    }

    [Header("ActionVideo")]
    public List<Item> items = new List<Item>();

    [Header("CountDown")]
    public int countdownSeconds = 3;


    [Header("Stop by Recorder FrameCount")]
    public int stopAtRecordFrame = 0; // 0이면 기존처럼 '비디오 끝' 기준


    [Header("GUI Header Text")]
    public Vector2 startPos = new Vector2(20f, 20f);
    public bool showHeaderText = true;

    private GUIStyle headerStyle;
    private string participantIdCached = "";
    private string currentActionLabel = "None";

    [Header("IMGUI Layout (Top-Left Row)")]
    public int columns = 5;
    public Vector2 buttonSize = new Vector2(160f, 40f);
    public float spacing = 12f;

    [Header("Reset Button")]
    public float spacingwithHeader = 500f;
    public bool showResetButton = true;
    public string resetLabel = "Reset";
    public Vector2 resetButtonSize = new Vector2(160f, 40f);

    [Header("References")]
    public HMDDataRecorder recorder;
    public RecordingSyncController UDPcontrol;
    public VideoPlayer videoPlayer;

    [Header("UI (TextMeshPro)")]
    public TMP_Text ActionName_Panel;   // 선택된 버튼 이름 표시
    public TMP_Text countdownText;     // 3 2 1 / 종료 표시

    private GUIStyle buttonStyle;
    private GUIStyle resetStyle;

    private bool busy = false;               // 진행 중이면 추가 클릭 막기
    private Coroutine flowCoroutine;

    private void Start()
    {
        if (recorder != null)
            participantIdCached = recorder.participantID;
        else
            participantIdCached = "Unknown";
    }


    private void OnGUI()
    {
        if (items == null || items.Count == 0) return;

        if (buttonStyle == null)
        {
            buttonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 30,
                alignment = TextAnchor.MiddleCenter
            };
        }

        if (resetStyle == null)
        {
            resetStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 30,
                alignment = TextAnchor.MiddleCenter
            };
        }

        float x = startPos.x;
        float y = startPos.y;

        // --- Header text (above action buttons) ---
        if (showHeaderText)
        {
            if (headerStyle == null)
            {
                headerStyle = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 50,
                    alignment = TextAnchor.UpperLeft
                };
            }

            float headerX = startPos.x;
            float headerY = startPos.y; // 버튼 위로 올림

            string header = $"{participantIdCached}   -   {currentActionLabel}";
            GUI.Label(new Rect(headerX, headerY, 500f, 100f), header, headerStyle);
        }

        float buttonY = y + 100f;

        // --- Action 버튼들 (6개씩 한줄) ---
        for (int i = 0; i < items.Count; i++)
        {
            var item = items[i];
            string label = string.IsNullOrEmpty(item.text) ? $"Item{i}" : item.text;

            int col = i % columns;   // 열
            int row = i / columns;   // 행

            float posX = startPos.x + col * (buttonSize.x + spacing);
            float posY = buttonY + row * (buttonSize.y + spacing);

            GUI.enabled = !busy;

            if (GUI.Button(new Rect(posX, posY, buttonSize.x, buttonSize.y), label, buttonStyle))
            {
                StartSessionFlow(item);
            }

            GUI.enabled = true;
        }


        // --- Reset 버튼 (Action 버튼 줄 아래) ---
        if (showResetButton)
        {
            float resetX = startPos.x + spacingwithHeader;
            float resetY = startPos.y;

            // Reset은 busy 중에도 눌려야 해서 GUI.enabled = true 고정
            if (GUI.Button(new Rect(resetX, resetY, resetButtonSize.x, resetButtonSize.y), resetLabel, resetStyle))
            {
                ResetFlow();
            }
        }
    }

    private void StartSessionFlow(Item item)
    {
        if (busy) return;

        if (recorder == null)
        {
            Debug.LogError("[ActionBtn] Recorder is not assigned.");
            return;
        }
        if (videoPlayer == null)
        {
            Debug.LogError("[ActionBtn] VideoPlayer is not assigned.");
            return;
        }
        if (item.video == null)
        {
            Debug.LogError($"[ActionBtn] VideoClip is missing for session: {item.text}");
            return;
        }

        currentActionLabel = item.text;

        // 이전 실행이 남아있으면 정리
        if (flowCoroutine != null)
        {
            StopCoroutine(flowCoroutine);
            flowCoroutine = null;
        }

        flowCoroutine = StartCoroutine(SessionFlowCoroutine(item));
    }



    private IEnumerator SessionFlowCoroutine(Item item)
    {
        busy = true;

        // 1) 버튼 이름 표시
        if (ActionName_Panel != null){
            ActionName_Panel.text = $"{item.text}";
        }

        // countdown 텍스트 초기화
        if (countdownText != null)
            countdownText.text = "";

        // 2) 3초 카운트다운 표시 (3,2,1)
        for (int t = countdownSeconds; t >= 1; t--)
        {
            if (countdownText != null)
                countdownText.text = t.ToString();

            yield return new WaitForSecondsRealtime(1f);
        }

        if (countdownText != null)
            countdownText.text = "";

        // 3) 비디오 실행 & CSV 저장 시작
        // mkv도 csv와 같은 participant 폴더에 저장되게 출력 폴더 생성
        string mkvOutDir = System.IO.Path.Combine(Application.dataPath, recorder.outputFolder, recorder.participantID);
        UDPcontrol.SendCommand($"START_RECORD|{item.text}|{recorder.participantID}|{mkvOutDir}");


        recorder.SetSessionName(item.text);
        recorder.StartRecording();

        // 4) 비디오 재생
        //videoPlayer.Stop();
        //videoPlayer.clip = item.video;
        //videoPlayer.Play();

        //yield return new WaitUntil(() => videoPlayer.isPlaying);
        //yield return new WaitUntil(() => !videoPlayer.isPlaying);


        // 4) 비디오 재생
        videoPlayer.Stop();
        videoPlayer.clip = item.video;

        // FrameCount 기준으로 멈출 거면 루프 켬
        bool stopByRecorderFrame = (stopAtRecordFrame > 0);
        videoPlayer.isLooping = stopByRecorderFrame;

        videoPlayer.Play();
        yield return new WaitUntil(() => videoPlayer.isPlaying);

        if (stopByRecorderFrame)
        {
            // recorder.FrameCount가 stopAtRecordFrame에 도달할 때까지 대기
            // (recordLoop가 WaitForSecondsRealtime로 돌기 때문에, 여기서도 realtime로 기다리는 편이 안정적)
            while (recorder != null && recorder.IsRecording && recorder.FrameCount < stopAtRecordFrame)
            {
                yield return null; // 매 프레임 체크
            }

            // 목표 도달 → 비디오 멈춤
            videoPlayer.isLooping = false;
            videoPlayer.Stop();
        }
        else
        {
            // 기존 동작: 비디오가 끝날 때까지
            yield return new WaitUntil(() => !videoPlayer.isPlaying);
        }



        // 5) 비디오 끝나면 CSV 저장 종료
        UDPcontrol.SendCommand("STOP_RECORD");
        recorder.StopRecording();


        // 6) 종료 표시
        if (countdownText != null)
            countdownText.text = "종료";

        busy = false;
        flowCoroutine = null;
    }

    // Reset 버튼이 호출하는 함수
    private void ResetFlow()
    {
        // 진행 중 코루틴 중단
        if (flowCoroutine != null)
        {
            StopCoroutine(flowCoroutine);
            flowCoroutine = null;
        }

        // 비디오 중단
        if (videoPlayer != null)
            videoPlayer.Stop();


        if (UDPcontrol != null)
        {
            UDPcontrol.SendCommand("RESET_RECORD"); // 외부는 이걸 받으면 stop+rename 처리
        }


        if (recorder != null && recorder.IsRecording)
        {
            recorder.StopRecording();
            recorder.MarkLastFileAsReset();
        }

        // UI 초기화(원하면 sessionNameText는 유지해도 됨)
        if (ActionName_Panel != null) ActionName_Panel.text = "";

        if (countdownText != null)
            countdownText.text = ""; // reset 시 "종료"도 지움

        currentActionLabel = "None";


        // 다시 버튼 선택 가능
        busy = false;
    }



    private void OnDisable()
    {
        Debug.Log("[ActionBtn] OnDisable fired!");

        ResetFlow(); 
    }
}
