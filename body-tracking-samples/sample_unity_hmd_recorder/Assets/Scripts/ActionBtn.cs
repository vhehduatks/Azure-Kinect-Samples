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

    [Header("Data")]
    public List<Item> items = new List<Item>();

    [Header("References")]
    public HMDDataRecorder recorder;
    public RecordingSyncController UDPcontrol;
    public VideoPlayer videoPlayer;

    [Header("UI (TextMeshPro)")]
    public TMP_Text ActionName_Panel;   // 선택된 버튼 이름 표시
    public TMP_Text countdownText;     // 3 2 1 / 종료 표시

    [Header("Flow")]
    public int countdownSeconds = 3;

    [Header("GUI Header Text")]
    public Vector2 startPos = new Vector2(20f, 20f);
    public bool showHeaderText = true;

    private GUIStyle headerStyle;
    private string participantIdCached = "";
    private string currentActionLabel = "None";

    [Header("IMGUI Layout (Top-Left Row)")]
    public Vector2 buttonSize = new Vector2(160f, 40f);
    public float spacing = 12f;

    [Header("Reset Button")]
    public bool showResetButton = true;
    public string resetLabel = "Reset";
    public Vector2 resetButtonSize = new Vector2(160f, 40f);



    private GUIStyle buttonStyle;
    private GUIStyle resetStyle;

    private bool busy = false;               // 진행 중이면 추가 클릭 막기
    private Coroutine flowCoroutine;

    private void Start()
    {
        // participant ID는 시작 시 한 번만 가져와서 고정
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
        // --- Action 버튼들 (가로 일렬) ---
        for (int i = 0; i < items.Count; i++)
        {
            var item = items[i];
            string label = string.IsNullOrEmpty(item.text) ? $"Item{i}" : item.text;

            GUI.enabled = !busy; // 진행 중이면 버튼 비활성화
            if (GUI.Button(new Rect(x, buttonY, buttonSize.x, buttonSize.y), label, buttonStyle))
            {
                StartSessionFlow(item);
            }
            GUI.enabled = true;

            x += buttonSize.x + spacing;
        }

        // --- Reset 버튼 (Action 버튼 줄 아래) ---
        if (showResetButton)
        {
            float resetX = startPos.x;
            float resetY = buttonY + 20f + buttonSize.y;

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
        //UDPcontrol.SendCommand($"SET_PARTICIPANT {recorder.participantID}");
        UDPcontrol.SendCommand($"START_RECORD {item.text}");
        //UDPcontrol.SendCommand("START_RECORD");

        recorder.SetSessionName(item.text);
        recorder.StartRecording();

        // Video 준비/재생
        videoPlayer.Stop();
        videoPlayer.clip = item.video;
        videoPlayer.isLooping = false;
        videoPlayer.Play();

        // 4) 비디오 끝날 때까지 대기
        yield return new WaitUntil(() => videoPlayer.isPlaying);
        yield return new WaitUntil(() => !videoPlayer.isPlaying);

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
        ResetFlow(); // 안전 정리
    }
}
