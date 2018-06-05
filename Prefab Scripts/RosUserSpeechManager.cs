using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.Speech;
using HoloToolkit.Unity;

public class RosUserSpeechManager : ros.Singleton<RosUserSpeechManager>
{
    public RosPublisher<ros.std_msgs.String> pub;

    // Voice & Microphone
    private TextToSpeech voicebox;
    private KeywordRecognizer keywordRecognizer;
    private DictationRecognizer dictationRecognizer;

    private Dictionary<String, Action> Keywords = new Dictionary<string, Action>();

    private void Start()
    {
        Advertise("VoicePub", "/hololens/audio/user_transcript", 10, out pub);

        voicebox = gameObject.GetComponent<TextToSpeech>();

        // Activation phrase for dictation
        Keywords.Add("Hello", () =>
        {
            ros.std_msgs.String msg = new ros.std_msgs.String("Hello!");
            if (pub != null) pub.SendMessage(msg);
            voicebox.StartSpeaking("Hello");
        });

        Keywords.Add("hey robot", () =>
        {
            PhraseRecognitionSystem.Shutdown();
            voicebox.StartSpeaking("how can I help you?");
            dictationRecognizer.Start();
        });

        /*
        keywords.Add("Move there", () => {
            if (!wpManager.gameObject.activeSelf) voicebox.StartSpeaking("Waypoint manager is not active");
            else if (!wpManager.AddingMultipleWaypoints) wpManager.SingleWaypoint();
        });
        keywords.Add("Add point", () => {
            if (!wpManager.gameObject.activeSelf) voicebox.StartSpeaking("Waypoint manager is not active");
            else if (wpManager.AddWaypoint())
            {
                voicebox.StartSpeaking("Point added");
            }
            else
            {
                voicebox.StartSpeaking("Could not add point");
            }

        });
        keywords.Add("goodbye", () => {
            if (!wpManager.gameObject.activeSelf) voicebox.StartSpeaking("Waypoint manager is not active");
            else if (wpManager.AddingMultipleWaypoints)
            {
                wpManager.PublishWaypoints();
                voicebox.StartSpeaking("Moving along path");
            }
            else
            {
                voicebox.StartSpeaking("This should not happen");
            }
        });
        /*
        // Activation phrases for setting world markers
        keywords.Add("Set marker one", () =>
        {
            if (!alignManager.gameObject.activeSelf)
            {
                voicebox.StartSpeaking("Alignment manager is not active");
            }
            else if (alignManager.SetMarker(1))
            {
                voicebox.StartSpeaking("Marker one set");
            }
            else
            {
                voicebox.StartSpeaking("Could not set marker one");
            }
        });

        keywords.Add("Set marker two", () =>
        {
            if (!alignManager.gameObject.activeSelf)
            {
                voicebox.StartSpeaking("Alignment manager is not active");
            }
            else if (alignManager.SetMarker(2))
            {
                voicebox.StartSpeaking("Marker two set");
            }
            else
            {
                voicebox.StartSpeaking("Could not set marker two");
            }
        });

        keywords.Add("Set marker three", () =>
        {
            if (!alignManager.gameObject.activeSelf)
            {
                voicebox.StartSpeaking("Alignment manager is not active");
            }
            else if (alignManager.SetMarker(3))
            {
                voicebox.StartSpeaking("Marker three set");
            }
            else
            {
                voicebox.StartSpeaking("Could not set marker three");
            }
        });

        keywords.Add("Begin scan", () =>
        {
            if (ScanManager.Instance.isActiveAndEnabled)
            {
                voicebox.StartSpeaking("Starting scan");
                ScanManager.Instance.StartScan();
                //Parameters.FloorDepth = 0.0f;
            }
            else
            {
                voicebox.StartSpeaking("Scan Manager is not active");
            }
        });


        keywords.Add("Stop scan", () =>
        {
            if (ScanManager.Instance.isActiveAndEnabled)
            {
                voicebox.StartSpeaking("Stopping scan");
                ScanManager.Instance.StopScan();

                GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
                floor.transform.position = new Vector3(0, Parameters.FloorDepth, 0);
                floor.transform.localScale = new Vector3(1, 1, 1) * 30;
                floor.GetComponent<MeshRenderer>().enabled = false;
            }
            else
            {
                voicebox.StartSpeaking("Scan Manager is not active");
            }
        });
        */

        dictationRecognizer = new DictationRecognizer();
        dictationRecognizer.DictationComplete += DictationComplete;
        dictationRecognizer.DictationError += DictationError;
        dictationRecognizer.DictationHypothesis += DictationHypothesis;
        dictationRecognizer.DictationResult += DictationResult;

        keywordRecognizer = new KeywordRecognizer(Keywords.Keys.ToArray());
        keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
        keywordRecognizer.Start();


    }

    private void KeywordRecognizer_OnPhraseRecognized(PhraseRecognizedEventArgs args)
    {
        Action keywordAction;
        if (Keywords.TryGetValue(args.text, out keywordAction))
        {
            keywordAction.Invoke();
        }
    }


    public void AddNewPhrase(String phrase, Action action)
    {
        Keywords.Add(phrase, action);
        keywordRecognizer.OnPhraseRecognized -= KeywordRecognizer_OnPhraseRecognized;
        keywordRecognizer.Dispose();

        keywordRecognizer = new KeywordRecognizer(Keywords.Keys.ToArray());
        keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
        keywordRecognizer.Start();
    }


    private void DictationResult(String text, ConfidenceLevel confidence)
    {
        ros.std_msgs.String transcript = new ros.std_msgs.String(text);
        Publish(pub, transcript);
        dictationRecognizer.Stop();
    }

    private void DictationComplete(DictationCompletionCause cause)
    {
        PhraseRecognitionSystem.Restart();
        keywordRecognizer.Start();
    }

    private void DictationHypothesis(string text)
    {

    }

    private void DictationError(string error, int hresult)
    {
        /*
        dictationRecognizer.DictationComplete -= DictationComplete;
        dictationRecognizer.DictationError -= DictationError;
        dictationRecognizer.DictationHypothesis -= DictationHypothesis;
        dictationRecognizer.DictationResult -= DictationResult;
        dictationRecognizer.Dispose();
        dictationRecognizer.Stop();
        keywordRecognizer.Start();
        */
    }



    // Update is called once per frame
    void Update()
    {

    }
}
