using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.Speech;
using HoloToolkit.Unity;

public class RosUserSpeechManager : ros.Singleton<RosUserSpeechManager>
{
    public RosPublisher<ros.std_msgs.String> pub;
    public AudioSource StartBeep;
    public AudioSource StopBeep;


    // Voice & Microphone
    public TextToSpeech voicebox { get; private set; }
    private KeywordRecognizer keywordRecognizer;
    private DictationRecognizer dictationRecognizer;

    private Dictionary<String, Action> Keywords = new Dictionary<string, Action>();

    private void Start()
    {
        Advertise("VoicePub", "/hololens/audio/user_transcript", 1, out pub);

        voicebox = gameObject.GetComponent<TextToSpeech>();

        // Activation phrase for dictation
        Keywords.Add("Hello", () =>
        {
            ros.std_msgs.String msg = new ros.std_msgs.String("Hello!");
            if (pub != null) pub.SendMessage(msg);
            voicebox.StartSpeaking("Hello");
        });

        Keywords.Add("record this", () =>
        {
            PhraseRecognitionSystem.Shutdown();
            StartBeep.Play();
            dictationRecognizer.Start();
        });

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
        StopBeep.Play();
        PhraseRecognitionSystem.Restart();
        keywordRecognizer.Start();
    }

    private void DictationHypothesis(string text)
    {

    }

    private void DictationError(string error, int hresult)
    {

    }



    // Update is called once per frame
    void Update()
    {

    }
}
