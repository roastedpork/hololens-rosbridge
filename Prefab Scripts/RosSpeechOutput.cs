using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.Speech;
using HoloToolkit.Unity;


public class RosSpeechOutput : RosComponent
{

    // ROS communication
    private RosSubscriber<ros.std_msgs.String> sub;
    private RosPublisher<ros.std_msgs.String> pub;

    
    // Voice & Microphone
    private TextToSpeech voicebox;


    private void Start()
    {
        Subscribe("VoiceOutputSub", "/hololens/audio/speech_output", 10, out sub);

        voicebox = GetComponent<TextToSpeech>();
    }

    // Update is called once per frame
    void Update()
    {
        ros.std_msgs.String msg;
        if (Receive(sub, out msg))
        {
            voicebox.StartSpeaking(msg.data);
        }
    }
}
