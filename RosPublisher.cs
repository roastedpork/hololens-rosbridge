using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RosPublisher<T>
    where T : IRosClassInterface, new()
{

    private bool connected = false;

    private Queue<T> SendQueue;

    public string name;
    private string RosTopic;
    private string RosType;
    private int QueueSize;
    
    public RosPublisher(String nodeName,
                        String rosTopic,
                        float rate = 20,
                        int queueSize = 10)
    {

        name = nodeName;
        RosTopic = rosTopic;
        RosType = typeof(T).ToString();
        RosType = RosType.Substring(4, RosType.Length - 4).Replace(".", "/");
        QueueSize = queueSize;
        SendQueue = new Queue<T>();
                
#if !UNITY_EDITOR
        RosMessenger.Instance.Advertise(RosTopic, RosType);
        Debug.Log("[" + name + "] Advertised successfully");
#endif
        connected = true;

    }

    public void SendMessage(T data)
    {
        if (connected)
        {
#if !UNITY_EDITOR
            // Custom parser to interpret the JSON data into Unity datatypes
            if (SendQueue.Count > 0)
            {
                String msg = RosMsg.Encode(SendQueue.Dequeue());
                RosMessenger.Instance.Publish(RosTopic, msg);

                Debug.Log("[" + name + "] Publishing: " + msg);
            }

            String processed = RosMsg.Encode(data);
            RosMessenger.Instance.Publish(RosTopic, processed);

            Debug.Log("[" + name + "] Publishing: " + processed);
#endif
        }
        else
        {
            SendQueue.Enqueue(data);
            while (SendQueue.Count > QueueSize)
            {
                SendQueue.Dequeue();
            }
        }
    }
}
