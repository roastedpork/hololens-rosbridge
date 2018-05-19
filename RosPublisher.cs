using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RosPublisher<T>
    where T : IRosClassInterface, new()
{

    private RosMessenger messenger;
    private bool connected = false;

    private Queue<T> SendQueue;

    private string NodeName;
    private string RosTopic;
    private string RosType;
    private int QueueSize;

    public RosPublisher(GameObject manager,
                     String nodeName,
                     String rosTopic,
                     int queueSize = 10)
    {
        messenger = manager.GetComponent<RosMessenger>();
        NodeName = nodeName;
        RosTopic = rosTopic;
        RosType = typeof(T).ToString();
        RosType = RosType.Substring(4, RosType.Length - 4).Replace(".", "/");
        QueueSize = queueSize;
        SendQueue = new Queue<T>();
#if !UNITY_EDITOR
        messenger.Advertise(RosTopic, RosType);
        Debug.Log("[" + NodeName + "] Advertised successfully");
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
                messenger.Publish(RosTopic, msg);

                Debug.Log("[" + NodeName + "] Publishing: " + msg);
            }

            String processed = RosMsg.Encode(data);
            messenger.Publish(RosTopic, processed);

            Debug.Log("[" + NodeName + "] Publishing: " + processed);
#endif
        } else
        {
            SendQueue.Enqueue(data);
            while (SendQueue.Count > QueueSize)
            {
                SendQueue.Dequeue();
            }
        }
    }
}
