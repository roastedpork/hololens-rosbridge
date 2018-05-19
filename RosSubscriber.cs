using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SimpleJSON;

public class RosSubscriber<T>
    where T : IRosClassInterface, new()
{

    private Queue<JSONNode> buffer;
    private Queue<T> MsgQueue;

    private String NodeName;
    private String RosTopic;
    private String RosType;
    private int QueueSize;

    public bool MsgReady
    {
        get
        {
            if (buffer != null)
            {
                return (buffer.Count > 0);
            }
            else
            {
                return false;
            }

        }
    }

    public RosSubscriber(GameObject manager, 
                         String nodeName,
                         String rosTopic,
                         int queueSize = 10)
    {
        // During instantiation, it is assumed that the RosManager instance is ready and connected
        RosMessenger messenger = manager.GetComponent<RosMessenger>();
        NodeName = nodeName;
        RosTopic = rosTopic;
        RosType = typeof(T).ToString(); //rosType;
        RosType = RosType.Substring(4, RosType.Length - 4).Replace(".", "/");
        QueueSize = queueSize;
        MsgQueue = new Queue<T>();

#if !UNITY_EDITOR
        messenger.Subscribe(RosTopic, RosType);
        buffer = messenger.topicBuffer[RosTopic];

        Debug.Log("[" + NodeName + "] Subscribed successfully, message type: " + RosType);
#endif

    }

    public T GetNewMessage()
    {
        while ((buffer != null) && (buffer.Count > 0))
        { 
            // Custom parser to interpret the JSON data into Unity datatypes
            JSONNode data = buffer.Dequeue();
            T processed = RosMsg.Decode<T>(data);
            MsgQueue.Enqueue(processed);

            Debug.Log("[" + NodeName + "] Received: " + processed.ToJSON());

            while(MsgQueue.Count > QueueSize)
            {
                MsgQueue.Dequeue(); // Removes the oldest messages to obtain QueueSize latest messages
            }
        }

        return MsgQueue.Dequeue();
    }
}
