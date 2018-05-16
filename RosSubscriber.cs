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
                         String rosType,
                         int queueSize = 10)
    {
        // During instantiation, it is assumed that the RosManager instance is ready and connected
        RosMessenger messenger = manager.GetComponent<RosMessenger>();
        NodeName = nodeName;
        RosTopic = rosTopic;
        RosType = rosType;
        QueueSize = queueSize;
        MsgQueue = new Queue<T>();

#if !UNITY_EDITOR
        messenger.Subscribe(RosTopic, RosType);
        buffer = messenger.topicBuffer[RosTopic];
        Debug.Log("[" + NodeName + "] Subscribed successfully");
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

/*
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RosSubscriber : MonoBehaviour {

    private Type unityType;
    private RosMessenger messenger;
    private Queue<String> buffer;
    private Queue<IRosClassInterface> MsgQueue; // Queue for processed results. Warning: Type casting is required once the message is dequeued.
   

    // Note: Edit these values in the Unity Editor!!!
    public GameObject RosManager;
    public string NodeName = "RosSubscriber";
    public string RosTopic = "/hololens/template_topic";
    public string RosType  = "std_msgs/Empty";
    public int QueueSize = 10;

    public bool MsgReady { get { return MsgQueue.Count > 0; } }


    private void Awake()
    {
        buffer = new Queue<String>();
        MsgQueue = new Queue<IRosClassInterface>();
    }
    // Use this for initialization
    private void Start ()
    {
        messenger = RosManager.GetComponent<RosMessenger>();
        StartCoroutine(WaitUntilConnected());
    }

    // Subscribes once RosMessenger is connected
    private IEnumerator WaitUntilConnected()
    {
        Debug.Log("[" + NodeName + "] Waiting until RosMessenger is connected to rosbridge...");
        yield return new WaitUntil(() => messenger.Con);
        Debug.Log("[" + NodeName + "] Connected to rosbridge, now subscribing to '" + RosTopic + "'");

#if !UNITY_EDITOR
        messenger.Subscribe(RosTopic, RosType);
        buffer = messenger.topicBuffer[RosTopic];
#endif
        Debug.Log("[" + NodeName + "] Subscribed successfully");
    }

    
    private void Update () {


#if !UNITY_EDITOR
        if(unityType == null)
        {
            unityType = RosMsg.GetUnityType(RosType);
        }
        while ((buffer != null) && (buffer.Count > 0) &&(unityType != null))
        { 
            // Custom parser to interpret the JSON data into Unity datatypes
            string data = buffer.Dequeue();
            IRosClassInterface processed = (IRosClassInterface)RosMsg.Decode<unityType>(data);
            MsgQueue.Enqueue(processed);

            Debug.Log("[" + NodeName + "] Received: " + processed.ToString());

            while(MsgQueue.Count > QueueSize)
            {
                MsgQueue.Dequeue(); // Removes the oldest messages to obtain QueueSize latest messages
            }
        }
#endif
    }

    public IRosClassInterface GetNewMessage()
    {
        return MsgQueue.Dequeue();
    }
}
*/