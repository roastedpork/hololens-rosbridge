using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class RosComponent : MonoBehaviour
{
    public GameObject RosManager;
    protected Dictionary<System.String, System.Double> prevTimeStamp;
    protected Dictionary<System.String, System.Double> period;

    public void Awake()
    {
        prevTimeStamp = new Dictionary<string, double>();
        period = new Dictionary<string, double>();
    }


    protected bool Subscribe<T>(string name, string topic, double rate, out RosSubscriber<T> subscriber)
        where T : IRosClassInterface, new()
    {
        if (prevTimeStamp.ContainsKey(name))
        {
            subscriber = default(RosSubscriber<T>);
            return false;
        }

        prevTimeStamp[name] = Time.unscaledDeltaTime;
        period[name] = 1 / rate;
        subscriber = new RosSubscriber<T>(RosManager, name, topic, 1);
        return true;
    }

    protected bool Advertise<T>(string name, string topic, double rate, out RosPublisher<T> publisher)
        where T : IRosClassInterface, new()
    {
        if (prevTimeStamp.ContainsKey(name))
        {
            publisher = default(RosPublisher<T>);
            return false;
        }
        prevTimeStamp[name] = Time.unscaledDeltaTime;
        period[name] = 1 / rate;
        publisher = new RosPublisher<T>(RosManager, name, topic, 1);
        return true;
    }


    // Throttles message publishing rate to a defined period, to be used within the Update function
    protected bool Publish<T>(RosPublisher<T> publisher, T data)
        where T : IRosClassInterface, new()
    {
        System.Double currTimeStamp = Time.unscaledTime;
        if(currTimeStamp - prevTimeStamp[publisher.name] > period[publisher.name])
        {
            publisher.SendMessage(data);
            prevTimeStamp[publisher.name] = currTimeStamp;
            return true;
        }
        return false;
    }
    
    protected bool Receive<T>(RosSubscriber<T> subscriber, out T message)
        where T : IRosClassInterface, new()
    {
        System.Double currTimeStamp = Time.unscaledTime;

        if (subscriber.MsgReady && 
            (currTimeStamp - prevTimeStamp[subscriber.name] > period[subscriber.name]))
        {
            message = subscriber.GetNewMessage();
            return true;
        }

        message = default(T);
        return false;
    }

































    // DEPRECATED
    protected IEnumerator WaitForRosMessengerInitialisation(string nodename = "")
    {
        Debug.Log("[" + nodename + "] Waiting until RosMessenger to be initialised");
        yield return new WaitUntil(() => RosManager.GetComponent<RosMessenger>() != null);

        Debug.Log("[" + nodename + "] Waiting until RosMessenger is connected to rosbridge...");
        RosMessenger messenger = RosManager.GetComponent<RosMessenger>();
        yield return new WaitUntil(() => messenger.Con);

        Debug.Log("[" + nodename + "] Connected to rosbridge");
    }

    // DEPRECATED
    protected IEnumerator WaitUntilRosMessengerConnected(string nodename = "")
    {
        RosMessenger messenger = RosManager.GetComponent<RosMessenger>();
        Debug.Log("[" + nodename + "] Waiting until RosMessenger is connected to rosbridge...");
        yield return new WaitUntil(() => messenger.Con);
        Debug.Log("[" + nodename + "] Connected to rosbridge");

    }

}
