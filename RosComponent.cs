using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class RosComponent : MonoBehaviour
{
    public GameObject RosManager;
    private System.Double prevTimeStamp;
   
    public void Awake()
    {
        prevTimeStamp = Time.unscaledTime;
    }

    // Throttles message publishing rate to a defined period, to be used within the Update function
    protected void Publish<T>(RosPublisher<T> publisher, T data, System.Double period)
        where T : IRosClassInterface, new()
    {
        System.Double currTimeStamp = Time.unscaledTime;
        if(currTimeStamp - prevTimeStamp > period)
        {
            publisher.SendMessage(data);
            prevTimeStamp = currTimeStamp;
        }
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
