using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class RosComponent : MonoBehaviour
{
    public GameObject RosManager;
   
    protected IEnumerator WaitForRosMessengerInitialisation(string nodename = "")
    {
        Debug.Log("[" + nodename + "] Waiting until RosMessenger to be initialised");
        yield return new WaitUntil(() => RosManager.GetComponent<RosMessenger>() != null);

        Debug.Log("[" + nodename + "] Waiting until RosMessenger is connected to rosbridge...");
        RosMessenger messenger = RosManager.GetComponent<RosMessenger>();
        yield return new WaitUntil(() => messenger.Con);

        Debug.Log("[" + nodename + "] Connected to rosbridge");
    }

    protected IEnumerator WaitUntilRosMessengerConnected(string nodename = "")
    {
        RosMessenger messenger = RosManager.GetComponent<RosMessenger>();
        Debug.Log("[" + nodename + "] Waiting until RosMessenger is connected to rosbridge...");
        yield return new WaitUntil(() => messenger.Con);
        Debug.Log("[" + nodename + "] Connected to rosbridge");

    }

}
