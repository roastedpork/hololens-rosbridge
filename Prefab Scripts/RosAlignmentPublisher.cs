using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RosAlignmentPublisher : RosComponent
{
    public List<GameObject> Markers;
    private UnityEngine.XR.WSA.Persistence.WorldAnchorStore anchorStore;
    private RosPublisher<ros.geometry_msgs.PolygonStamped> pub;

    // Use this for initialization
    void Start()
    {
        Advertise("RosAlignmentPublisher", "/hololens/reference_points", 1, out pub);

        UnityEngine.XR.WSA.Persistence.WorldAnchorStore.GetAsync(AnchorStoreReady);

        StartCoroutine(WaitForSpeechInit());


    }

    private IEnumerator WaitForSpeechInit()
    {
        yield return new WaitUntil(() => RosUserSpeechManager.Instance != null);

        RosUserSpeechManager.Instance.AddNewPhrase("Set marker one", () =>
        {
            if (SetMarker(1))
            {
                RosUserSpeechManager.Instance.voicebox.StartSpeaking("Marker one set");
            }
            else
            {
                RosUserSpeechManager.Instance.voicebox.StartSpeaking("Could not set marker one");
            }
        });
        RosUserSpeechManager.Instance.AddNewPhrase("Set marker two", () =>
        {
            if (SetMarker(2))
            {
                RosUserSpeechManager.Instance.voicebox.StartSpeaking("Marker two set");
            }
            else
            {
                RosUserSpeechManager.Instance.voicebox.StartSpeaking("Could not set marker two");
            }
        });
        RosUserSpeechManager.Instance.AddNewPhrase("Set marker three", () =>
        {
            if (SetMarker(3))
            {
                RosUserSpeechManager.Instance.voicebox.StartSpeaking("Marker three set");
            }
            else
            {
                RosUserSpeechManager.Instance.voicebox.StartSpeaking("Could not set marker three");
            }
        });

    }


    void AnchorStoreReady(UnityEngine.XR.WSA.Persistence.WorldAnchorStore store)
    {
        anchorStore = store;

        string[] ids = anchorStore.GetAllIds();

        foreach (GameObject marker in Markers)
        {

            // Retrieve WorldAnchors for each marker if it exists
            Debug.Log("Looking for " + marker.name + " in WorldAnchorStore");
            UnityEngine.XR.WSA.WorldAnchor wa = anchorStore.Load(marker.name, marker);
            if (wa != null) Debug.Log("Retrieved anchor from WorldAnchorStore");
            else
            {
                // Set new WorldAnchor if no existing WorldAnchor found
                Debug.Log("No WorldAnchor for " + marker.name + " found, creating new WorldAnchor");
                UnityEngine.XR.WSA.WorldAnchor attachingAnchor = marker.AddComponent<UnityEngine.XR.WSA.WorldAnchor>();
                if (attachingAnchor.isLocated)
                {
                    bool saved = anchorStore.Save(marker.name, attachingAnchor);
                    Debug.Log("Saved new WorldAnchor position for " + marker.name + ": " + saved);
                }
                else
                {
                    attachingAnchor.OnTrackingChanged += AttachingAnchor_OnTrackingChanged;
                }
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        ros.geometry_msgs.PolygonStamped msg = new ros.geometry_msgs.PolygonStamped();
        msg.header.frame_id = "/Unity";


        foreach (GameObject obj in Markers)
        {
            ros.geometry_msgs.Point32 temp = new ros.geometry_msgs.Point32(obj.transform.position.x, obj.transform.position.z, Parameters.FloorDepth);
            msg.polygon.points.Add(temp);
        }

        Publish(pub, msg);

    }


    public bool SetMarker(int marker)
    {
        int idx = marker - 1;
        if (marker > Markers.Count) return false;


        //RaycastHit hitInfo;
        /*Physics.Raycast(Camera.main.transform.position, Camera.main.transform.forward, out hitInfo)*/
        if (RosGazeManager.Instance.Focused)
        {
            if (anchorStore == null) return false;

            // Delete WorldAnchor if selected marker currently has one
            UnityEngine.XR.WSA.WorldAnchor anchor = Markers[idx].GetComponent<UnityEngine.XR.WSA.WorldAnchor>();
            if (anchor != null) DestroyImmediate(anchor);
            anchorStore.Delete(Markers[idx].name);

            // Move marker to new position
            Markers[idx].transform.position = RosGazeManager.Instance.position;

            // Set new WorldAnchor
            UnityEngine.XR.WSA.WorldAnchor attachingAnchor = Markers[idx].AddComponent<UnityEngine.XR.WSA.WorldAnchor>();
            if (attachingAnchor.isLocated)
            {
                bool saved = anchorStore.Save(Markers[idx].name, attachingAnchor);
                Debug.Log("Saved new WorldAnchor position for " + Markers[idx].name + ": " + saved);
            }
            else
            {
                attachingAnchor.OnTrackingChanged += AttachingAnchor_OnTrackingChanged;
            }

            // return success
            return true;
        }


        return false;
    }

    private void AttachingAnchor_OnTrackingChanged(UnityEngine.XR.WSA.WorldAnchor anchor, bool located)
    {
        if (located)
        {
            bool saved = anchorStore.Save(anchor.gameObject.name, anchor);
            Debug.Log("Saved WorldAnchor for " + anchor.gameObject.name + " in callback: " + saved);
            anchor.OnTrackingChanged -= AttachingAnchor_OnTrackingChanged;
        }
    }

}
