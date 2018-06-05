using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RosGazeManager : HoloToolkit.Unity.Singleton<RosGazeManager> {

    
    public float MaxRange = 10.0f;
    public bool Focused { get; private set; }
    public GameObject FocusedObject { get; private set; }
    public Vector3 position { get; private set; }
    public Vector3 normal { get; private set; }
    public float distance { get; private set; }

    private GameObject Pointer;

    // Use this for initialization
    void Start()
    {
        Pointer = transform.Find("Pointer").gameObject;
    }

    // Update is called once per frame
    void Update()
    {

        RaycastHit hitInfo;
        Focused = Physics.Raycast(Camera.main.transform.position, Camera.main.transform.forward, out hitInfo, MaxRange, Physics.DefaultRaycastLayers);
        if (Focused)
        {
            FocusedObject = hitInfo.collider.gameObject;
            position = hitInfo.point;
            normal = hitInfo.normal;
            distance = hitInfo.distance;


            Pointer.SetActive(true);
            Pointer.transform.position = hitInfo.point;
        }
        else
        {
            FocusedObject = null;
            position = new Vector3();
            normal = new Vector3();
            distance = 0;

            Pointer.SetActive(false);
        }



    }
}
