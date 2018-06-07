using System;
using System.Collections;
using HoloToolkit.Unity;
using HoloToolkit.Unity.InputModule;
using UnityEngine;

public class RosScanManager : Singleton<RosScanManager>
{
    public TextMesh InstructionTextMesh;
    public bool DrawMesh = true;

    public GameObject SpatialMappingPrefab;
    public GameObject SpatialUnderstandingPrefab;

    private GameObject smHandler;
    private GameObject suHandler;
    private GameObject floor;



    public bool IsScanning { get; private set; }

    // Use this for initialization
    void Start()
    {
        floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.transform.position = new Vector3(0, Parameters.FloorDepth, 0);
        floor.transform.localScale = new Vector3(1, 1, 1) * 30;
        floor.GetComponent<MeshRenderer>().enabled = false;
        IsScanning = false;
        InstructionTextMesh.text = "Say \"Begin scan\" to begin searching for floor depth";
        StartCoroutine(WaitForSpeechInit());
    }

    private IEnumerator WaitForSpeechInit()
    {
        yield return new WaitUntil(() => RosUserSpeechManager.Instance != null);

        RosUserSpeechManager.Instance.AddNewPhrase("Begin scan", () =>
        {
            if (floor != null) Destroy(floor);
            RosUserSpeechManager.Instance.voicebox.StartSpeaking("Scanning for floor depth");
            StartScan();
        });

        RosUserSpeechManager.Instance.AddNewPhrase("Stop scan", () =>
        {
            RosUserSpeechManager.Instance.voicebox.StartSpeaking("Stopping scan");
            StopScan();

            floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.transform.position = new Vector3(0, Parameters.FloorDepth, 0);
            floor.transform.localScale = new Vector3(1, 1, 1) * 30;
            floor.GetComponent<MeshRenderer>().enabled = false;
        });
    }


    public void StartScan()
    {
        if (HoloToolkit.Unity.SpatialMapping.SpatialMappingManager.Instance == null)
        {
            smHandler = Instantiate(SpatialMappingPrefab);
            smHandler.GetComponent<HoloToolkit.Unity.SpatialMapping.SpatialMappingManager>().DrawVisualMeshes = false;
            suHandler = Instantiate(SpatialUnderstandingPrefab);
            suHandler.GetComponent<SpatialUnderstandingCustomMesh>().CreateMeshColliders = DrawMesh;

            IsScanning = true;

            InstructionTextMesh.text = "Scanning for floor depth...";
        }
    }

    public void StopScan()
    {
        if (IsScanning)
        {
            SpatialUnderstanding.Instance.RequestFinishScan();
            HoloToolkit.Unity.SpatialMapping.SpatialMappingManager.Instance.StopObserver();
            Destroy(smHandler);
            Destroy(suHandler);
        }

        IsScanning = false;
        InstructionTextMesh.text = "";
    }


    // Update is called once per frame
    void Update()
    {
        if(IsScanning) InstructionTextMesh.text = "Current Floor Depth: " + Parameters.FloorDepth.ToString();
        if (IsScanning && RosGazeManager.Instance.Focused)
        {
            Parameters.FloorDepth = (Parameters.FloorDepth > RosGazeManager.Instance.position.y) ? RosGazeManager.Instance.position.y : Parameters.FloorDepth;    
        }
    }
}