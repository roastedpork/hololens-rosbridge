using System;
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



    public bool IsScanning { get; private set; }

    // Use this for initialization
    void Start()
    {
        IsScanning = false;
        InstructionTextMesh.text = "Say \"Begin scan\" to begin searching for floor depth";
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
        if (IsScanning && RosGazeManager.Instance.Focused)
        {
            Parameters.FloorDepth = (Parameters.FloorDepth > RosGazeManager.Instance.position.y) ? RosGazeManager.Instance.position.y : Parameters.FloorDepth;
            InstructionTextMesh.text = "Current Floor Depth: " + Parameters.FloorDepth.ToString();
        }
    }
}