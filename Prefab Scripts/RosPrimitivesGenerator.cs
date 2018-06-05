using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RosPrimitivesGenerator : RosComponent {

    private RosSubscriber<ros.hololens_project.Primitive> PrimitiveSub;
    private RosSubscriber<ros.hololens_project.PrimitiveArray> ArraySub;
    private Dictionary<string, GameObject> PrimitivesDict;

	// Use this for initialization
	void Start () {
        PrimitivesDict = new Dictionary<string, GameObject>();
        Subscribe("PrimitiveGeneratorSub", "/hololens/primitives/generate_single", 5, out PrimitiveSub);
        Subscribe("PrimitiveArraySub", "/hololens/primitives/generate_array", 5, out ArraySub);
    }
	
	// Update is called once per frame
	void Update () {

        ros.hololens_project.Primitive PrimitiveMsg;
        if (Receive(PrimitiveSub, out PrimitiveMsg))
        {
            GameObject temp;
            if (PrimitivesDict.ContainsKey(PrimitiveMsg.id))
            {
                temp = PrimitivesDict[PrimitiveMsg.id];
            }
            else
            {
                switch (PrimitiveMsg.shape)
                {
                    case "capsule":
                        temp = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                        break;
                    case "cube":
                        temp = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        break;
                    case "cylinder":
                        temp = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                        break;
                    case "plane":
                        temp = GameObject.CreatePrimitive(PrimitiveType.Plane);
                        break;
                    case "quad":
                        temp = GameObject.CreatePrimitive(PrimitiveType.Quad);
                        break;
                    case "sphere":
                        temp = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        break;
                    default:
                        temp = null;
                        break;
                }

                if (temp != null) PrimitivesDict[PrimitiveMsg.id] = temp;
            }

            if (temp != null)
            {
                temp.transform.position = PrimitiveMsg.pose.position.AsUnityVector;
                temp.transform.rotation = PrimitiveMsg.pose.orientation.AsUnityQuaternion;
                temp.transform.localScale = PrimitiveMsg.scale.AsUnityVector;
                temp.GetComponent<Renderer>().material.color = PrimitiveMsg.color.AsUnityColor;
            }
        }



        ros.hololens_project.PrimitiveArray ArrayMsg;
        if(Receive(ArraySub, out ArrayMsg))
        {
            foreach (var t in ArrayMsg.primitives)
            {
                GameObject temp;
                if (PrimitivesDict.ContainsKey(t.id))
                {
                    temp = PrimitivesDict[t.id];
                }
                else
                {
                    switch (t.shape)
                    {
                        case "capsule":
                            temp = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                            break;
                        case "cube":
                            temp = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            break;
                        case "cylinder":
                            temp = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                            break;
                        case "plane":
                            temp = GameObject.CreatePrimitive(PrimitiveType.Plane);
                            break;
                        case "quad":
                            temp = GameObject.CreatePrimitive(PrimitiveType.Quad);
                            break;
                        case "sphere":
                            temp = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                            break;
                        default:
                            continue;
                    }

                    PrimitivesDict[t.id] = temp;
                }
                temp.transform.position = t.pose.position.AsUnityVector;
                temp.transform.rotation = t.pose.orientation.AsUnityQuaternion;
                temp.transform.localScale = t.scale.AsUnityVector;
                temp.GetComponent<Renderer>().material.color = t.color.AsUnityColor;  
            }
        }
	}
}
