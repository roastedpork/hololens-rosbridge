using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using SimpleJSON;

namespace ros
{
    namespace geometry_msgs
    {
        public class Accel : IRosClassInterface
        {
            public Vector3 linear;
            public Vector3 angular;


            // Constructor Functions
            public Accel()
            {
                linear = new Vector3();
                angular = new Vector3
                {
                    IsRotationVector = true
                };
            }
            public Accel(Vector3 _linear, Vector3 _angular)
            {
                linear = _linear;
                angular = _angular;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                linear.FromJSON(msg["linear"]);
                angular.FromJSON(msg["angular"]);
            }
            public System.String ToJSON()
            { 
                return "{"
                       + "\"linear\": " + linear.ToJSON() + ", "
                       + "\"angular\": " + angular.ToJSON() + "}";
            }

        }

        public class AccelStamped :IRosClassInterface
        {
            public std_msgs.Header header;
            public Accel accel;

            // Constructor Functions
            public AccelStamped()
            {
                header = new std_msgs.Header();
                accel = new Accel();
            }
            public AccelStamped(std_msgs.Header _header, Accel _accel)
            {
                header = _header;
                accel = _accel;
            }

            // IRosClassImplementation
            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                accel.FromJSON(msg["accel"]);
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"header\": " + header.ToJSON() + ", "
                       + "\"accel\": " + accel.ToJSON() + "}";
            }
        }

        // TODO: Implement AccelWithCovariance class
        
        // TODO: Implement AccelWithCovarianceStamped class

        // TODO: Implement Inertia class

        // TODO: Implement InertiaStamped class

        public class Point :IRosClassInterface
        {
            public System.Double x;
            public System.Double y;
            public System.Double z;

            public UnityEngine.Vector3 AsUnityVector
            {
                // Because Unity world axes are screwed up                                                    
                get
                {
                    return new UnityEngine.Vector3((float)x,
                                                   (float)z,
                                                   (float)y);
                }
            }

            // Constructor Functions
            public Point()
            {
                x = 0;
                y = 0;
                z = 0;
            }
            public Point(System.Double _x, System.Double _y, System.Double _z)
            {
                x = _x;
                y = _y;
                z = _z;
            }
            public Point(UnityEngine.Vector3 vector)
            {
                x = vector.x;
                y = vector.z;
                z = vector.y;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                x = msg["x"].AsDouble;
                y = msg["y"].AsDouble;
                z = msg["z"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"x\": " + x.ToString("F3") + ", "
                       + "\"y\": " + y.ToString("F3") + ", "
                       + "\"z\": " + z.ToString("F3") + "}";
            }
            
            
        }

        public class Point32 : IRosClassInterface
        {
            public float x;
            public float y;
            public float z;

            public UnityEngine.Vector3 AsUnityVector
            {
                // Because Unity world axes are screwed up                                                    
                get
                {
                    return new UnityEngine.Vector3(x, z, y);
                }
            }

            // Constructor Functions
            public Point32()
            {
                x = 0;
                y = 0;
                z = 0;
            }
            public Point32(System.Double _x, System.Double _y, System.Double _z)
            {
                x = (float)_x;
                y = (float)_y;
                z = (float)_z;
            }
            public Point32(UnityEngine.Vector3 vector)
            {
                x = vector.x;
                y = vector.z;
                z = vector.y;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                x = (float)msg["x"].AsDouble;
                y = (float)msg["y"].AsDouble;
                z = (float)msg["z"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"x\": " + x.ToString("F3") + ", "
                       + "\"y\": " + y.ToString("F3") + ", "
                       + "\"z\": " + z.ToString("F3") + "}";
            }

        }

        public class PointStamped : IRosClassInterface
        {
            public std_msgs.Header header;
            public Point point;

            // Constructor Functions
            public PointStamped()
            {
                header = new std_msgs.Header();
                point = new Point();
            }
            public PointStamped(std_msgs.Header _header, Point _point)
            {
                header = _header;
                point = _point;
            }

            //IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                point.FromJSON(msg["Point"]);
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"header\": " + header.ToJSON() + ", "
                       + "\"point\": " + point.ToJSON() + "}";
            }
        }

        public class Polygon : IRosClassInterface
        {
            public List<Point32> points;

            // Constructor Functions
            public Polygon()
            {
                points = new List<Point32>();
            }
            public Polygon(List<Point32> _points)
            {
                points = _points;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                points = new List<Point32>();
                foreach (var t in msg["points"].Children)
                {
                    Point32 temp = new Point32();
                    temp.FromJSON(t);
                    points.Add(temp);
                }
            }
            public System.String ToJSON()
            {
                System.String ret = "{";
                ret += "\"points\": [";
                ret += System.String.Join(", ", points.Select(a => a.ToJSON()).ToArray());
                ret += "]}";
                return ret;
            }
        }

        public class PolygonStamped : IRosClassInterface
        {
            public std_msgs.Header header;
            public Polygon polygon;

            // Constructor Functions
            public PolygonStamped()
            {
                header = new std_msgs.Header();
                polygon = new Polygon();
            }
            public PolygonStamped(std_msgs.Header _header, Polygon _polygon)
            {
                header = _header;
                polygon = _polygon;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                polygon.FromJSON(msg["polygon"]);
            }
            public System.String ToJSON()
            {
                System.String ret = "{";
                ret += "\"header\": " + header.ToJSON() + ", ";
                ret += "\"polygon\": " + polygon.ToJSON() + "}";
                return ret;
            }

        }

        public class Pose : IRosClassInterface
        {
            public Point position;
            public Quaternion orientation;

            public UnityEngine.Pose AsUnityPose {
                get { return new UnityEngine.Pose(position.AsUnityVector,
                                                  orientation.AsUnityQuaternion);
                }
            }

            // Constructor Functions
            public Pose()
            {
                position = new Point();
                orientation = new Quaternion();
            }
            public Pose(Point _pos, Quaternion _orient)
            {
                position = _pos;
                orientation = _orient;
            }

            public Pose(UnityEngine.Pose _pose)
            {
                position = new Point(_pose.position);
                orientation = new Quaternion(_pose.rotation);
            }

            public Pose(UnityEngine.Vector3 _position, UnityEngine.Quaternion _rotation)
            {
                position = new Point(_position);
                orientation = new Quaternion(_rotation);
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                position.FromJSON(msg["position"]);
                orientation.FromJSON(msg["orientation"]);
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"position\": " + position.ToJSON() + ", "
                       + "\"orientation\": " + orientation.ToJSON() + "}";
            }
        }

        public class Pose2D : IRosClassInterface
        {
            public System.Double x;
            public System.Double y;
            public System.Double theta;

            public UnityEngine.Pose AsUnityPose
            {
                get
                {
                    UnityEngine.Vector3 p = new UnityEngine.Vector3((float)x, 0, (float)y);
                    UnityEngine.Quaternion q = UnityEngine.Quaternion.Euler(0, -(float) theta, 0);
                    return new UnityEngine.Pose(p, q);
                }
            }

            // Constructor Functions
            public Pose2D()
            {
                x = 0;
                y = 0;
                theta = 0;
                
            }
            public Pose2D(System.Double _x, System.Double _y, System.Double _theta)
            {
                x = _x;
                y = _y;
                theta = _theta;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                x = msg["x"].AsDouble;
                y = msg["y"].AsDouble;
                theta = msg["theta"].AsDouble;

            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"x\": " + x.ToString("F3") + ", "
                       + "\"y\": " + y.ToString("F3") + ", "
                       + "\"theta\": " + theta.ToString("F3") + "}";
            }
        }

        public class PoseArray : IRosClassInterface
        {
            public std_msgs.Header header;
            public List<Pose> poses;

            // Constructor Functions
            public PoseArray()
            {
                header = new std_msgs.Header();
                poses = new List<Pose>();
            }
            public PoseArray(std_msgs.Header _head, List<Pose> _poses)
            {
                header = _head;
                poses = _poses;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                foreach (var t in msg["poses"].Children)
                {
                    Pose temp = new Pose();
                    temp.FromJSON(t);
                    poses.Add(temp);
                }
            
            }
            public System.String ToJSON()
            {
                System.String ret = "{";
                ret += "\"header\": " + header.ToJSON() + ", ";
                ret += "\"poses\": [";
                ret += System.String.Join(", ", poses.Select(a => a.ToJSON()).ToArray());
                ret += "]}";
                return ret;
            }
        }

        public class PoseStamped : IRosClassInterface
        {
            public std_msgs.Header header;
            public Pose pose;

            // Constructor Functions
            public PoseStamped()
            {
                header = new std_msgs.Header();
                pose = new Pose();
            }
            public PoseStamped(std_msgs.Header _head, Pose _pose)
            {
                header = _head;
                pose = _pose;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                pose.FromJSON(msg["pose"]);
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"header\": " + header.ToJSON() + ", "
                       + "\"pose\": " + pose.ToJSON() + "}";
            }

        }

        // TODO: Implement PoseWithCovariance class
        
        // TODO: Implement PoseWithCovarianceStamped class

        public class Quaternion : IRosClassInterface
        {
            public System.Double x;
            public System.Double y;
            public System.Double z;
            public System.Double w;

            public UnityEngine.Quaternion AsUnityQuaternion
            {
                // Unity axes are swapped and rotating in the opposite direction
                get { return new UnityEngine.Quaternion((float)x,
                                                        (float)z,
                                                        (float)y,
                                                        -(float)w);
                }    
            }

            // Constructor Functions
            public Quaternion()
            {
                x = 0;
                y = 0;
                z = 0;
                w = 1;
            }
            public Quaternion(System.Double _x, System.Double _y, System.Double _z, System.Double _w)
            {
                x = _x;
                y = _y;
                z = _z;
                w = _w;
            }
            public Quaternion(UnityEngine.Quaternion _q)
            {
                x = _q.x;
                y = _q.z;
                z = _q.y;
                w = -_q.w;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                x = msg["x"].AsDouble;
                y = msg["y"].AsDouble;
                z = msg["z"].AsDouble;
                w = msg["w"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"x\": " + x.ToString("F3") + ", "
                       + "\"y\": " + y.ToString("F3") + ", "
                       + "\"z\": " + z.ToString("F3") + ", "
                       + "\"w\": " + w.ToString("F3") + "}";
            }
        }

        public class QuaternionStamped : IRosClassInterface
        {
            public std_msgs.Header header;
            public Quaternion quaternion;

            // Constructor Functions
            public QuaternionStamped()
            {
                header = new std_msgs.Header();
                quaternion = new Quaternion();
            }
            public QuaternionStamped(std_msgs.Header _header, Quaternion _q)
            {
                header = _header;
                quaternion = _q;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                quaternion.FromJSON(msg["quaternion"]);
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"header\": " + header.ToJSON() + ", "
                       + "\"quaternion\":" + quaternion.ToJSON() + "}";
            }
        }

        // TODO: Implement Transform class

        // TODO: Redo TransformStamped class
        /*
        public class TransformStamped
        {
            public Header header;
            public String child_id;
            public Pose transform;

            // Constructor Functions
            public TransformStamped()
            {
                header = new Header();
                child_id = "";
                transform = new Pose();
            }
            public TransformStamped(Header _head, string _id, Pose _trans)
            {
                header = _head;
                child_id = _id;
                transform = _trans;
            }

        }
        */

        // TODO: Implement Twist class
        public class Twist : IRosClassInterface
        {
            public Vector3 linear;
            public Vector3 angular;


            // Constructor Functions
            public Twist()
            {
                linear = new Vector3();
                angular = new Vector3
                {
                    IsRotationVector = true
                };
            }
            public Twist(Vector3 _linear, Vector3 _angular)
            {
                linear = _linear;
                angular = _angular;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                linear.FromJSON(msg["linear"]);
                angular.FromJSON(msg["angular"]);
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"linear\": " + linear.ToJSON() + ", "
                       + "\"angular\": " + angular.ToJSON() + "}";
            }

        }

        // TODO: Implement TwistStamped class

        // TODO: Implement TwistWithCovariance class

        // TODO: Implement TwistWithCovarianceStamped class

        public class Vector3 : IRosClassInterface
        {
            public System.Double x;
            public System.Double y;
            public System.Double z;
            public System.Boolean IsRotationVector;

            public UnityEngine.Vector3 AsUnityVector
            {
                // Because Unity world axes are screwed up                                                    
                get
                {
                    if (IsRotationVector)
                    {
                        return new UnityEngine.Vector3(-(float)x,
                                                       -(float)z,
                                                       -(float)y);
                    }
                    else
                    {
                        return new UnityEngine.Vector3((float)x,
                                                       (float)z,
                                                       (float)y);
                    }
                }
            }

            // Constructor Functions
            public Vector3()
            {
                x = 0;
                y = 0;
                z = 0;
                IsRotationVector = false;
            }
            public Vector3(System.Double _x, System.Double _y, System.Double _z, System.Boolean _rot = false)
            {
                x = _x;
                y = _y;
                z = _z;
                IsRotationVector = _rot;
            }
            public Vector3(UnityEngine.Vector3 vector, System.Boolean isRotationVector = false)
            {
                x = vector.x;
                y = vector.z;
                z = vector.y;
                IsRotationVector = isRotationVector;
                // Unity performs left-handed rotation, this converts it to right-hand rotation
                if (IsRotationVector)
                {
                    x = -x;
                    y = -y;
                    z = -z;
                }
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                x = msg["x"].AsDouble;
                y = msg["y"].AsDouble;
                z = msg["z"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"x\": " + x.ToString("F3") + ", "
                       + "\"y\": " + y.ToString("F3") + ", "
                       + "\"z\": " + z.ToString("F3") + "}";
            }
        }

        public class Vector3Stamped : IRosClassInterface
        {
            public std_msgs.Header header;
            public Vector3 vector;

            // Constructor Functions
            public Vector3Stamped()
            {
                header = new std_msgs.Header();
                vector = new Vector3();
            }
            public Vector3Stamped(std_msgs.Header _header, Vector3 _vec)
            {
                header = _header;
                vector = _vec;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                vector.FromJSON(msg["vector"]);
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"header\": " + header.ToJSON() + ", "
                       + "\"vector\": " + vector.ToJSON() + "}";
            }
        }

        // TODO: Implement Wrench class
        
        // TODO: Implement WrenchStamped class

    } // geometry_msgs
} // ros