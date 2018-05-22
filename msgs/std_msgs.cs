using SimpleJSON;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ros
{
    namespace std_msgs
    {
        public class Bool : IRosClassInterface
        {
            public System.Boolean data;

            // Constructor Functions
            public Bool()
            {
                data = false;
            }
            public Bool(System.Boolean val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = msg["data"].AsBool;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        public class Byte : IRosClassInterface
        {
            public System.Byte data;

            // Constructor Functions
            public Byte()
            {
                data = 0;
            }
            public Byte(System.Byte val)
            {
                data = val;
            }
            
            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (System.Byte)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{ \"data\": "+ data.ToString() +"}";
            }
        }

        // TODO: Implement ByteMultiArray class

        public class Char : IRosClassInterface
        {
            public System.Char data;

            // Constructor Functions
            public Char()
            {
                data = ' ';
            }
            public Char(System.Char val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = msg["data"].Value.ToCharArray()[0];
            }
            public System.String ToJSON()
            {
                return "{ \"data\": " + data.ToString() + "}";
            }
        }

        public class ColorRGBA : IRosClassInterface
        {
            public float r;
            public float g;
            public float b;
            public float a;

            public UnityEngine.Color AsUnityColor { get { return new UnityEngine.Color(r, g, b, a); } }

            // Constructor Functions
            public ColorRGBA()
            {
                r = 0;
                g = 0;
                b = 0;
                a = 0;
            }
            public ColorRGBA(float _r, float _g, float _b, float _a)
            {
                r = _r;
                g = _g;
                b = _b;
                a = _a;
            }

            // IRosClassInterface
            public void FromJSON(JSONNode msg)
            {
                r = (float)msg["r"].AsDouble;
                g = (float)msg["g"].AsDouble;
                b = (float)msg["b"].AsDouble;
                a = (float)msg["a"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{ "
                        + "\"r\": " + r.ToString("F3") + ", "
                        + "\"g\": " + g.ToString("F3") + ", "
                        + "\"b\": " + b.ToString("F3") + ", "
                        + "\"a\": " + a.ToString("F3") + "}";
            }
        }

        // TODO: Implement Duration class

        public class MultiArrayDimension : IRosClassInterface
        {
            public System.String label;
            public System.UInt32 size;
            public System.UInt32 stride;

            // Constructor Functions
            public MultiArrayDimension()
            {
                label = "";
                size = 0;
                stride = 0;
            }
            public MultiArrayDimension(System.String _label, System.UInt32 _size, System.UInt32 _stride)
            {
                label = _label;
                size = _size;
                stride = _stride;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                label = msg["label"].Value;
                size = (System.UInt32)msg["size"].AsDouble;
                stride = (System.UInt32)msg["stride"].AsDouble;
            }
            public System.String ToJSON()
            {
                
                return "{" 
                       + "\"label\": \"" + label + "\", "
                       + "\"size\": " + size.ToString() + ", "
                       + "\"stride\": " + stride.ToString() + "}";
            }
        }

        public class MultiArrayLayout : IRosClassInterface
        {
            public List<MultiArrayDimension> dim;
            public System.UInt32 data_offset;

            // Constructor Functions
            public MultiArrayLayout()
            {
                // TODO: Implement implement this somehow
            }
            public MultiArrayLayout(List<MultiArrayDimension> _dim, System.UInt32 _data_offset)
            {
                dim = _dim;
                data_offset = _data_offset;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                dim = new System.Collections.Generic.List<MultiArrayDimension>();
                foreach (var t in msg["dim"].Children)
                {
                    dim.Add(new MultiArrayDimension(t["label"].Value,
                                                    (System.UInt32)t["size"].AsDouble,
                                                    (System.UInt32)t["stride"].AsDouble));
                }

                data_offset = (System.UInt32)msg["data_offset"].AsDouble;
            }
            public System.String ToJSON()
            {
                System.String ret = "{";

                ret += "\"dim\": [";
                ret += System.String.Join(", ", dim.Select(a => a.ToJSON()).ToArray());
                ret += "], \"data_offset\" : " + data_offset.ToString() + "}";
                return ret;
            }
        }
        
        public class Float32 : IRosClassInterface
        {
            public float data;

            // Constructor Functions
            public Float32()
            {
                data = 0.0f;
            }
            public Float32(float val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (float)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        // TODO: Implement Float32MultiArray class
        
        public class Float64 :IRosClassInterface
        {
            public System.Double data;

            // Constructor Functions
            public Float64()
            {
                data = 0.0;
            }
            public Float64(System.Double val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }
        
        // TODO: Implement Float64MultiArray class

        public class Header :IRosClassInterface
        {
            public System.Int32 seq;
            public System.Double stamp;
            public System.String frame_id;

            // Constructor Functions
            public Header()
            {
                seq = new System.Int32();
                stamp = new System.Double();
                frame_id = " ";
            }
            public Header(int _seq, double _stamp, string _id)
            {
                seq = _seq;
                stamp = _stamp;
                frame_id = _id;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                seq = (System.Int32)msg["seq"].AsDouble;
                stamp = msg["stamp"]["secs"].AsDouble + msg["stamp"]["nsecs"].AsDouble * 1e-9;
                frame_id = msg["frame_id"].Value;
            }
            public System.String ToJSON()
            {
                return "{"
                       + "\"seq\": " + seq.ToString() + ", "
                       + "\"stamp\": " + stamp.ToString() + ", "
                       + "\"frame_id\": \"" + frame_id + "\"}";
            }
        }

        public class Int16 :IRosClassInterface
        {
            public System.Int16 data;

            // Constructor Functions
            public Int16()
            {
                data = 0;
            }
            public Int16(System.Int16 val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (System.Int16)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        // TODO: Implement Int16MultiArray class

        public class Int32 : IRosClassInterface
        {
            public System.Int32 data;

            // Constructor Functions
            public Int32()
            {
                data = 0;
            }
            public Int32(System.Int32 val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (System.Int32)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        // TODO: Implement Int32MultiArray class

        public class Int64 : IRosClassInterface
        {
            public System.Int64 data;
            
            // Constructor Functions
            public Int64()
            {
                data = 0;
            }
            public Int64(System.Int64 val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (System.Int64)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        // TODO: Implement Int64MultiArray class

        public class String : IRosClassInterface
        {
            public System.String data;

            // Constructor Functions
            public String()
            {
                data = "";
            }
            public String(System.String val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = msg["data"].Value;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : \"" + data + "\"}";
            }

        }

        // TODO: Implement Time class

        // TODO: Implement UInt8 class
        // TODO: Implement UInt8MultiArray class

        public class UInt16 : IRosClassInterface
        {
            public System.UInt16 data;

            // Constructor Functions
            public UInt16()
            {
                data = 0;
            }
            public UInt16(System.UInt16 val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (System.UInt16)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        // TODO: Implement UInt16MultiArray class

        public class UInt32 : IRosClassInterface
        {
            public System.UInt32 data;

            // Constructor Functions
            public UInt32()
            {
                data = 0;
            }
            public UInt32(System.UInt32 val)
            {
                data = val;
            }
            
            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (System.UInt32)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        // TODO: Implement UInt32MultiArray class

        public class UInt64 : IRosClassInterface
        {
            public System.UInt64 data;

            // Constructor Functions
            public UInt64()
            {
                data = 0;
            }
            public UInt64(System.UInt64 val)
            {
                data = val;
            }

            // IRosClassInterface Implementation
            public void FromJSON(JSONNode msg)
            {
                data = (System.UInt64)msg["data"].AsDouble;
            }
            public System.String ToJSON()
            {
                return "{\"data\" : " + data.ToString() + "}";
            }

        }

        // TODO: Implement UInt64MultiArray class
        
    } // std_msgs
} // ros
