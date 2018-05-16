using System.Collections.Generic;
using System.Linq;
using SimpleJSON;

namespace ros
{
    namespace sensor_msgs
    {
        public class Joy : IRosClassInterface
        {
            public std_msgs.Header header;
            public List<float> axes;
            public List<int> buttons;

            public Joy()
            {
                header = new std_msgs.Header();
                axes = new List<float>();
                buttons = new List<int>();
            }
            public Joy(std_msgs.Header _header, List<float> _axes, List<int> _buttons)
            {
                header = _header;
                axes = _axes;
                buttons = _buttons;
            }

            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                foreach (var t in msg["axes"].Children)
                {
                    axes.Add((float)t.AsDouble);
                }
                foreach (var t in msg["buttons"].Children)
                {
                    axes.Add((int)t.AsDouble);
                }
            }
            public System.String ToJSON()
            {
                System.String ret = "{";
                ret += "\"header\": " + header.ToJSON() + ", ";
                ret += "\"axes\": [";
                ret += System.String.Join(", ", axes.Select(a => a.ToString()).ToArray());
                ret += "], \"buttons\": [";
                ret += System.String.Join(", ", buttons.Select(a => a.ToString()).ToArray());
                ret += "]}";
                return ret;
            }

        }
    } // sensor_msgs
} // ros