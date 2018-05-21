using System.Collections.Generic;
using System.Linq;
using SimpleJSON;

namespace ros
{
    namespace nav_msgs
    {
        public class GridCells : IRosClassInterface
        {
            public std_msgs.Header header;
            public float cell_width;
            public float cell_height;
            public List<geometry_msgs.Point> cells;

            public GridCells()
            {
                header = new std_msgs.Header();
                cell_width = 0;
                cell_height = 0;
                cells = new List<geometry_msgs.Point>();
            }

            public GridCells(std_msgs.Header _header, float _cell_width, float _cell_height, List<geometry_msgs.Point> _cells)
            {
                header = _header;
                cell_width = _cell_width;
                cell_height = _cell_height;
                cells = _cells;
            }

            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                cell_width = (float)msg["cell_width"].AsDouble;
                cell_height = (float)msg["cell_height"].AsDouble;
                foreach (var t in msg["cells"].Children)
                {
                    geometry_msgs.Point temp = new geometry_msgs.Point();
                    temp.FromJSON(t);
                    cells.Add(temp);
                }
            }

            public System.String ToJSON()
            {
                System.String ret = "{";
                ret += "\"header\": " + header.ToJSON() + ", ";
                ret += "\"cell_width\": " + cell_width.ToString("F3") + ", ";
                ret += "\"cell_height\": " + cell_height.ToString("F3") + ", ";
                ret += "\"cells\": [";
                ret += System.String.Join(", ", cells.Select(a => a.ToJSON()).ToArray());
                ret += "]}";
                return ret;
            }

        }

    } // nav_msgs
} // ros