using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using SimpleJSON;

namespace ros
{
    namespace hololens_project
    {
        public class Obstacle : IRosClassInterface
        {
            public geometry_msgs.Point rel_position;
            public System.Double width;
            public System.Double height;
            public System.Double box_angle;

            public Obstacle()
            {
                rel_position = new geometry_msgs.Point();
                width = 0;
                height = 0;
                box_angle = 0;
            }

            public Obstacle(geometry_msgs.Point _rel_position, float _width, float _height, float _box_angle)
            {
                rel_position = _rel_position;
                width = _width;
                height = _height;
                box_angle = _box_angle;
            }

            public void FromJSON(JSONNode msg)
            {
                rel_position.FromJSON(msg["rel_position"]);
                width = msg["width"].AsDouble;
                height = msg["height"].AsDouble;
                box_angle = msg["box_angle"].AsDouble;
                
            }

            public System.String ToJSON()
            {
                System.String ret = "{";
                ret += "\"rel_position\": " + rel_position.ToJSON() + ", ";
                ret += "\"width\": " + width.ToString() + ", ";
                ret += "\"height\": " + height.ToString() + ", ";
                ret += "\"box_angle\": " + box_angle.ToString() + "}";
                return ret;
            }

        }

        public class ObstacleArray : IRosClassInterface
        {
            public std_msgs.Header header;
            public List<Obstacle> obstacles;
            public ObstacleArray()
            {
                header = new std_msgs.Header();
                obstacles = new List<Obstacle>();
            }

            public ObstacleArray(std_msgs.Header _header, List<Obstacle> _obstacles)
            {
                header = _header;
                obstacles = _obstacles;
            }

            public void FromJSON(JSONNode msg)
            {
                header.FromJSON(msg["header"]);
                foreach (var t in msg["obstacles"].Children)
                {
                    Obstacle temp = new Obstacle();
                    temp.FromJSON(t);
                    obstacles.Add(temp);
                }

            }

            public System.String ToJSON()
            {
                System.String ret = "{";
                ret += "\"rel_position\": " + header.ToJSON() + ", ";
                ret += "\"obstacles\": [";
                ret += System.String.Join(", ", obstacles.Select(a => a.ToJSON()).ToArray()); 
                ret += "]} ";
                return ret;
            }
        }
        
    } // hololens_drive
} // ros