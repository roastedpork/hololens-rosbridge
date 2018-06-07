using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RosImageSubscriber : RosComponent
{
    private const String valuemap = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    private RawImage rawImage;
    private RosSubscriber<ros.std_msgs.String> sub;

    public String ImageTopic = "/hololens/display/image";
    public double SubscriptionRate = 10;

    // Use this for initialization
    void Start()
    {
        Subscribe("RosImageSubscriber", ImageTopic, SubscriptionRate, out sub);
        
        rawImage = transform.Find("RawImage").GetComponent<RawImage>();
        rawImage.color = new Color(1, 1, 1, 0);
    }

    byte[] DecodeString(String str)
    {
        List<byte> buff = new List<byte>();
        int pad = str.Count(c => c == '=');

        String strip = str.Replace("=", "A");

        for (int i = 0; i < strip.Length; i += 4)
        {
            String chunk = strip.Substring(i, 4);
            byte[] base64 = new byte[4];

            for (int j = 0; j < 4; j++)
            {
                char c = chunk[j];
                base64[j] = (byte)valuemap.IndexOf(c);
            }

            buff.Add((byte)((base64[0] << 2) + (base64[1] >> 4)));
            buff.Add((byte)((base64[1] << 4) + (base64[2] >> 2)));
            buff.Add((byte)((base64[2] << 6) + (base64[3])));
        }
        for (int i = 0; i < pad; i++) buff.RemoveAt(buff.Count - 1);
        return buff.ToArray();
    }

    // Update is called once per frame
    void Update()
    {
        ros.std_msgs.String mapMsg;
        

        if (Receive(sub, out mapMsg))
        {
            String encoded = mapMsg.data;
            byte[] image = DecodeString(encoded);

            Texture2D tex = new Texture2D(2, 2);
            tex.LoadImage(image);
            rawImage.texture = tex;
            rawImage.color = new Color(1, 1, 1, 1);

        }
    }
}
