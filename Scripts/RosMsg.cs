using UnityEngine;
using System;
using System.Collections.Generic;
using SimpleJSON;


public interface IRosClassInterface
{
    void FromJSON(JSONNode json);
    String ToJSON();
}


public static class RosMsg
{
    public static T Decode<T>(JSONNode msg)
        where T : IRosClassInterface, new()
    {
        T ret = new T();
        ret.FromJSON(msg);
        return ret;
        
    }

    public static System.String Encode(IRosClassInterface msg)
    {
        return msg.ToJSON();
    }

}
