# hololens-rosbridge

This repository can be added into a Unity Hololens project and should work off-the-shelf.
It is an easy-to-use API that connects to rosbridge on the ROS master machine via websocket, based on the [holoROS](https://github.com/soliagabriel/holoROS) project done by Solia Gabriel.
Before starting your application on the Hololens, ensure that the rosbridge ROS package has been installed on the master ROS machine. To start the rosbridge server, run:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

# Setting up your project

First, create a `Manager` GameObject with the `RosMessenger` script as a child component as such:

![](images/rosmanager.JPG)

There are three parameters within RosMessenger:
* `Host`: The IP address of the ROS master node
* `Port`: The port of the rosbridge_websocket server (default: 9090)
* `Activation List`: A dynamic list of GameObject that publish/subscribe ROS topics

It is vital that all GameObjects that communicate with rosbridge are set to inactive at the start of the application, and added into the `Activation List` parameter. RosMessenger will automatically activate the given list of GameObjects, as this will prevent any attempts to publish/subscribe before RosMessenger is properly set up.


# Creating a custom script

The Unity-generated scripts can be used with a few modifications.
Ensure that the custom script inherits from RosComponent and not MonoBehaviour to inherit the `RosManager` member variable. 

```csharp
public class TestObject : RosComponent
{
        //...
}
```

At this stage, the `TestObject` component will have a new `Ros Manager` field in the Unity Editor:

![](images/setup.JPG)

Simply drag the `Manager` GameObject from the Hierarchy Window into the `Ros Manager` field in order to create the reference.

# Subscribing and Publishing

The following is an example of how to use a RosSubscriber and RosPublisher, which is similar to the C++ syntax. Note that `RosManager` is a member variable of `RosComponent` and has to be passed to RosSubscriber/RosPublisher whenever they are being instantiated. Unlike the C++/Python implementations, there are no callback functions as that is handled by RosMessenger.

```csharp
public class TestObject : RosComponent
{
    RosSubscriber<ros.std_msgs.String> sub;
    RosPublisher<ros.std_msgs.String> pub;

    string RosSubTopic = "hololens/test_sub_topic"; // Example subscribed topic
    string RosSubType = "std_msgs/String";          // Example message type

    // ...

    void Start(){
        RosSubscriber<ros.std_msgs.String> sub = new RosSubscriber<ros.std_msgs.String>(RosManager,
                                                                                        SubName,
                                                                                        RosSubTopic,
                                                                                        RosSubType);

        RosSubscriber<ros.std_msgs.String> sub = new RosSubscriber<ros.std_msgs.String>(RosManager,
                                                                                        PubName,
                                                                                        RosPubTopic,
                                                                                        RosPubType);
        // ...

    }

    void Update(){ 
        // To receive a message
        if(sub.MsgReady){
            ros.std_msgs.String msg = sub.GetNewMessage();
        }

        // ...
        
        // To publish a message
        ros.std_msgs.String resp = ... ;
        pub.SendMessage(resp);
    }

}
```

Currently, most of `std_msgs` and `geometry_msgs` have been implemented. More will be included in the future if necessary.
# Custom Messages

Custom messages are required to inherit the `IRosClassInterface` interface:

```csharp
public interface IRosClassInterface
{
    void FromJSON(JSONNode json);
    String ToJSON();
}
```
This is because the messages received from the rosbridge server is sent in JSON format, and each message type has to be handled appropriately. To get an idea of how to implement this, you can inspect `std_msgs` or `geometry_msgs`. The messages are also required to be under the `ros.custom_msgs` namespace:

```csharp
using namespace ros{

    using namespace custom_msgs{

        public class CustomMsg : IRosClassInterface{
            // ...
        }

    }
}
```
































