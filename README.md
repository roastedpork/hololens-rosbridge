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

There are two parameters that has to be configured:
* `Host`: The IP address of the ROS master node
* `Port`: The port of the rosbridge_websocket server (default: 9090)

# Creating a custom script

The Unity-generated scripts can be used with a few modifications.
Ensure that the custom script inherits from `RosComponent` and not `MonoBehaviour, and that the following modifications are performed.

```csharp
public class TestObject : RosComponent
{

    void Start() {
        StartCoroutine(WaitForRosMessengerInitialisation());
        StartCoroutine(WaitUntilRosMessengerConnected());

        //...
    }
}
```

Since GameObjects are instantiated in a random order, the modification will pause the execution of `Start` of this specific GameObject component until the Hololens has successfully connected to the rosbridge server. However, this also means that the instantiation of variables in the later part of the `Start` function would be delayed as well. This is a known cause of `NullReferenceExceptions`, and will be addressed in the future.

At this stage, the `TestObject` component will have a new `Ros Manager` field in the Unity Editor:

![](images/setup.JPG)






















