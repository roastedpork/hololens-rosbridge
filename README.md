# hololens-rosbridge

This repository can be added into a Unity Hololens project and should work off-the-shelf.
It is an easy-to-use client that connects to rosbridge on the ROS master machine via websocket.
Before starting your application on the Hololens, ensure that the rosbridge has been installed on the master ROS machine. To start the rosbridge server, run:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

# Setting up your project

First, create a `Manager` GameObject with the `RosMessenger` script as a child component as such:

![](images/rosmanager.JPG)

There are two parameters that has to be configured:
* `Host`: The IP address of the ROS master node
* `Port`: The port of the rosbridge_websocket server (default: 9090)
