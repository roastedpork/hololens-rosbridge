//Here the connection and interaction between Unity/HL and ROS is made.

//Two different WebSockets libraries are used to establish a connection.
//WebSocketSharp provides a websocket client that allow us to connect Unity with ROS via RosBridge. 
//Since this library it is not available for Windows Store Apps (HL format), Windows.Networking.Socket is responsible
//for connecting HL and ROS.

//Pieces of code wrapped in #if UNITY_EDITOR are used only when Unity Play mode is running.
//Pieces of code wrapped in #if !UNITY_EDITOR are used only when the app is running on HL.

//Methodology for establishing an async Rosbridge connection during Unity play mode and to build a message responsible for accessing and 
//sending a ROS service message were given here: 
//github.com/2016UAVClass/Simulation-Unity3D @author Michael Jenkin, Robert Codd-Downey and Andrew Speers

using UnityEngine;
using Scripts;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text;
using SimpleJSON;

#if !UNITY_EDITOR
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
using Windows.Networking;
using Windows.Foundation;
using Windows.UI.Core;
using System.Threading.Tasks;
#endif

public class RosMessenger : Singleton<RosMessenger>
{

    
    public string host = "192.168.100.127"; // IP address of ROS Master
    public int port    = 9090;              // default for rosbridge_websocket


    //State variables
    private bool busy = false;
    public bool Con { get; private set; }


    // List of advertised/subscribed ROS topics
    private List<string> advertiseList;
    private List<string> subscribeList;

    // List of queued ROS messages from subscribed topics
    public Dictionary<string, Queue<JSONNode>> topicBuffer;
    public Dictionary<string, string> topicType;

    //WebSocket client from Windows.Networking.Sockets

#if !UNITY_EDITOR
    private MessageWebSocket messageWebSocket;
    Uri server;
    DataWriter dataWriter;
#endif

    // Default Unity Object Methods
    public void Awake()
    {
        Con = false;
        Connect();
        advertiseList = new List<string>();
        subscribeList = new List<string>();
        topicBuffer   = new Dictionary<string, Queue<JSONNode>>();
        topicType     = new Dictionary<string, string>();
    }

    public void Update()
    {
    }

    public void OnDestroy()
    {
#if !UNITY_EDITOR
        foreach (string topic in advertiseList)
        {
            Unadvertise(topic);
        }
        foreach (string topic in subscribeList)
        {
            Unsubscribe(topic);
        }
        Disconnect();
#endif
    }


    // HL-rosbridge connection methods
    public void Connect()
    {
        //Async connection.
        if (!Con && !busy)
        {
            busy = true;
            Debug.Log("Connecting to rosbridge...");


#if !UNITY_EDITOR
            messageWebSocket = new MessageWebSocket();
            messageWebSocket.Control.MessageType = SocketMessageType.Utf8;
            messageWebSocket.MessageReceived += Win_MessageReceived;

            server = new Uri("ws://" + host + ":" + port.ToString());

            IAsyncAction outstandingAction = messageWebSocket.ConnectAsync(server);
            AsyncActionCompletedHandler aach = new AsyncActionCompletedHandler(NetworkConnectedHandler);
            outstandingAction.Completed = aach;
#endif
        }
    }

#if !UNITY_EDITOR
    //Successful network connection handler on HL
    public void NetworkConnectedHandler(IAsyncAction asyncInfo, AsyncStatus status)
    {
        // Status completed is successful.
        if (status == AsyncStatus.Completed)
        {
            //Guarenteed connection
            Con = true;
            Debug.Log("Connected to ROS!");
            busy = false;

            //Creating the writer that will be repsonsible to send a message through Rosbridge
            dataWriter = new DataWriter(messageWebSocket.OutputStream);

        }
        else
        {
            Con = false;
        }
    }

    private void Win_MessageReceived(MessageWebSocket sender, MessageWebSocketMessageReceivedEventArgs args)
    {
        try
        {
            DataReader messageReader = args.GetDataReader();
            messageReader.UnicodeEncoding = Windows.Storage.Streams.UnicodeEncoding.Utf8;
            string messageString = messageReader.ReadString(messageReader.UnconsumedBufferLength);
            ParseMessage(messageString);
        }
        catch (InvalidCastException e)
        {

        }
        //Add code here to do something with the string that is received.
    }

    private void ParseMessage(string str)
    {
        var N = JSON.Parse(str);
        if (subscribeList.Contains(N["topic"]))
        {
            topicBuffer[N["topic"]].Enqueue(N["msg"]);
            Debug.Log("Raw message: " + str + "\n\rReceived message from '" + N["topic"] + "' topic:\n\r'" + N["msg"].ToString() + "'");
        }
        else
        {
            // Debug.Log("Received unknown message");
        }
    }

    public void Disconnect()
    {
        //Killing connection
        messageWebSocket.Dispose();
        messageWebSocket = null;
        Con = false;
    }


    // Methods to handle rosbridge messaging
    public async Task Send(string str)
    {
        busy = true;
        await WebSock_SendMessage(messageWebSocket, str);
        busy = false;
    }
    private async Task WebSock_SendMessage(MessageWebSocket webSock, string message)
    {
       dataWriter.WriteString(message);
       await dataWriter.StoreAsync();
       
    }

    public void Subscribe(string topic, string type){
        string msg = "{\"op\": \"subscribe\", \"topic\": \"" + topic + "\",\"type\": \"" + type + "\"}";
        Send(msg);
        // Debug.Log("Subscribing to '" + topic + "' topic (" + type + ")");
        subscribeList.Add(topic);
        topicBuffer[topic] = new Queue<JSONNode>();
        topicType[topic] = type;
    }

    public void Unsubscribe(string topic){
        string msg = "{\"op\": \"unsubscribe\", \"topic\": \"" + topic + "\"}";
        Send(msg);
        // Debug.Log("Unsubscribing from '" + topic + "' topic");
        subscribeList.Remove(topic);
    }

    public void Advertise(string topic, string type){
        string msg = "{\"op\": \"advertise\", \"topic\": \"" + topic + "\",\"type\": \"" + type + "\"}";
        Send(msg);
        // Debug.Log("Advertising on '" + topic + "' topic (" + type + ")");
        advertiseList.Add(topic);
    }

    public void Unadvertise(string topic){
        string msg = "{\"op\": \"unadvertise\", \"topic\": \"" + topic + "\"}";
        Send(msg);
        // Debug.Log("Unadvertising from '" + topic + "' topic");
        advertiseList.Remove(topic);
    }

    public void Publish(string topic, string str)
    {
        string msg = "{\"op\": \"publish\", \"topic\": \"" + topic + "\", \"msg\": " + str + "}";
        Send(msg);
        Debug.Log("Publishing '" + msg + "'");
    }

#endif

}