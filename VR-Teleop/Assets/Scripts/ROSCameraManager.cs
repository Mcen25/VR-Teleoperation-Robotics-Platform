using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;
using System.Collections.Generic;

/// <summary>
/// Manages multiple ROS camera subscriptions and displays them in a grid
/// Uses ROS# (RosSharp) to connect directly to ROS topics
/// </summary>
public class ROSCameraManager : MonoBehaviour
{
    [Header("ROS Connection")]
    [SerializeField] private string rosBridgeServerUrl = "ws://100.114.29.13:9090";
    [SerializeField] private RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.Newtonsoft_JSON;
    
    [Header("Camera Topics")]
    [SerializeField] private List<CameraTopicConfig> cameraTopics = new List<CameraTopicConfig>()
    {
        new CameraTopicConfig { topic = "/camera0/image_raw", name = "USB_Cam_0" },
        new CameraTopicConfig { topic = "/camera1/image_raw", name = "USB_Cam_1" },
        new CameraTopicConfig { topic = "/camera_0/color/image_raw", name = "D435_Cam_0" },
        new CameraTopicConfig { topic = "/camera_1/color/image_raw", name = "D435_Cam_1" },
    };
    
    [Header("Display Settings")]
    [SerializeField] private bool autoCreateDisplay = true;
    [SerializeField] private int gridColumns = 2;
    [SerializeField] private Vector2 cameraDisplaySize = new Vector2(640, 480);
    [SerializeField] private Vector3 gridPosition = new Vector3(0, 1.5f, 2f);
    [SerializeField] private float gridScale = 0.001f;
    
    [Header("Status")]
    public bool isConnected = false;
    public int activeCameras = 0;
    
    private RosConnector rosConnector;
    private List<GameObject> cameraDisplays = new List<GameObject>();

    [System.Serializable]
    public class CameraTopicConfig
    {
        public string topic;
        public string name;
    }

    void Start()
    {
        Debug.Log("ROSCameraManager: Initializing ROS camera system...");
        
        // Create ROS connector
        SetupRosConnection();
        
        // Create camera displays
        if (autoCreateDisplay)
        {
            CreateCameraDisplayGrid();
        }
    }

    void SetupRosConnection()
    {
        // Add RosConnector component
        rosConnector = gameObject.AddComponent<RosConnector>();
        
        // Configure ROS Bridge connection
        rosConnector.RosBridgeServerUrl = rosBridgeServerUrl;
        rosConnector.Serializer = serializer;
        
        // Set protocol (auto-detect websocket/webSocket secure)
        if (rosBridgeServerUrl.StartsWith("wss://"))
        {
            rosConnector.protocol = Protocol.WebSocketSharp;
        }
        else
        {
            rosConnector.protocol = Protocol.WebSocketNET;
        }
        
        Debug.Log($"Connecting to ROS Bridge at: {rosBridgeServerUrl}");
        
        // The RosConnector will automatically attempt to connect
        // Check connection status after a delay
        Invoke(nameof(CheckConnection), 3f);
    }

    void CheckConnection()
    {
        if (rosConnector != null && rosConnector.IsConnected != null)
        {
            isConnected = rosConnector.IsConnected.WaitOne(0);
            
            if (isConnected)
            {
                Debug.Log("✓ Connected to ROS Bridge successfully!");
            }
            else
            {
                Debug.LogWarning("⚠ Not connected to ROS Bridge. Check if rosbridge_server is running.");
                Debug.LogWarning($"Expected: roslaunch rosbridge_server rosbridge_websocket.launch");
                // Retry connection
                Invoke(nameof(CheckConnection), 5f);
            }
        }
    }

    void CreateCameraDisplayGrid()
    {
        Debug.Log("Creating camera display grid...");
        
        // Create world-space canvas
        GameObject canvasObj = new GameObject("ROSCameraCanvas");
        canvasObj.transform.SetParent(transform);
        
        Canvas canvas = canvasObj.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.transform.localPosition = gridPosition;
        canvas.transform.localRotation = Quaternion.identity;
        canvas.transform.localScale = new Vector3(gridScale, gridScale, gridScale);
        
        canvasObj.AddComponent<CanvasScaler>();
        canvasObj.AddComponent<GraphicRaycaster>();
        
        // Calculate grid layout
        int rows = Mathf.CeilToInt((float)cameraTopics.Count / gridColumns);
        
        for (int i = 0; i < cameraTopics.Count; i++)
        {
            int row = i / gridColumns;
            int col = i % gridColumns;
            
            // Create camera display panel
            GameObject displayObj = new GameObject($"Camera_{i}_{cameraTopics[i].name}");
            displayObj.transform.SetParent(canvas.transform);
            
            // Add RosConnector reference to each camera display
            displayObj.AddComponent<RosConnector>().CopyFrom(rosConnector);
            
            // Add camera subscriber
            var subscriber = displayObj.AddComponent<ROSCameraSubscriber>();
            subscriber.SetCameraTopic(cameraTopics[i].topic);
            subscriber.SetCameraName(cameraTopics[i].name);
            
            // Create background panel
            GameObject panelObj = new GameObject("Panel");
            panelObj.transform.SetParent(displayObj.transform);
            
            Image panel = panelObj.AddComponent<Image>();
            panel.color = new Color(0.1f, 0.1f, 0.1f, 1f);
            
            RectTransform panelRect = panelObj.GetComponent<RectTransform>();
            
            // Position in grid
            float xPos = col * (cameraDisplaySize.x + 20) - (gridColumns * cameraDisplaySize.x) / 2;
            float yPos = -row * (cameraDisplaySize.y + 20) + (rows * cameraDisplaySize.y) / 2;
            
            panelRect.anchoredPosition = new Vector2(xPos, yPos);
            panelRect.sizeDelta = cameraDisplaySize;
            
            // Create RawImage for camera feed
            GameObject imageObj = new GameObject("CameraImage");
            imageObj.transform.SetParent(panelObj.transform);
            
            RawImage rawImage = imageObj.AddComponent<RawImage>();
            RectTransform imageRect = imageObj.GetComponent<RectTransform>();
            imageRect.anchorMin = Vector2.zero;
            imageRect.anchorMax = Vector2.one;
            imageRect.offsetMin = new Vector2(10, 50);
            imageRect.offsetMax = new Vector2(-10, -10);
            
            // Create label
            GameObject labelObj = new GameObject("Label");
            labelObj.transform.SetParent(panelObj.transform);
            
            Text label = labelObj.AddComponent<Text>();
            label.text = cameraTopics[i].name;
            label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            label.fontSize = 24;
            label.color = Color.yellow;
            label.alignment = TextAnchor.MiddleCenter;
            
            RectTransform labelRect = labelObj.GetComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0, 0);
            labelRect.anchorMax = new Vector2(1, 0);
            labelRect.pivot = new Vector2(0.5f, 0);
            labelRect.anchoredPosition = new Vector2(0, 10);
            labelRect.sizeDelta = new Vector2(-20, 40);
            
            // Assign display elements to subscriber
            subscriber.GetType().GetField("displayImage", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                ?.SetValue(subscriber, rawImage);
            subscriber.GetType().GetField("labelText",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                ?.SetValue(subscriber, label);
            
            cameraDisplays.Add(displayObj);
            activeCameras++;
        }
        
        Debug.Log($"Created {cameraDisplays.Count} camera displays");
    }

    void OnDestroy()
    {
        // Cleanup
        foreach (var display in cameraDisplays)
        {
            if (display != null)
            {
                Destroy(display);
            }
        }
        cameraDisplays.Clear();
    }

    // Public API
    public void SetRosBridgeUrl(string url)
    {
        rosBridgeServerUrl = url;
    }

    public void AddCameraTopic(string topic, string name)
    {
        cameraTopics.Add(new CameraTopicConfig { topic = topic, name = name });
    }
}

// Extension method to copy RosConnector settings
public static class RosConnectorExtensions
{
    public static void CopyFrom(this RosConnector target, RosConnector source)
    {
        if (source != null && target != null)
        {
            target.RosBridgeServerUrl = source.RosBridgeServerUrl;
            target.Serializer = source.Serializer;
            target.protocol = source.protocol;
        }
    }
}
