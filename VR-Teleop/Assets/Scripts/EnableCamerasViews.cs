using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.Networking;
using System;

/// <summary>
/// Connects to ROS camera topics and displays camera feeds in Unity VR scene
/// Supports both USB cameras and D435 depth cameras
/// </summary>
public class EnableCamerasViews : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    [SerializeField] private string rosHost = "100.114.29.13"; // Robot IP address
    [SerializeField] private int rosPort = 8080; // Port from LiveCamerasWeb Python script
    [SerializeField] private bool useSSHTunnel = false; // Use SSH connection for security (disabled - using direct Tailscale connection)
    
    [Header("Camera Display Settings")]
    [SerializeField] private RawImage[] cameraDisplays; // Assign RawImage UI elements in inspector
    [SerializeField] private float refreshRate = 30f; // FPS for camera updates
    
    [Header("Camera Grid Layout")]
    [SerializeField] private bool autoCreateCameraGrid = true;
    [SerializeField] private int gridColumns = 3;
    [SerializeField] private Vector2 cameraDisplaySize = new Vector2(480, 360);
    
    [Header("Connection Status")]
    public bool isConnected = false;
    [SerializeField] private Text statusText; // Optional: Display connection status
    
    // Camera data storage
    private Dictionary<string, Texture2D> cameraTextures = new Dictionary<string, Texture2D>();
    private List<string> availableCameras = new List<string>();
    
    // ROS camera topic names
    private readonly string[] usbCameraTopics = new string[] 
    { 
        "/camera0/image_raw", "/camera1/image_raw", "/camera2/image_raw",
        "/camera3/image_raw", "/camera4/image_raw", "/camera5/image_raw"
    };
    
    private readonly string[] d435CameraTopics = new string[] 
    { 
        "/camera_0/color/image_raw", "/camera_1/color/image_raw",
        "/camera_2/color/image_raw", "/camera_3/color/image_raw"
    };
    
    private SSH sshConnection; // Reference to SSH script for tunnel
    private bool isUpdating = false;

    void Start()
    {
        Debug.Log("EnableCamerasViews: Initializing camera view system...");
        
        // Get SSH connection if using tunnel
        if (useSSHTunnel)
        {
            sshConnection = GetComponent<SSH>();
            if (sshConnection == null)
            {
                Debug.LogWarning("SSH script not found! Add SSH component or disable useSSHTunnel.");
            }
        }
        
        // Auto-create camera display grid if enabled
        if (autoCreateCameraGrid && (cameraDisplays == null || cameraDisplays.Length == 0))
        {
            CreateCameraDisplayGrid();
        }
        
        // Start connecting to ROS cameras
        StartCoroutine(InitializeCameraConnections());
    }

    /// <summary>
    /// Creates a grid of RawImage UI elements to display camera feeds
    /// </summary>
    void CreateCameraDisplayGrid()
    {
        Debug.Log("Creating camera display grid...");
        
        // Create canvas if it doesn't exist
        Canvas canvas = FindObjectOfType<Canvas>();
        if (canvas == null)
        {
            GameObject canvasObj = new GameObject("CameraCanvas");
            canvas = canvasObj.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.transform.SetParent(transform);
            canvas.transform.localPosition = new Vector3(0, 1.5f, 2f); // Position in front of player
            canvas.transform.localScale = new Vector3(0.001f, 0.001f, 0.001f);
            
            canvasObj.AddComponent<CanvasScaler>();
            canvasObj.AddComponent<GraphicRaycaster>();
        }
        
        // Calculate total cameras (USB + D435)
        int totalCameras = usbCameraTopics.Length + d435CameraTopics.Length;
        List<RawImage> displays = new List<RawImage>();
        
        // Create grid layout
        int rows = Mathf.CeilToInt((float)totalCameras / gridColumns);
        
        for (int i = 0; i < totalCameras; i++)
        {
            int row = i / gridColumns;
            int col = i % gridColumns;
            
            GameObject camObj = new GameObject($"Camera_{i}_Display");
            camObj.transform.SetParent(canvas.transform);
            
            RawImage rawImage = camObj.AddComponent<RawImage>();
            RectTransform rect = camObj.GetComponent<RectTransform>();
            
            // Position in grid
            float xPos = col * (cameraDisplaySize.x + 10) - (gridColumns * cameraDisplaySize.x) / 2;
            float yPos = -row * (cameraDisplaySize.y + 10) + (rows * cameraDisplaySize.y) / 2;
            
            rect.anchoredPosition = new Vector2(xPos, yPos);
            rect.sizeDelta = cameraDisplaySize;
            
            // Add label
            GameObject labelObj = new GameObject("Label");
            labelObj.transform.SetParent(camObj.transform);
            Text label = labelObj.AddComponent<Text>();
            label.text = $"Camera {i}";
            label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            label.fontSize = 24;
            label.color = Color.green;
            label.alignment = TextAnchor.UpperLeft;
            
            RectTransform labelRect = labelObj.GetComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0, 1);
            labelRect.anchorMax = new Vector2(0, 1);
            labelRect.pivot = new Vector2(0, 1);
            labelRect.anchoredPosition = new Vector2(10, -10);
            labelRect.sizeDelta = new Vector2(300, 50);
            
            displays.Add(rawImage);
        }
        
        cameraDisplays = displays.ToArray();
        Debug.Log($"Created {displays.Count} camera displays in grid layout");
    }

    /// <summary>
    /// Initialize connections to all available ROS camera topics
    /// </summary>
    IEnumerator InitializeCameraConnections()
    {
        UpdateStatus("Connecting to ROS cameras...");
        
        // Wait for SSH connection if using tunnel
        if (useSSHTunnel && sshConnection != null)
        {
            float timeout = 10f;
            float elapsed = 0f;
            
            while (!sshConnection.isConnected && elapsed < timeout)
            {
                yield return new WaitForSeconds(0.5f);
                elapsed += 0.5f;
            }
            
            if (!sshConnection.isConnected)
            {
                Debug.LogError("SSH connection timeout! Cannot connect to ROS cameras.");
                UpdateStatus("SSH Connection Failed");
                yield break;
            }
        }
        
        // Test connection to web server
        string testUrl = $"http://{rosHost}:{rosPort}/";
        using (UnityWebRequest www = UnityWebRequest.Get(testUrl))
        {
            yield return www.SendWebRequest();
            
            if (www.result == UnityWebRequest.Result.ConnectionError || 
                www.result == UnityWebRequest.Result.ProtocolError)
            {
                Debug.LogError($"Cannot connect to ROS camera server: {www.error}");
                UpdateStatus($"Connection Failed: {www.error}");
                yield break;
            }
        }
        
        isConnected = true;
        UpdateStatus("Connected - Streaming cameras");
        Debug.Log("Successfully connected to ROS camera server!");
        
        // Start streaming camera feeds
        StartCoroutine(StreamCameraFeeds());
    }

    /// <summary>
    /// Continuously stream camera feeds from ROS
    /// </summary>
    IEnumerator StreamCameraFeeds()
    {
        float updateInterval = 1f / refreshRate;
        
        while (isConnected)
        {
            if (!isUpdating)
            {
                isUpdating = true;
                StartCoroutine(FetchCameraFrame());
            }
            
            yield return new WaitForSeconds(updateInterval);
        }
    }

    /// <summary>
    /// Fetch a single frame from the ROS camera server
    /// </summary>
    IEnumerator FetchCameraFrame()
    {
        // Use the video feed endpoint from the Flask server
        string videoFeedUrl = $"http://{rosHost}:{rosPort}/video_feed";
        
        using (UnityWebRequest www = UnityWebRequestTexture.GetTexture(videoFeedUrl))
        {
            yield return www.SendWebRequest();
            
            if (www.result == UnityWebRequest.Result.Success)
            {
                // Get the texture from the response
                Texture2D texture = DownloadHandlerTexture.GetContent(www);
                
                // Display on the first available camera display
                if (cameraDisplays != null && cameraDisplays.Length > 0 && cameraDisplays[0] != null)
                {
                    cameraDisplays[0].texture = texture;
                }
            }
            else
            {
                Debug.LogWarning($"Failed to fetch camera frame: {www.error}");
            }
        }
        
        isUpdating = false;
    }

    /// <summary>
    /// Update status text display
    /// </summary>
    void UpdateStatus(string message)
    {
        Debug.Log($"Camera Status: {message}");
        
        if (statusText != null)
        {
            statusText.text = message;
        }
    }

    /// <summary>
    /// Public method to manually refresh camera connections
    /// </summary>
    public void RefreshCameras()
    {
        StopAllCoroutines();
        isConnected = false;
        isUpdating = false;
        
        StartCoroutine(InitializeCameraConnections());
    }

    void OnDestroy()
    {
        isConnected = false;
        StopAllCoroutines();
        
        // Clean up textures
        foreach (var texture in cameraTextures.Values)
        {
            if (texture != null)
            {
                Destroy(texture);
            }
        }
        cameraTextures.Clear();
    }

    void OnApplicationQuit()
    {
        isConnected = false;
    }
}
