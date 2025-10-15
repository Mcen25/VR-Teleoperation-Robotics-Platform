using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using System;
using RosImage = RosSharp.RosBridgeClient.MessageTypes.Sensor.Image;

/// <summary>
/// Subscribes to ROS camera topics using ROS# (RosSharp)
/// Displays camera feed on a RawImage UI element
/// </summary>
[RequireComponent(typeof(RosConnector))]
public class ROSCameraSubscriber : MonoBehaviour
{
    [Header("ROS Topic Settings")]
    [SerializeField] private string cameraTopic = "/camera0/image_raw";
    [SerializeField] private string cameraName = "USB_Cam_0";
    
    [Header("Display Settings")]
    [SerializeField] private RawImage displayImage;
    [SerializeField] private Text labelText;
    
    [Header("Status")]
    public bool isSubscribed = false;
    public int framesReceived = 0;
    public float currentFPS = 0f;
    
    private RosConnector rosConnector;
    private string subscriptionId;
    private Texture2D cameraTexture;
    
    private float lastFrameTime;
    private int framesSinceLastUpdate = 0;
    private float fpsUpdateInterval = 1f;

    void Start()
    {
        // Get ROS connector
        rosConnector = GetComponent<RosConnector>();
        
        if (rosConnector == null)
        {
            Debug.LogError($"ROSCameraSubscriber: RosConnector component not found! Add RosConnector to {gameObject.name}");
            return;
        }
        
        // Wait a bit for ROS connection to establish
        Invoke(nameof(SubscribeToCamera), 2f);
        
        // Setup display
        InitializeDisplay();
    }

    void InitializeDisplay()
    {
        // Create texture for camera feed (default size, will resize when first frame arrives)
        cameraTexture = new Texture2D(640, 480, TextureFormat.RGB24, false);
        
        if (displayImage != null)
        {
            displayImage.texture = cameraTexture;
        }
        
        if (labelText != null)
        {
            labelText.text = $"{cameraName}\nConnecting...";
        }
        
        Debug.Log($"ROSCameraSubscriber: Initialized display for {cameraName} on topic {cameraTopic}");
    }

    void SubscribeToCamera()
    {
        if (rosConnector.RosSocket == null || rosConnector.IsConnected == null || !rosConnector.IsConnected.WaitOne(0))
        {
            Debug.LogWarning($"ROSCameraSubscriber: ROS not connected yet. Retrying in 2 seconds...");
            Invoke(nameof(SubscribeToCamera), 2f);
            return;
        }
        
        // Subscribe to camera topic
        subscriptionId = rosConnector.RosSocket.Subscribe<CompressedImage>(
            cameraTopic + "/compressed",  // Use compressed images for better performance
            ReceiveCompressedImage,
            (int)(1000 / 30)  // 30 Hz throttle rate
        );
        
        // If compressed not available, try raw
        if (string.IsNullOrEmpty(subscriptionId))
        {
            subscriptionId = rosConnector.RosSocket.Subscribe<RosImage>(
                cameraTopic,
                ReceiveRawImage,
                (int)(1000 / 30)
            );
        }
        
        isSubscribed = !string.IsNullOrEmpty(subscriptionId);
        
        if (isSubscribed)
        {
            Debug.Log($"✓ Subscribed to ROS topic: {cameraTopic}");
            if (labelText != null)
            {
                labelText.text = $"{cameraName}\nSubscribed";
            }
        }
        else
        {
            Debug.LogError($"✗ Failed to subscribe to {cameraTopic}");
            if (labelText != null)
            {
                labelText.text = $"{cameraName}\nFailed";
                labelText.color = Color.red;
            }
        }
    }

    void ReceiveCompressedImage(CompressedImage message)
    {
        try
        {
            // Decompress JPEG/PNG data
            byte[] imageData = message.data;
            
            // Load texture from compressed image data
            if (cameraTexture == null)
            {
                cameraTexture = new Texture2D(2, 2);
            }
            
            cameraTexture.LoadImage(imageData);
            
            if (displayImage != null)
            {
                displayImage.texture = cameraTexture;
            }
            
            framesReceived++;
            framesSinceLastUpdate++;
            UpdateFPS();
        }
        catch (Exception e)
        {
            Debug.LogError($"Error processing compressed image: {e.Message}");
        }
    }

    void ReceiveRawImage(RosImage message)
    {
        try
        {
            // Process raw image data
            int width = (int)message.width;
            int height = (int)message.height;
            string encoding = message.encoding;
            
            // Resize texture if needed
            if (cameraTexture == null || cameraTexture.width != width || cameraTexture.height != height)
            {
                cameraTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
            }
            
            // Convert ROS image data to Unity texture
            // Note: ROS images are typically BGR8, Unity uses RGB24
            byte[] imageData = message.data;
            
            if (encoding == "bgr8" || encoding == "rgb8")
            {
                Color32[] pixels = new Color32[width * height];
                
                for (int i = 0; i < width * height; i++)
                {
                    int idx = i * 3;
                    if (encoding == "bgr8")
                    {
                        // BGR to RGB
                        pixels[i] = new Color32(imageData[idx + 2], imageData[idx + 1], imageData[idx], 255);
                    }
                    else
                    {
                        // RGB
                        pixels[i] = new Color32(imageData[idx], imageData[idx + 1], imageData[idx + 2], 255);
                    }
                }
                
                cameraTexture.SetPixels32(pixels);
                cameraTexture.Apply();
                
                if (displayImage != null)
                {
                    displayImage.texture = cameraTexture;
                }
            }
            
            framesReceived++;
            framesSinceLastUpdate++;
            UpdateFPS();
        }
        catch (Exception e)
        {
            Debug.LogError($"Error processing raw image: {e.Message}");
        }
    }

    void UpdateFPS()
    {
        float currentTime = Time.time;
        float deltaTime = currentTime - lastFrameTime;
        
        if (deltaTime >= fpsUpdateInterval)
        {
            currentFPS = framesSinceLastUpdate / deltaTime;
            framesSinceLastUpdate = 0;
            lastFrameTime = currentTime;
            
            if (labelText != null)
            {
                labelText.text = $"{cameraName}\n{currentFPS:F1} FPS\n{framesReceived} frames";
                labelText.color = Color.green;
            }
        }
    }

    void OnDestroy()
    {
        // Unsubscribe from topic
        if (rosConnector != null && rosConnector.RosSocket != null && !string.IsNullOrEmpty(subscriptionId))
        {
            rosConnector.RosSocket.Unsubscribe(subscriptionId);
        }
        
        if (cameraTexture != null)
        {
            Destroy(cameraTexture);
        }
    }

    // Public API
    public void SetCameraTopic(string topic)
    {
        cameraTopic = topic;
    }

    public void SetCameraName(string name)
    {
        cameraName = name;
        if (labelText != null)
        {
            labelText.text = name;
        }
    }
}
