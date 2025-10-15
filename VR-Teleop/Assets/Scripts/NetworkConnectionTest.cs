using UnityEngine;
using UnityEngine.Networking;
using System.Collections;

/// <summary>
/// Quick test script to verify HTTP connections and network connectivity
/// Attach to any GameObject and check Console for results
/// </summary>
public class NetworkConnectionTest : MonoBehaviour
{
    [Header("Test Settings")]
    [SerializeField] private string robotIP = "100.114.29.13";
    [SerializeField] private int robotPort = 8080;
    [SerializeField] private bool runOnStart = true;

    void Start()
    {
        if (runOnStart)
        {
            RunAllTests();
        }
    }

    [ContextMenu("Run All Tests")]
    public void RunAllTests()
    {
        StartCoroutine(RunTestSequence());
    }

    IEnumerator RunTestSequence()
    {
        Debug.Log("========================================");
        Debug.Log("NETWORK CONNECTION TEST - Starting");
        Debug.Log("========================================");
        
        // Test 1: HTTP Support
        yield return StartCoroutine(TestHTTPSupport());
        yield return new WaitForSeconds(1f);
        
        // Test 2: Robot Connectivity
        yield return StartCoroutine(TestRobotConnection());
        yield return new WaitForSeconds(1f);
        
        // Test 3: Camera Server
        yield return StartCoroutine(TestCameraServer());
        
        Debug.Log("========================================");
        Debug.Log("NETWORK CONNECTION TEST - Complete");
        Debug.Log("========================================");
    }

    IEnumerator TestHTTPSupport()
    {
        Debug.Log("[TEST 1/3] Testing HTTP support...");
        
        // Test with public HTTP endpoint
        using (UnityWebRequest www = UnityWebRequest.Get("http://httpbin.org/get"))
        {
            www.timeout = 5;
            yield return www.SendWebRequest();
            
            if (www.result == UnityWebRequest.Result.Success)
            {
                Debug.Log("✅ <color=green>HTTP IS ENABLED!</color> Camera streaming should work.");
            }
            else
            {
                Debug.LogError("❌ <color=red>HTTP IS BLOCKED!</color>");
                Debug.LogError($"Error: {www.error}");
                Debug.LogError("SOLUTION: The Editor script 'EnableHTTPForROS.cs' should fix this automatically.");
                Debug.LogError("Or manually: Edit > Project Settings > Player > Other Settings > Insecure HTTP Option = Always Allowed");
            }
        }
    }

    IEnumerator TestRobotConnection()
    {
        Debug.Log($"[TEST 2/3] Testing robot connectivity to {robotIP}...");
        
        // Try to reach robot's camera server
        string url = $"http://{robotIP}:{robotPort}/";
        
        using (UnityWebRequest www = UnityWebRequest.Get(url))
        {
            www.timeout = 10;
            yield return www.SendWebRequest();
            
            if (www.result == UnityWebRequest.Result.Success)
            {
                Debug.Log($"✅ <color=green>ROBOT IS REACHABLE!</color> Connected to {robotIP}:{robotPort}");
            }
            else if (www.result == UnityWebRequest.Result.ConnectionError)
            {
                Debug.LogWarning($"⚠️ <color=yellow>CANNOT REACH ROBOT</color> at {robotIP}:{robotPort}");
                Debug.LogWarning($"Error: {www.error}");
                Debug.LogWarning("TROUBLESHOOTING:");
                Debug.LogWarning("1. Check robot is powered on");
                Debug.LogWarning("2. Verify IP address is correct");
                Debug.LogWarning("3. Test in PowerShell: ping " + robotIP);
                Debug.LogWarning("4. If using Tailscale VPN, ensure it's connected");
                Debug.LogWarning("5. Check Python camera server is running on robot");
            }
            else
            {
                Debug.LogWarning($"⚠️ Robot responded but with error: {www.error}");
            }
        }
    }

    IEnumerator TestCameraServer()
    {
        Debug.Log($"[TEST 3/3] Testing camera stream endpoint...");
        
        string url = $"http://{robotIP}:{robotPort}/video_feed";
        
        using (UnityWebRequest www = UnityWebRequestTexture.GetTexture(url))
        {
            www.timeout = 10;
            yield return www.SendWebRequest();
            
            if (www.result == UnityWebRequest.Result.Success)
            {
                Debug.Log("✅ <color=green>CAMERA STREAM AVAILABLE!</color> Ready to display cameras.");
                Texture2D texture = DownloadHandlerTexture.GetContent(www);
                Debug.Log($"   Received frame: {texture.width}x{texture.height}");
            }
            else
            {
                Debug.LogWarning($"⚠️ <color=yellow>CAMERA STREAM NOT AVAILABLE</color>");
                Debug.LogWarning($"Error: {www.error}");
                Debug.LogWarning("TROUBLESHOOTING:");
                Debug.LogWarning("1. Start ROS camera drivers: roslaunch door_traverse collect.launch");
                Debug.LogWarning("2. Start Python server: python3 live_cameras_web.py");
                Debug.LogWarning("3. Test in browser: http://" + robotIP + ":" + robotPort);
            }
        }
    }

    void OnGUI()
    {
        // Draw button in editor
        if (GUI.Button(new Rect(10, 150, 200, 30), "Run Connection Tests"))
        {
            RunAllTests();
        }
    }
}
