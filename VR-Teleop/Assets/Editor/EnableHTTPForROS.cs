#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

/// <summary>
/// Automatically enables HTTP connections for ROS camera streaming
/// This is required in Unity 6+ which blocks insecure HTTP by default
/// </summary>
[InitializeOnLoad]
public static class EnableHTTPForROS
{
    static EnableHTTPForROS()
    {
        // Enable insecure HTTP for development
        // Required for ROS camera streaming over HTTP
        PlayerSettings.insecureHttpOption = InsecureHttpOption.AlwaysAllowed;
        
        Debug.Log("[ROS Camera Setup] HTTP connections enabled for camera streaming");
        Debug.Log("[ROS Camera Setup] This allows HTTP requests to your robot at http://100.114.29.13:8080");
        Debug.Log("[ROS Camera Setup] For production, consider using HTTPS instead");
    }
}
#endif
