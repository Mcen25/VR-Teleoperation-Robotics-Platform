using UnityEngine;
using System;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Text;

public class SSH : MonoBehaviour
{
    [Header("SSH Connection Settings")]
    [SerializeField] private string host = "100.114.29.13";
    [SerializeField] private int port = 22;
    [SerializeField] private string username = "rm";
    [SerializeField] private string keyPath = ""; // Optional: Path to SSH key file (leave empty for password)
    
    [Header("Connection Status")]
    public bool isConnected = false;
    
    private string sshCommand = "ssh"; // or "C:\\Windows\\System32\\OpenSSH\\ssh.exe"

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        TestSSHConnection();
    }

    private async void TestSSHConnection()
    {
        try
        {
            UnityEngine.Debug.Log($"Testing SSH connection to {username}@{host}:{port}...");
            
            // Test connection with a simple command
            string result = await ExecuteSSHCommandAsync("echo 'SSH Connection Successful'");
            
            if (!string.IsNullOrEmpty(result))
            {
                isConnected = true;
                UnityEngine.Debug.Log("SSH connection established successfully!");
                UnityEngine.Debug.Log($"Test result: {result}");
            }
            else
            {
                isConnected = false;
                UnityEngine.Debug.LogWarning("SSH connection test returned empty result.");
            }
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"SSH connection failed: {ex.Message}");
            isConnected = false;
        }
    }

    public async Task<string> ExecuteSSHCommandAsync(string command)
    {
        try
        {
            string result = await Task.Run(() => ExecuteSSHCommand(command));
            return result;
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"SSH command execution failed: {ex.Message}");
            return null;
        }
    }

    private string ExecuteSSHCommand(string command)
    {
        try
        {
            ProcessStartInfo psi = new ProcessStartInfo
            {
                FileName = sshCommand,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true
            };

            // Build SSH arguments
            StringBuilder args = new StringBuilder();
            
            // Add port if not default
            if (port != 22)
            {
                args.Append($"-p {port} ");
            }

            // Add key file if specified
            if (!string.IsNullOrEmpty(keyPath))
            {
                args.Append($"-i \"{keyPath}\" ");
            }

            // Add connection options
            args.Append("-o StrictHostKeyChecking=no ");
            args.Append("-o UserKnownHostsFile=/dev/null ");
            args.Append("-o ConnectTimeout=10 ");

            // Add user@host and command
            args.Append($"{username}@{host} \"{command}\"");

            psi.Arguments = args.ToString();

            UnityEngine.Debug.Log($"Executing SSH: {sshCommand} {psi.Arguments}");

            using (Process process = Process.Start(psi))
            {
                string output = process.StandardOutput.ReadToEnd();
                string error = process.StandardError.ReadToEnd();
                process.WaitForExit();

                if (!string.IsNullOrEmpty(error))
                {
                    UnityEngine.Debug.LogWarning($"SSH stderr: {error}");
                }

                return output;
            }
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"Failed to execute SSH command: {ex.Message}");
            return null;
        }
    }

    // Public method to execute commands from other scripts
    public async void ExecuteCommand(string command)
    {
        if (!isConnected)
        {
            UnityEngine.Debug.LogWarning("SSH client is not connected!");
            return;
        }

        string result = await ExecuteSSHCommandAsync(command);
        if (!string.IsNullOrEmpty(result))
        {
            UnityEngine.Debug.Log($"Command result: {result}");
        }
    }

    private void OnDestroy()
    {
        UnityEngine.Debug.Log("SSH script destroyed.");
    }

    private void OnApplicationQuit()
    {
        UnityEngine.Debug.Log("Application quit - SSH connections will be closed.");
    }
}
