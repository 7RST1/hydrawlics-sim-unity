using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Manages the hydraulic pump system and flow distribution among multiple joints.
/// Simulates realistic pressure/flow sharing when multiple valves are open simultaneously.
/// </summary>
public class HydraulicPumpSystem : MonoBehaviour
{
    [Header("Pump Configuration")]
    [Tooltip("Maximum pump pressure (bar)")]
    public float maxPressure = 9f;

    [Tooltip("Total pump flow rate (liters per minute)")]
    public float pumpFlowRate = 12f; // 12 L/min is typical for small hydraulic pumps

    [Tooltip("Minimum flow percentage per joint (prevents complete starvation)")]
    [Range(0f, 0.5f)]
    public float minFlowPercentage = 0.15f;

    [Header("Debug")]
    [Tooltip("Show pump system debug info")]
    public bool showDebugGUI = true;

    // Track active joints requesting flow
    private Dictionary<HydraulicJoint, float> _activeJoints = new Dictionary<HydraulicJoint, float>();
    private float _totalFlowDemand = 0f;

    /// <summary>
    /// Registers a joint as requesting flow with the specified demand (0-1 normalized).
    /// Should be called each frame by joints that have open valves.
    /// </summary>
    /// <param name="joint">The hydraulic joint requesting flow.</param>
    /// <param name="demandNormalized">Flow demand from 0 (no flow) to 1 (max flow).</param>
    public void RequestFlow(HydraulicJoint joint, float demandNormalized)
    {
        _activeJoints[joint] = Mathf.Abs(demandNormalized);
    }

    /// <summary>
    /// Calculates the flow multiplier for a specific joint based on current system load.
    /// Returns a value from minFlowPercentage to 1.0 representing available flow capacity.
    /// </summary>
    /// <param name="joint">The hydraulic joint to calculate flow for.</param>
    /// <returns>Flow multiplier (0-1) to apply to the joint's max speed.</returns>
    public float GetFlowMultiplier(HydraulicJoint joint)
    {
        if (!_activeJoints.ContainsKey(joint) || _activeJoints[joint] < 0.01f)
            return 1f; // No demand, full capacity available

        int activeCount = 0;
        foreach (var demand in _activeJoints.Values)
        {
            if (demand > 0.01f) activeCount++;
        }

        if (activeCount == 0) return 1f;
        if (activeCount == 1) return 1f; // Only this joint active, full flow

        // Calculate flow distribution
        // When multiple joints are active, they share the pump's capacity
        // Use a simple equal distribution with minimum flow guarantee

        float equalShare = 1f / activeCount;
        float distributedFlow = Mathf.Max(equalShare, minFlowPercentage);

        // Normalize so total doesn't exceed 1.0
        float totalDistributed = distributedFlow * activeCount;
        if (totalDistributed > 1f)
        {
            distributedFlow = distributedFlow / totalDistributed;
        }

        return distributedFlow;
    }

    /// <summary>
    /// Called at the end of each frame to reset flow tracking for the next frame.
    /// Unity-specific simulation code.
    /// </summary>
    void LateUpdate()
    {
        _totalFlowDemand = 0f;
        foreach (var demand in _activeJoints.Values)
        {
            _totalFlowDemand += demand;
        }

        // Clear active joints list for next frame
        // Joints must re-register each frame if they need flow
        _activeJoints.Clear();
    }

    /// <summary>
    /// Renders debug GUI showing pump status and active joint count.
    /// </summary>
    void OnGUI()
    {
        if (!showDebugGUI) return;

        GUILayout.BeginArea(new Rect(Screen.width - 320, 10, 310, 200));

        // Semi-transparent background
        GUI.color = new Color(0, 0, 0, 0.7f);
        GUI.Box(new Rect(0, 0, 310, 200), "");
        GUI.color = Color.white;

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 11;
        style.normal.textColor = Color.white;
        style.padding = new RectOffset(10, 10, 10, 10);

        int activeCount = 0;
        foreach (var demand in _activeJoints.Values)
        {
            if (demand > 0.01f) activeCount++;
        }

        float flowPerJoint = activeCount > 0 ? 1f / activeCount : 1f;
        float actualPressure = maxPressure * (activeCount > 0 ? Mathf.Min(1f, 1f / Mathf.Sqrt(activeCount)) : 1f);

        string debugText = "<b>Hydraulic Pump System</b>\n" +
                          "━━━━━━━━━━━━━━━━━━━━━━\n" +
                          $"<b>Pressure:</b> {actualPressure:F1} bar (max: {maxPressure:F1})\n" +
                          $"<b>Flow Rate:</b> {pumpFlowRate:F1} L/min\n" +
                          $"<b>Active Joints:</b> {activeCount}\n" +
                          $"<b>Flow per Joint:</b> {flowPerJoint * 100:F0}%\n" +
                          $"<b>Total Demand:</b> {_totalFlowDemand:F2}\n" +
                          "\n<color=#ffff00>When multiple joints move,\nflow is distributed equally.</color>";

        GUILayout.Label(debugText, style);

        GUILayout.EndArea();
    }
}