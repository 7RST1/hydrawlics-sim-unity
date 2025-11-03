using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

/// <summary>
/// Represents a single G-Code command with its parameters
/// </summary>
[System.Serializable]
public class GCodeCommand
{
    public string commandType; // G00, G01, G02, G03, M03, M05, etc.
    public float? x;
    public float? y;
    public float? z;
    public float? feedRate; // F parameter

    public GCodeCommand(string type)
    {
        commandType = type;
    }
}

public class HydraulicArmController : MonoBehaviour
{
    [Header("Joints")]
    public HydraulicJoint[] joints;

    [Header("Pump System")]
    public HydraulicPumpSystem pumpSystem;

    [Header("Base")]
    public GameObject baseObject;

    [Header("Control")]
    public bool useKeyboardControl = true;
    public float angleAdjustSpeed = 20f;

    [Header("Testing")]
    public Vector3 testPosition = new Vector3(0.3f, 0.2f, 0.3f);
    public KeyCode testGoToKey = KeyCode.T;

    [Header("G-Code")]
    public KeyCode executeGCodeKey = KeyCode.G;
    public KeyCode goToOrigoDrawingSpace = KeyCode.O;
    public float gCodeMoveDelay = 0.5f; // Delay between G-Code movements in seconds

    private int selectedJoint = 0;
    private bool isExecutingGCode = false;
    private Vector3 currentPosition = Vector3.zero; // Current position in drawing space
    private bool absoluteMode = true; // G90 = absolute, G91 = relative

    /// <summary>
    /// Unity's Start method called once when the script instance is being loaded.
    /// Initializes the pump system reference for all joints.
    /// </summary>
    void Start()
    {
        // Wire up pump system to all joints
        if (pumpSystem != null)
        {
            foreach (var joint in joints)
            {
                if (joint != null)
                {
                    joint.pumpSystem = pumpSystem;
                }
            }
            Debug.Log($"Hydraulic pump system initialized with {joints.Length} joints");
        }
        else
        {
            Debug.LogWarning("No HydraulicPumpSystem assigned! Joints will operate independently without flow sharing.");
        }
    }

    /// <summary>
    /// Unity's Update method called once per frame. Equivalent to the Arduino's loop() function.
    /// Handles keyboard control and would handle serial communication in hardware implementation.
    /// </summary>
    void Update()
    {
        // Unity exclusive functionality! Control the joints with the keyboard
        KeyboardControl();
        
        //// here, in Arduino, there would be a
        // for (int i = 0; i < joints.Length; i++)
        //     joints[i].update();
        
        //// there would also be a
        // checkSerialBuffer();
        //// and
        // Vector3 position = getNextGCodeCommand();
        // moveTip(position); // equivalent to MoveTip(position)
        //// or something similar
    }

    /// <summary>
    /// Provides the keyboard control functionality while Unity sim is running.
    /// </summary>
    void KeyboardControl()
    {
        if (!useKeyboardControl) return;
        
        // Select joint
        for (int i = 0; i < Mathf.Min(joints.Length, 9); i++)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1 + i))
            {
                selectedJoint = i;
                Debug.Log($"Selected Joint {i + 1}");
            }
        }

        // Adjust target angle
        if (selectedJoint < joints.Length)
        {
            if (Input.GetKey(KeyCode.UpArrow))
            {
                joints[selectedJoint].targetAngle += angleAdjustSpeed * Time.deltaTime;
            }
            if (Input.GetKey(KeyCode.DownArrow))
            {
                joints[selectedJoint].targetAngle -= angleAdjustSpeed * Time.deltaTime;
            }

            // Clamp angle
            joints[selectedJoint].targetAngle = Mathf.Clamp(
                joints[selectedJoint].targetAngle, -170f, 170f);
        }
        
        if (baseObject)
        {
            if (Input.GetKey(KeyCode.LeftArrow))
            {
                baseObject.transform.Rotate(0, -angleAdjustSpeed * Time.deltaTime, 0);
            }
            if (Input.GetKey(KeyCode.RightArrow))
            {
                baseObject.transform.Rotate(0, angleAdjustSpeed * Time.deltaTime, 0);
            }

        }
        
        // Reset all joints
        if (Input.GetKeyDown(KeyCode.R))
        {
            foreach (var joint in joints)
            {
                joint.resetToInit();
            }
        }

        // Test GoTo function
        if (Input.GetKeyDown(testGoToKey))
        {
            Debug.Log($"Testing GoTo with position: {testPosition}");
            MoveTip(testPosition);
        }

        // Execute G-Code
        if (Input.GetKeyDown(goToOrigoDrawingSpace) && !isExecutingGCode)
        {
            MoveInDrawingSpace(Vector3.zero);
        }
        
        // Execute G-Code
        if (Input.GetKeyDown(executeGCodeKey) && !isExecutingGCode)
        {
            string smileyGCode = GetSmileyFaceGCode();
            StartCoroutine(ExecuteGCode(smileyGCode));
        }
    }

    /// <summary>
    /// Moves the arm tip to the specified position in drawing space.
    /// Adds drawing space vector offset and hands over to MoveTip.
    /// </summary>
    /// <param name="gCodeCommands">The target position in drawing space for the arm tip.</param>
    void MoveThroughGCode()
    {
        
    }
    
    /// <summary>
    /// Moves the arm tip to the specified position in drawing space.
    /// Adds drawing space vector offset and hands over to MoveTip.
    /// </summary>
    /// <param name="gCode">The target position in drawing space for the arm tip.</param>
    void MoveInDrawingSpace(Vector3 gCode)
    {
        // see maths in week 11 post
        // assert space limits
        Debug.Assert(gCode.x <= 0.3264f);
        Debug.Assert(gCode.x >= 0);

        Debug.Assert(gCode.y <= 0.1846f);
        Debug.Assert(gCode.y >= 0);

        Vector3 gCodeTranslated = new Vector3(
            gCode.y,
            gCode.z,
            gCode.x * -1
        );

        // Drawing surface offset - Y value sets the height of the drawing surface
        Vector3 drawSpaceOffset = new Vector3(0.3225f, 0.2f, 0.1632f);
        Vector3 targetWorldPos = drawSpaceOffset + gCodeTranslated;

        Debug.Log($"GCode: ({gCode.x:F3}, {gCode.y:F3}, {gCode.z:F3}) -> World: ({targetWorldPos.x:F3}, {targetWorldPos.y:F3}, {targetWorldPos.z:F3})");
        MoveTip(targetWorldPos);

    }
    
    /// <summary>
    /// Moves the arm tip to the specified position using inverse kinematics.
    /// Adjusts for the end effector length and delegates to MoveEndEffectorOrigin.
    /// </summary>
    /// <param name="position">The target position in world space for the arm tip.</param>
    void MoveTip(Vector3 position)
    {
        float e_magnitude = 0.07f;
        // We need to remove the vector that is the end effector from the
        // equation, and forward it to MoveEndEffectorOrigin
        Debug.Log($"MoveTip - Input Position: {position}");
        Vector3 P_vector = position - joints[0].transform.position;
        Debug.Log($"MoveTip - P_vector before zeroing Y: {P_vector}");
        P_vector.y = 0;
        Debug.Log($"MoveTip - P_vector after zeroing Y: {P_vector}");
        Vector3 P_vector_normalized = P_vector.normalized;
        Vector3 P_vector_normalized_scaled = P_vector_normalized * e_magnitude;
        Vector3 endEffectorOriginPos = position - P_vector_normalized_scaled;
        Debug.Log($"MoveTip - Sending to MoveEndEffectorOrigin: {endEffectorOriginPos}");
        MoveEndEffectorOrigin(endEffectorOriginPos);

    }

    /// <summary>
    /// Performs inverse kinematics calculations to position the end effector origin at the specified position.
    /// Calculates joint angles theta_1, theta_2, and theta_3 using geometric methods and applies them to the arm.
    /// </summary>
    /// <param name="position">The target position in world space for the end effector origin.</param>
    void MoveEndEffectorOrigin(Vector3 position)
    {
        //renaming axis. Unity uses y up instead of z up. y and z are swapped.
        float x_3 = position.x;
        float y_3 = position.z;
        float z_3 = position.y;
        
        // Horizontal plane:
        // arm projected onto x-y plane, x+ towards you, y+ towards the right

        Debug.Log($"x:{x_3}, y:{y_3}");
        double theta_1_rad = Math.PI + Math.Atan2(x_3, y_3);
        // Negated to account for Unity's rotation direction
        
        float a_1 = 0.098f;
        float a_2 = 0.270f;
        float a_3 = 0.320f;
        
        double r = Math.Sqrt(x_3*x_3 + (z_3 - a_1)*(z_3 - a_1));
        
        double phi_1_rad = Math.Acos((a_2*a_2 + r*r - a_3*a_3)/(2*a_2*r));
        double phi_2_rad = Math.Acos((a_2*a_2 + a_3*a_3 - r*r)/(2*a_2*a_3));
        double phi_3_rad = Math.Atan((z_3-a_1)/x_3);
        
        double theta_2_rad = phi_3_rad + phi_1_rad;
        double theta_3_rad = phi_2_rad - Math.PI;
        
        
        float phi_1 = (180f/(float)Math.PI * (float)phi_1_rad);
        float phi_2 = (180f/(float)Math.PI * (float)phi_2_rad);
        float phi_3 = (180f/(float)Math.PI * (float)phi_3_rad);
        
        float theta_1 = (180f/(float)Math.PI * (float)theta_1_rad);
        float theta_2 = (180f/(float)Math.PI * (float)theta_2_rad);
        float theta_3 = (180f/(float)Math.PI * (float)theta_3_rad);

        baseObject.transform.localRotation = Quaternion.Euler(0f, theta_1 + 90, 0f);
        joints[0].targetAngle = theta_2 - 90f;
        joints[1].targetAngle = theta_3;
        
        // Calculate the angle of the end effector joint
        joints[2].targetAngle =
            (
                180
                - phi_1
                - phi_2
            ) - phi_3;
        
        Debug.Log($"Phi_1: {phi_1}");
        Debug.Log($"Phi_2: {phi_2}");
        Debug.Log($"Phi_3: {phi_3}");
        
        Debug.Log($"Theta_1: {theta_1}");
        Debug.Log($"Theta_2: {theta_2}");
        Debug.Log($"Theta_3: {theta_3}");
        
        
        // t = 180-i
    }

    // ==================== G-Code Functions ====================

    /// <summary>
    /// Returns a hardcoded smiley face G-Code pattern for testing
    /// </summary>
    /// <returns>G-Code string representing a smiley face</returns>
    string GetSmileyFaceGCode()
    {
        return @"
G90 ; Absolute positioning mode
G00 X0.16 Y0.09 Z0.01 ; Move to start position (pen up)

; Draw left eye (circle)
G00 X0.10 Y0.12 Z0.00 ; Move to left eye position
G01 X0.09 Y0.13 Z0.00 ; Draw left eye
G01 X0.09 Y0.11 Z0.00
G01 X0.11 Y0.11 Z0.00
G01 X0.11 Y0.13 Z0.00
G01 X0.09 Y0.13 Z0.00

; Pen up and move to right eye
G00 X0.21 Y0.12 Z0.01

; Draw right eye (circle)
G01 X0.20 Y0.13 Z0.00
G01 X0.20 Y0.11 Z0.00
G01 X0.22 Y0.11 Z0.00
G01 X0.22 Y0.13 Z0.00
G01 X0.20 Y0.13 Z0.00

; Pen up and move to mouth start
G00 X0.08 Y0.05 Z0.01

; Draw smile (arc approximated with line segments)
G01 X0.08 Y0.05 Z0.00 ; Start of smile
G01 X0.10 Y0.04 Z0.00
G01 X0.13 Y0.03 Z0.00
G01 X0.16 Y0.03 Z0.00
G01 X0.19 Y0.03 Z0.00
G01 X0.22 Y0.04 Z0.00
G01 X0.24 Y0.05 Z0.00

; Pen up and return to home
G00 X0.16 Y0.09 Z0.01
";
    }

    /// <summary>
    /// Parses a G-Code string into a list of GCodeCommand objects
    /// </summary>
    /// <param name="gCodeString">The raw G-Code string to parse</param>
    /// <returns>List of parsed GCodeCommand objects</returns>
    List<GCodeCommand> ParseGCode(string gCodeString)
    {
        List<GCodeCommand> commands = new List<GCodeCommand>();
        string[] lines = gCodeString.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);

        foreach (string line in lines)
        {
            // Remove comments (everything after semicolon)
            string cleanLine = line.Split(';')[0].Trim();
            if (string.IsNullOrEmpty(cleanLine)) continue;

            // Split line into tokens
            string[] tokens = cleanLine.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (tokens.Length == 0) continue;

            string commandType = tokens[0].ToUpper();

            // Skip non-movement commands except G90/G91
            if (commandType == "G90")
            {
                absoluteMode = true;
                Debug.Log("Switched to absolute positioning mode");
                continue;
            }
            else if (commandType == "G91")
            {
                absoluteMode = false;
                Debug.Log("Switched to relative positioning mode");
                continue;
            }

            // Only process movement commands (G00, G01, G02, G03)
            if (!commandType.StartsWith("G0") && !commandType.StartsWith("G1"))
                continue;

            GCodeCommand cmd = new GCodeCommand(commandType);

            // Parse parameters
            for (int i = 1; i < tokens.Length; i++)
            {
                string token = tokens[i].ToUpper();
                if (token.Length < 2) continue;

                char param = token[0];
                string valueStr = token.Substring(1);
                float value;

                // Use InvariantCulture to parse numbers with period as decimal separator
                if (float.TryParse(valueStr, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
                {
                    switch (param)
                    {
                        case 'X':
                            cmd.x = value;
                            break;
                        case 'Y':
                            cmd.y = value;
                            break;
                        case 'Z':
                            cmd.z = value;
                            break;
                        case 'F':
                            cmd.feedRate = value;
                            break;
                    }
                }
                else
                {
                    Debug.LogWarning($"Failed to parse GCode parameter: {param}{valueStr}");
                }
            }

            commands.Add(cmd);
        }

        Debug.Log($"Parsed {commands.Count} G-Code commands");
        return commands;
    }

    /// <summary>
    /// Executes a G-Code string by parsing it and moving through each command sequentially
    /// </summary>
    /// <param name="gCodeString">The G-Code string to execute</param>
    /// <returns>IEnumerator for coroutine execution</returns>
    IEnumerator ExecuteGCode(string gCodeString)
    {
        isExecutingGCode = true;
        Debug.Log("Starting G-Code execution");

        List<GCodeCommand> commands = ParseGCode(gCodeString);

        foreach (GCodeCommand cmd in commands)
        {
            // Calculate target position
            Vector3 targetPos = currentPosition;

            if (absoluteMode)
            {
                // Absolute positioning
                if (cmd.x.HasValue) targetPos.x = cmd.x.Value;
                if (cmd.y.HasValue) targetPos.y = cmd.y.Value;
                if (cmd.z.HasValue) targetPos.z = cmd.z.Value;
            }
            else
            {
                // Relative positioning
                if (cmd.x.HasValue) targetPos.x += cmd.x.Value;
                if (cmd.y.HasValue) targetPos.y += cmd.y.Value;
                if (cmd.z.HasValue) targetPos.z += cmd.z.Value;
            }

            currentPosition = targetPos;

            // Execute movement
            Debug.Log($"Executing {cmd.commandType} -> X:{targetPos.x:F3} Y:{targetPos.y:F3} Z:{targetPos.z:F3}");

            // Only move if Z is at drawing height (Z <= 0.005, considering "pen down")
            // For now, we'll move to all positions to visualize the path
            MoveInDrawingSpace(targetPos);

            // Wait for movement to complete
            yield return new WaitForSeconds(gCodeMoveDelay);
        }

        Debug.Log("G-Code execution complete");
        isExecutingGCode = false;
    }

    /// <summary>
    /// Renders the GUI overlay showing controller status and instructions.
    /// Displays selected joint, control keys, and current/target angles for all joints.
    /// </summary>
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 500));
        GUILayout.Label("Hydraulic Arm Controller", GUI.skin.box);
        GUILayout.Label($"Selected Joint: {selectedJoint + 1}");
        GUILayout.Label("Keys 1-9: Select joint");
        GUILayout.Label("Up/Down: Adjust angle");
        GUILayout.Label("Left/Right: Rotate base");
        GUILayout.Label("R: Reset all");
        GUILayout.Label($"T: GoTo test position");
        GUILayout.Label($"G: Execute G-Code (Smiley Face)");

        if (isExecutingGCode)
        {
            GUILayout.Label(">>> EXECUTING G-CODE <<<", GUI.skin.box);
        }

        GUILayout.Space(10);

        for (int i = 0; i < joints.Length; i++)
        {
            string selected = i == selectedJoint ? " <--" : "";
            float currentAngle = joints[i].transform.localEulerAngles.z;
            if (currentAngle > 180f) currentAngle -= 360f;
            GUILayout.Label($"Joint {i + 1}: Target={joints[i].targetAngle:F1}° " +
                          $"Current={currentAngle:F1}°{selected}");
        }

        GUILayout.EndArea();
    }

    /// <summary>
    /// Draws debug gizmos in the Scene view showing the test target position and coordinate axes.
    /// </summary>
    void OnDrawGizmos()
    {
        // Draw the test target position
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(testPosition, 0.05f);

        // Draw coordinate axes at the target position for reference
        Gizmos.color = Color.red;
        Gizmos.DrawLine(testPosition, testPosition + Vector3.right * 0.03f); // X axis
        Gizmos.color = Color.green;
        Gizmos.DrawLine(testPosition, testPosition + Vector3.up * 0.03f); // Y axis
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(testPosition, testPosition + Vector3.forward * 0.03f); // Z axis
    }
}