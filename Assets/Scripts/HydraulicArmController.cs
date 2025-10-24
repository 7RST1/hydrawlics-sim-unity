using System;
using UnityEngine;

public class HydraulicArmController : MonoBehaviour
{
    [Header("Joints")]
    public HydraulicJoint[] joints;

    [Header("Base")]
    public GameObject baseObject;
    
    [Header("Control")]
    public bool useKeyboardControl = true;
    public float angleAdjustSpeed = 20f;

    [Header("Testing")]
    public Vector3 testPosition = new Vector3(0.3f, 0.2f, 0.3f);
    public KeyCode testGoToKey = KeyCode.T;

    private int selectedJoint = 0;
    
    void Update()
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
    }

    void MoveTip(Vector3 position)
    {
        float e_magnitude = 0.07f;
        // We need to remove the vector that is the end effector from the
        // equation, and forward it to MoveEndEffectorOrigin
        Debug.Log($"Position: {position}");
        Vector3 P_vector = position - joints[0].transform.position;
        P_vector.y = 0;
        Debug.Log($"P_vector: {P_vector}");
        Vector3 P_vector_normalized = P_vector.normalized;
        Vector3 P_vector_normalized_scaled = P_vector_normalized * e_magnitude;
        MoveEndEffectorOrigin(position - P_vector_normalized_scaled);
        
    }
    
    void MoveEndEffectorOrigin(Vector3 position)
    {
        //renaming axis. Unity uses y up instead of z up. y and z are swapped.
        float x_3 = position.x;
        float y_3 = position.z;
        float z_3 = position.y;
        
        // Horizontal plane:
        // arm projected onto x-y plane, x+ towards you, y+ towards the right

        double theta_1_rad = Math.Atan2(x_3, y_3);
        // add pi if not within the wanted range
        
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
        
        baseObject.transform.localRotation = Quaternion.Euler(0f, (float)theta_1_rad, 0f);
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