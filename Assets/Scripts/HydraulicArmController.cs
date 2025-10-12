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
}