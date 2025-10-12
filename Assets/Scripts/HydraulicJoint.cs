using UnityEngine;

public class HydraulicJoint : MonoBehaviour
{
    [Header("Piston Limits")]
    [Tooltip("Minimum piston length (meters)")]
    public float minPistonLength = 0.5f;
    
    [Tooltip("Maximum piston length (meters)")]
    public float maxPistonLength = 2.5f;
    
    [Header("Hydraulic Properties")]
    [Tooltip("Maximum extension/retraction speed (m/s)")]
    public float maxPistonSpeed = 0.5f;
    
    [Tooltip("Hydraulic damping factor (higher = faster response)")]
    public float hydraulicDamping = 2.0f;
    
    [Header("PWM Control")]
    [Tooltip("PWM frequency (Hz)")]
    public float pwmFrequency = 50f;
    
    [Header("PID Controller")]
    public float kP = 2.0f;
    public float kI = 0.1f;
    public float kD = 0.5f;
    
    [Header("References")]
    [Tooltip("Piston base attachment point (doesn't rotate with joint)")]
    public Transform pistonBaseVisual;
    
    [Tooltip("Piston end attachment point (rotates with joint)")]
    public Transform pistonEndVisual;
    
    [Tooltip("LineRenderer to visualize piston")]
    public LineRenderer pistonLine;
    
    [Header("Target")]
    public float targetAngle = 45f;

    [Header("Debug")]
    [Tooltip("Show debug GUI overlay")]
    public bool showDebugGUI = true;

    // Geometry - calculated from visual positions
    private float pistonBaseDistance;  // Distance from joint to base attachment (in parent's space)
    private float pistonEndDistance;   // Distance from joint to end attachment (in joint's local space)
    private float pistonBaseAngleInParentSpace;  // Angle of base attachment in parent's space
    private float pistonEndAngle;   // Angle of end attachment relative to joint (local space)
    
    // Internal state
    private float currentAngle;
    private float currentPistonLength;
    private float pistonVelocity;
    
    // PID state
    private float integralError;
    private float previousError;
    
    // PWM state
    private float pwmAccumulator;
    private bool valveExtend;
    private bool valveRetract;
    
    void Start()
    {
        // Calculate triangle geometry from actual visual positions
        CalculateGeometry();

        // Initialize piston length based on target angle
        currentPistonLength = CalculatePistonLength(targetAngle);

        // Initialize velocity and PID
        pistonVelocity = 0f;
        integralError = 0f;
        previousError = 0f;

        // Set initial joint rotation
        UpdateJointFromPiston();
        UpdateVisualization();

        // Auto-find LineRenderer if not assigned
        if (pistonLine == null)
            pistonLine = GetComponent<LineRenderer>();
    }
    
    void CalculateGeometry()
    {
        // For multi-link arms:
        // - pistonBaseVisual is typically on the parent/fixed linkage (doesn't rotate with this joint)
        // - pistonEndVisual is on this joint's rotating linkage (rotates with this joint)

        if (pistonBaseVisual != null && transform.parent != null)
        {
            // Calculate base attachment in PARENT's space (since it doesn't rotate with this joint)
            Vector3 parentSpaceBasePos = transform.parent.InverseTransformPoint(pistonBaseVisual.position);
            Vector3 parentSpaceJointPos = transform.parent.InverseTransformPoint(transform.position);
            Vector3 baseRelativeToJoint = parentSpaceBasePos - parentSpaceJointPos;

            pistonBaseDistance = baseRelativeToJoint.magnitude;
            pistonBaseAngleInParentSpace = Mathf.Atan2(baseRelativeToJoint.y, baseRelativeToJoint.x) * Mathf.Rad2Deg;
        }

        if (pistonEndVisual != null)
        {
            // Calculate end attachment in this joint's LOCAL space (rotates with joint)
            Vector3 localEndPos = transform.InverseTransformPoint(pistonEndVisual.position);
            pistonEndDistance = localEndPos.magnitude;
            pistonEndAngle = Mathf.Atan2(localEndPos.y, localEndPos.x) * Mathf.Rad2Deg;
        }

        Debug.Log($"Joint {gameObject.name}: Base distance = {pistonBaseDistance:F3}, End distance = {pistonEndDistance:F3}");
        Debug.Log($"Joint {gameObject.name}: Base angle (parent space) = {pistonBaseAngleInParentSpace:F1}°, End angle (local) = {pistonEndAngle:F1}°");
    }
    
    void Update()
    {
        // Update current angle from joint rotation
        currentAngle = transform.localEulerAngles.z;
        if (currentAngle > 180f) currentAngle -= 360f;

        // Calculate target piston length for desired angle
        float targetLength = CalculatePistonLength(targetAngle);
        
        // PID control to get desired piston velocity
        float error = targetLength - currentPistonLength;
        integralError += error * Time.deltaTime;
        float derivative = (error - previousError) / Time.deltaTime;
        
        float pidOutput = kP * error + kI * integralError + kD * derivative;
        previousError = error;
        
        // Convert PID output to valve commands via PWM
        UpdatePWM(pidOutput);
        
        // Simulate hydraulic piston movement
        SimulateHydraulics();
        
        // Apply piston length to joint angle
        UpdateJointFromPiston();
        
        // Update visualization
        UpdateVisualization();
    }
    
    float CalculatePistonLength(float jointAngle)
    {
        // Calculate the angle in the triangle at the joint pivot
        // pistonBaseAngleInParentSpace: angle to base attachment (in parent's space, doesn't change with joint rotation)
        // pistonEndAngle: angle to end attachment (in joint's local space)
        // jointAngle: current rotation of the joint (in parent's space)

        // The end attachment's angle in parent space is: pistonEndAngle + jointAngle
        // The triangle angle at the joint is the difference between the two attachment angles
        float triangleAngle = (pistonEndAngle + jointAngle) - pistonBaseAngleInParentSpace;
        float triangleAngleRad = triangleAngle * Mathf.Deg2Rad;

        // Use law of cosines: c² = a² + b² - 2ab*cos(C)
        // where c is the piston length, a and b are the attachment distances
        float length = Mathf.Sqrt(
            pistonBaseDistance * pistonBaseDistance +
            pistonEndDistance * pistonEndDistance -
            2 * pistonBaseDistance * pistonEndDistance * Mathf.Cos(triangleAngleRad)
        );

        return Mathf.Clamp(length, minPistonLength, maxPistonLength);
    }
    
    float CalculateAngleFromPiston(float pistonLength)
    {
        // Use law of cosines to get the triangle angle (always positive from Acos)
        // cos(C) = (a² + b² - c²) / (2ab)
        float cosAngle = (pistonBaseDistance * pistonBaseDistance +
                         pistonEndDistance * pistonEndDistance -
                         pistonLength * pistonLength) /
                        (2 * pistonBaseDistance * pistonEndDistance);

        cosAngle = Mathf.Clamp(cosAngle, -1f, 1f);
        float triangleAngle = Mathf.Acos(cosAngle) * Mathf.Rad2Deg;

        // Convert triangle angle back to joint angle
        // triangleAngle = (pistonEndAngle + jointAngle) - pistonBaseAngleInParentSpace
        // Solving for jointAngle: jointAngle = triangleAngle + pistonBaseAngleInParentSpace - pistonEndAngle
        float jointAnglePositive = triangleAngle + pistonBaseAngleInParentSpace - pistonEndAngle;
        float jointAngleNegative = -triangleAngle + pistonBaseAngleInParentSpace - pistonEndAngle;

        // Choose the solution closer to the target angle to avoid flipping
        float diffPositive = Mathf.Abs(Mathf.DeltaAngle(jointAnglePositive, targetAngle));
        float diffNegative = Mathf.Abs(Mathf.DeltaAngle(jointAngleNegative, targetAngle));

        return (diffPositive < diffNegative) ? jointAnglePositive : jointAngleNegative;
    }
    
    void UpdatePWM(float pidOutput)
    {
        // Normalize PID output to -1 to 1 range
        float normalizedOutput = Mathf.Clamp(pidOutput, -maxPistonSpeed, maxPistonSpeed) / maxPistonSpeed;
        
        // Update PWM accumulator
        pwmAccumulator += pwmFrequency * Time.deltaTime;
        if (pwmAccumulator >= 1f) pwmAccumulator -= 1f;
        
        // Calculate duty cycle (0 to 1)
        float dutyCycle = Mathf.Abs(normalizedOutput);
        
        // Set valve states based on PWM
        if (normalizedOutput > 0)
        {
            valveExtend = pwmAccumulator < dutyCycle;
            valveRetract = false;
        }
        else if (normalizedOutput < 0)
        {
            valveExtend = false;
            valveRetract = pwmAccumulator < dutyCycle;
        }
        else
        {
            valveExtend = false;
            valveRetract = false;
        }
    }
    
    void SimulateHydraulics()
    {
        // Target velocity based on valve states
        float targetVelocity = 0f;
        if (valveExtend) targetVelocity = maxPistonSpeed;
        else if (valveRetract) targetVelocity = -maxPistonSpeed;
        
        // Apply damping to simulate hydraulic inertia
        pistonVelocity = Mathf.Lerp(pistonVelocity, targetVelocity, 
                                     hydraulicDamping * Time.deltaTime);
        
        // Update piston length
        currentPistonLength += pistonVelocity * Time.deltaTime;
        currentPistonLength = Mathf.Clamp(currentPistonLength, minPistonLength, maxPistonLength);
    }
    
    void UpdateJointFromPiston()
    {
        // Calculate angle from current piston length
        float calculatedAngle = CalculateAngleFromPiston(currentPistonLength);
        
        // Apply to joint rotation
        transform.localRotation = Quaternion.Euler(0, 0, calculatedAngle);
    }
    
    void UpdateVisualization()
    {
        // Simply read positions from the visual objects (positioned in hierarchy)
        // and update the LineRenderer to connect them
        if (pistonLine != null && pistonBaseVisual != null && pistonEndVisual != null)
        {
            pistonLine.SetPosition(0, pistonBaseVisual.position);
            pistonLine.SetPosition(1, pistonEndVisual.position);
        }
    }
    
    void OnDrawGizmos()
    {
        if (pistonBaseVisual == null || pistonEndVisual == null) return;

        Vector3 jointPos = transform.position;
        Vector3 basePos = pistonBaseVisual.position;
        Vector3 endPos = pistonEndVisual.position;

        // Draw actual piston triangle (yellow)
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(jointPos, basePos);
        Gizmos.DrawLine(jointPos, endPos);
        Gizmos.DrawLine(basePos, endPos);

        // Draw where the script THINKS the endpoints should be (cyan)
        if (Application.isPlaying && pistonBaseDistance > 0 && pistonEndDistance > 0 && transform.parent != null)
        {
            // Calculate expected base position (in parent space, doesn't rotate with joint)
            Vector3 expectedBasePos = transform.parent.TransformPoint(
                transform.parent.InverseTransformPoint(jointPos) +
                new Vector3(
                    pistonBaseDistance * Mathf.Cos(pistonBaseAngleInParentSpace * Mathf.Deg2Rad),
                    pistonBaseDistance * Mathf.Sin(pistonBaseAngleInParentSpace * Mathf.Deg2Rad),
                    0
                )
            );

            // Calculate expected end position (in joint's local space, rotates with joint)
            Vector3 expectedEndPos = transform.TransformPoint(
                new Vector3(
                    pistonEndDistance * Mathf.Cos(pistonEndAngle * Mathf.Deg2Rad),
                    pistonEndDistance * Mathf.Sin(pistonEndAngle * Mathf.Deg2Rad),
                    0
                )
            );

            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(jointPos, expectedBasePos);
            Gizmos.DrawLine(jointPos, expectedEndPos);
            Gizmos.DrawLine(expectedBasePos, expectedEndPos);

            // Draw spheres at expected positions
            Gizmos.DrawWireSphere(expectedBasePos, 0.05f);
            Gizmos.DrawWireSphere(expectedEndPos, 0.05f);

            // Draw lines showing the mismatch (red = error)
            Gizmos.color = Color.red;
            Gizmos.DrawLine(basePos, expectedBasePos);
            Gizmos.DrawLine(endPos, expectedEndPos);
        }

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(jointPos, 0.1f);
    }

    void OnGUI()
    {
        if (!showDebugGUI || pistonBaseVisual == null || pistonEndVisual == null) return;
        if (Camera.main == null) return;

        // Calculate actual distance between endpoints
        float actualDistance = Vector3.Distance(pistonBaseVisual.position, pistonEndVisual.position);

        // Calculate minimum possible piston length for this joint's geometry
        // This occurs at maximum triangle angle (180 degrees)
        float geometricMinLength = Mathf.Abs(pistonBaseDistance - pistonEndDistance);

        // Calculate maximum possible piston length for this joint's geometry
        // This occurs at minimum triangle angle (0 degrees)
        float geometricMaxLength = pistonBaseDistance + pistonEndDistance;

        // Convert world position to screen position for the joint
        Vector3 screenPos = Camera.main.WorldToScreenPoint(transform.position);

        // Unity GUI uses top-left as origin, but WorldToScreenPoint uses bottom-left
        screenPos.y = Screen.height - screenPos.y;

        // Only show if joint is in front of camera
        if (screenPos.z > 0)
        {
            // Create a GUI box with debug info
            float boxWidth = 280;
            float boxHeight = 200;
            Rect boxRect = new Rect(screenPos.x + 20, screenPos.y - boxHeight / 2, boxWidth, boxHeight);

            // Semi-transparent background
            GUI.color = new Color(0, 0, 0, 0.7f);
            GUI.Box(boxRect, "");

            // White text
            GUI.color = Color.white;

            GUIStyle style = new GUIStyle(GUI.skin.label);
            style.fontSize = 11;
            style.normal.textColor = Color.white;
            style.padding = new RectOffset(5, 5, 5, 5);

            string debugText = $"<b>{gameObject.name}</b>\n" +
                             $"━━━━━━━━━━━━━━━━━━\n" +
                             $"<b>Piston Distance:</b>\n" +
                             $"  Actual: {actualDistance:F3} m\n" +
                             $"  Simulated: {currentPistonLength:F3} m\n" +
                             $"  Min/Max: {minPistonLength:F3} / {maxPistonLength:F3} m\n" +
                             $"  Geometric Range: {geometricMinLength:F3} - {geometricMaxLength:F3} m\n" +
                             $"\n<b>Angles:</b>\n" +
                             $"  Current: {currentAngle:F1}°\n" +
                             $"  Target: {targetAngle:F1}°\n" +
                             $"\n<b>Geometry:</b>\n" +
                             $"  Base dist: {pistonBaseDistance:F3} m\n" +
                             $"  End dist: {pistonEndDistance:F3} m\n" +
                             $"\n<b>Velocity:</b> {pistonVelocity:F3} m/s";

            GUI.Label(boxRect, debugText, style);

            // Reset GUI color
            GUI.color = Color.white;
        }
    }
}