using UnityEngine;

public class HydraulicJoint : MonoBehaviour
{
    // ----- Physical piston configuration.
    [Header("Piston Limits")]
    [Tooltip("Minimum piston length (meters)")]
    public float minPistonLength = 0.128f;
    
    [Tooltip("Maximum piston length (meters)")]
    public float maxPistonLength = 0.1657f;
    
    // This is a guess straight from AI, so will need verification and tuning
    [Header("Hydraulic Properties")]
    [Tooltip("Maximum extension/retraction speed (m/s) - Based on 1/8\" valve, 4 bar, 17mm piston, water")]
    public float maxPistonSpeed = 0.02f;
    
    [Tooltip("Hydraulic damping factor (higher = faster response)")]
    public float hydraulicDamping = 1.5f;
    
    
    // ----- Control configuration.
    [Header("PWM Control")]
    [Tooltip("PWM frequency (Hz)")]
    public float pwmFrequency = 1f;
    
    // These are not yet tweaked!!
    [Header("PID Controller")]
    public float kP = 2.0f;
    public float kI = 0.1f;
    public float kD = 0.5f;
    
    // ----- Game object configuration.
    [Header("References")]
    [Tooltip("Piston base attachment point (doesn't rotate with joint)")]
    public Transform pistonBaseVisual;
    
    [Tooltip("Piston end attachment point (rotates with joint)")]
    public Transform pistonEndVisual;
    
    [Tooltip("LineRenderer to visualize piston")]
    public LineRenderer pistonLine;
    
    [Header("Target")]
    public float targetAngle = 45f;
    private float _initialAngle;

    [Header("Debug")]
    [Tooltip("Show debug GUI overlay")]
    public bool showDebugGUI = true;

    // Geometry - calculated from visual positions
    private float _pistonBaseDistance;  // Distance from joint to base attachment (in parent's space)
    private float _pistonEndDistance;   // Distance from joint to end attachment (in joint's local space)
    private float _pistonBaseAngleInParentSpace;  // Angle of base attachment in parent's space
    private float _pistonEndAngle;   // Angle of end attachment relative to joint (local space)
    
    // Internal state
    private float _currentAngle;
    private float _currentPistonLength;
    private float _pistonVelocity;
    
    // PID state
    private float _integralError;
    private float _previousError;
    
    // PWM state
    private float _pwmAccumulator;
    private bool _valveExtend;
    private bool _valveRetract;
    
    void Start() // synonymous with setup()
    {
        _initialAngle = targetAngle;
        // Calculate triangle geometry from actual visual positions
        CalculateGeometry();

        // Initialize piston length based on target angle
        _currentPistonLength = CalculatePistonLength(targetAngle);

        // Initialize velocity and PID
        _pistonVelocity = 0f;
        _integralError = 0f;
        _previousError = 0f;

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
        // - pistonBaseVisual is on the parent/fixed linkage (doesn't rotate with this joint)
        // - pistonEndVisual is on this joint's rotating linkage (rotates with this joint)

        if (pistonBaseVisual != null && transform.parent != null)
        {
            // Calculate base attachment in PARENT's space (since it doesn't rotate with this joint)
            Vector3 parentSpaceBasePos = transform.parent.InverseTransformPoint(pistonBaseVisual.position);
            Vector3 parentSpaceJointPos = transform.parent.InverseTransformPoint(transform.position);
            Vector3 baseRelativeToJoint = parentSpaceBasePos - parentSpaceJointPos;

            _pistonBaseDistance = baseRelativeToJoint.magnitude;
            _pistonBaseAngleInParentSpace = Mathf.Atan2(baseRelativeToJoint.y, baseRelativeToJoint.x) * Mathf.Rad2Deg;
        }

        if (pistonEndVisual != null)
        {
            // Calculate end attachment in this joint's LOCAL space (rotates with joint)
            Vector3 localEndPos = transform.InverseTransformPoint(pistonEndVisual.position);
            _pistonEndDistance = localEndPos.magnitude;
            _pistonEndAngle = Mathf.Atan2(localEndPos.y, localEndPos.x) * Mathf.Rad2Deg;
        }

        Debug.Log($"Joint {gameObject.name}: Base distance = {_pistonBaseDistance:F3}, End distance = {_pistonEndDistance:F3}");
        Debug.Log($"Joint {gameObject.name}: Base angle (parent space) = {_pistonBaseAngleInParentSpace:F1}°, End angle (local) = {_pistonEndAngle:F1}°");
    }
    
    void Update() // synonymous with loop()
    {
        // Update current angle from joint rotation
        // This angle will come from the potentiometer in hardware
        _currentAngle = transform.localEulerAngles.z;
        if (_currentAngle > 180f) _currentAngle -= 360f;

        // Calculate target piston length for desired angle
        float targetLength = CalculatePistonLength(targetAngle);
        
        // PID control to get desired piston velocity
        float error = targetLength - _currentPistonLength;
        _integralError += error * Time.deltaTime;
        float derivative = (error - _previousError) / Time.deltaTime;
        
        float pidOutput = kP * error + kI * _integralError + kD * derivative;
        _previousError = error;
        
        // Convert PID output to valve commands via PWM
        UpdatePWM(pidOutput);
        
        // UNITY SPECIFICS AHEAD \/ -- These are outgoing signals to the objects in the simulation,
        // not relevant for actual hardware.
        
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
        float triangleAngle = (_pistonEndAngle + jointAngle) - _pistonBaseAngleInParentSpace;
        float triangleAngleRad = triangleAngle * Mathf.Deg2Rad;

        // Use law of cosines: c² = a² + b² - 2ab*cos(C)
        // where c is the piston length, a and b are the attachment distances
        //Debug.Log(_pistonBaseDistance +" "+ _pistonEndDistance);
        
        float length = Mathf.Sqrt(
            _pistonBaseDistance * _pistonBaseDistance +
            _pistonEndDistance * _pistonEndDistance -
            2 * _pistonBaseDistance * _pistonEndDistance * Mathf.Cos(triangleAngleRad)
        );

        return Mathf.Clamp(length, minPistonLength, maxPistonLength);
    }
    
    float CalculateAngleFromPiston(float pistonLength)
    {
        // Use law of cosines to get the triangle angle (always positive from Acos)
        // cos(C) = (a² + b² - c²) / (2ab)
        float cosAngle = (_pistonBaseDistance * _pistonBaseDistance +
                         _pistonEndDistance * _pistonEndDistance -
                         pistonLength * pistonLength) /
                        (2 * _pistonBaseDistance * _pistonEndDistance);

        cosAngle = Mathf.Clamp(cosAngle, -1f, 1f);
        float triangleAngle = Mathf.Acos(cosAngle) * Mathf.Rad2Deg;

        // Convert triangle angle back to joint angle
        // triangleAngle = (pistonEndAngle + jointAngle) - pistonBaseAngleInParentSpace
        // Solving for jointAngle: jointAngle = triangleAngle + pistonBaseAngleInParentSpace - pistonEndAngle
        float jointAnglePositive = triangleAngle + _pistonBaseAngleInParentSpace - _pistonEndAngle;
        float jointAngleNegative = -triangleAngle + _pistonBaseAngleInParentSpace - _pistonEndAngle;

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
        _pwmAccumulator += pwmFrequency * Time.deltaTime;
        if (_pwmAccumulator >= 1f) _pwmAccumulator -= 1f;
        
        /*
         * With the PWM frequency set at 1 Hz, the minimum duty cycle is 0.2/1 = 0.2s. We need to clamp the PWM
         * to 0%, 20%-80%, and 100%.
         */
        // Calculate duty cycle (0 to 1)
        float dutyCycle = Mathf.Abs(normalizedOutput);

        dutyCycle = dutyCycle switch
        {
            < 0.002f => 0f,
            > 1 - 0.002f => 1f,
            < 0.2f => 0.2f,
            > 0.8f => 0.8f,
            _ => dutyCycle
        };

        // Set valve states based on PWM. The valve is open when the accumulator is below the duty cycle.
        if (normalizedOutput > 0)
        {
            _valveExtend = _pwmAccumulator < dutyCycle;
            _valveRetract = false;
        }
        else if (normalizedOutput < 0)
        {
            _valveExtend = false;
            _valveRetract = _pwmAccumulator < dutyCycle;
        }
        else
        {
            _valveExtend = false;
            _valveRetract = false;
        }
    }
    
    void SimulateHydraulics()
    {
        // Target velocity based on valve states
        float targetVelocity = 0f;
        if (_valveExtend) targetVelocity = maxPistonSpeed;
        else if (_valveRetract) targetVelocity = -maxPistonSpeed;
        
        // Apply damping to simulate hydraulic inertia
        _pistonVelocity = Mathf.Lerp(_pistonVelocity, targetVelocity, 
                                     hydraulicDamping * Time.deltaTime);
        
        // Update piston length
        _currentPistonLength += _pistonVelocity * Time.deltaTime;
        _currentPistonLength = Mathf.Clamp(_currentPistonLength, minPistonLength, maxPistonLength);
    }
    
    void UpdateJointFromPiston()
    {
        // Calculate angle from current piston length
        float calculatedAngle = CalculateAngleFromPiston(_currentPistonLength);
        
        // Apply to joint rotation
        transform.localRotation = Quaternion.Euler(0, 0, calculatedAngle);
    }
    
    void UpdateVisualization()
    {
        // Simply read positions from the visual objects (positioned in hierarchy)
        // and update the LineRenderer to connect them
        if (pistonLine && pistonBaseVisual && pistonEndVisual)
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
        if (Application.isPlaying && _pistonBaseDistance > 0 && _pistonEndDistance > 0 && transform.parent != null)
        {
            // Calculate expected base position (in parent space, doesn't rotate with joint)
            Vector3 expectedBasePos = transform.parent.TransformPoint(
                transform.parent.InverseTransformPoint(jointPos) +
                new Vector3(
                    _pistonBaseDistance * Mathf.Cos(_pistonBaseAngleInParentSpace * Mathf.Deg2Rad),
                    _pistonBaseDistance * Mathf.Sin(_pistonBaseAngleInParentSpace * Mathf.Deg2Rad),
                    0
                )
            );

            // Calculate expected end position (in joint's local space, rotates with joint)
            Vector3 expectedEndPos = transform.TransformPoint(
                new Vector3(
                    _pistonEndDistance * Mathf.Cos(_pistonEndAngle * Mathf.Deg2Rad),
                    _pistonEndDistance * Mathf.Sin(_pistonEndAngle * Mathf.Deg2Rad),
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
        float geometricMinLength = Mathf.Abs(_pistonBaseDistance - _pistonEndDistance);

        // Calculate maximum possible piston length for this joint's geometry
        // This occurs at minimum triangle angle (0 degrees)
        float geometricMaxLength = _pistonBaseDistance + _pistonEndDistance;

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
            style.fontSize = 8;
            style.normal.textColor = Color.white;
            style.padding = new RectOffset(5, 5, 5, 5);

            string debugText = $"<b>{gameObject.name}</b>\n" +
                             $"━━━━━━━━━━━━━━━━━━\n" +
                             $"<b>Piston Distance:</b>\n" +
                             $"  Actual: {actualDistance:F3} m\n" +
                             $"  Simulated: {_currentPistonLength:F3} m\n" +
                             $"  Min/Max: {minPistonLength:F3} / {maxPistonLength:F3} m\n" +
                             $"  Geometric Range: {geometricMinLength:F3} - {geometricMaxLength:F3} m\n" +
                             $"\n<b>Angles:</b>\n" +
                             $"  Current: {_currentAngle:F1}°\n" +
                             $"  Target: {targetAngle:F1}°\n" +
                             $"\n<b>Geometry:</b>\n" +
                             $"  Base dist: {_pistonBaseDistance:F3} m\n" +
                             $"  End dist: {_pistonEndDistance:F3} m\n" +
                             $"\n<b>Velocity:</b> {_pistonVelocity:F3} m/s" +
                             $"\n<b>ValveState:</b> {_valveRetract} {_valveExtend}";

            GUI.Label(boxRect, debugText, style);

            // Reset GUI color
            GUI.color = Color.white;
        }
    }

    public void resetToInit()
    {
        targetAngle = _initialAngle;
    }
}