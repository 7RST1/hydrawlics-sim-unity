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
    
    // Geometry - calculated from visual positions
    private float pistonBaseDistance;
    private float pistonEndDistance;
    
    // Internal state
    private float currentAngle;
    private float currentPistonLength;
    private float pistonVelocity;
    private bool useNegativeAngle; // Track which side of the triangle we're on
    
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
        
        // Determine which side of the triangle based on initial target
        useNegativeAngle = targetAngle < 0;
        
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
        // Calculate distances from joint pivot to attachment points
        if (pistonBaseVisual != null)
        {
            Vector3 localBasePos = transform.InverseTransformPoint(pistonBaseVisual.position);
            pistonBaseDistance = localBasePos.magnitude;
        }
        
        if (pistonEndVisual != null)
        {
            Vector3 localEndPos = transform.InverseTransformPoint(pistonEndVisual.position);
            pistonEndDistance = localEndPos.magnitude;
        }
        
        Debug.Log($"Joint {gameObject.name}: Base distance = {pistonBaseDistance:F3}, End distance = {pistonEndDistance:F3}");
    }
    
    void Update()
    {
        // Update current angle from joint rotation
        currentAngle = transform.localEulerAngles.z;
        if (currentAngle > 180f) currentAngle -= 360f;
        
        // Track which side of triangle we should be on based on target
        if (Mathf.Abs(targetAngle) > 5f) // Only switch if target is clearly positive or negative
        {
            useNegativeAngle = targetAngle < 0;
        }
        
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
    
    float CalculatePistonLength(float angle)
    {
        float angleRad = angle * Mathf.Deg2Rad;
        float length = Mathf.Sqrt(
            pistonBaseDistance * pistonBaseDistance + 
            pistonEndDistance * pistonEndDistance - 
            2 * pistonBaseDistance * pistonEndDistance * Mathf.Cos(angleRad)
        );
        //print("Len: " + length + ", min: " + minPistonLength + ", max: " + maxPistonLength);;
        return Mathf.Clamp(length, minPistonLength, maxPistonLength);
    }
    
    float CalculateAngleFromPiston(float pistonLength)
    {
        float cosAngle = (pistonBaseDistance * pistonBaseDistance + 
                         pistonEndDistance * pistonEndDistance - 
                         pistonLength * pistonLength) / 
                        (2 * pistonBaseDistance * pistonEndDistance);
        
        cosAngle = Mathf.Clamp(cosAngle, -1f, 1f);
        float angle = Mathf.Acos(cosAngle) * Mathf.Rad2Deg;
        
        // Apply the correct sign based on which side of the triangle we're on
        if (useNegativeAngle)
        {
            angle = -angle;
        }
        
        return angle;
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
        
        // Draw piston triangle
        Gizmos.color = Color.yellow;
        
        Vector3 jointPos = transform.position;
        Vector3 basePos = pistonBaseVisual.position;
        Vector3 endPos = pistonEndVisual.position;
        
        Gizmos.DrawLine(jointPos, basePos);
        Gizmos.DrawLine(jointPos, endPos);
        Gizmos.DrawLine(basePos, endPos);
        
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(jointPos, 0.1f);
    }
}