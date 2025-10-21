using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActiveRagdoll : MonoBehaviour
{
    [Header("Core References")]
    public Transform hipsTransform;
    public Transform stepGuide;

    [Header("IK Targets")]
    public Transform leftFootIKTarget;
    public Transform rightFootIKTarget;

    [Header("Standing Position Values")]
    public float footSpacing = 0.3f;
    public float standingHeight = 0.9f;

    [Header("Step Values")]
    public float stepDuration = 0.2f;
    public float stepHeight = 0.3f;
    [Tooltip("Base stride length for normal walking")]
    public float baseStrideLength = 0.4f;
    [Tooltip("Maximum stride length for urgent steps")]
    public float maxStrideLength = 0.8f;
    [Tooltip("Minimum distance before considering a step")]
    public float minStepDistance = 0.15f;

    [Header("Balance Thresholds")]
    [Tooltip("How far CoM can drift from support center before stepping (in meters)")]
    public float stabilityThreshold = 0.4f;
    [Tooltip("How far ahead to predict CoM position based on velocity")]
    public float predictionTime = 0.2f;
    [Tooltip("Velocity at which we start taking longer strides")]
    public float urgentVelocityThreshold = 1.0f;

    [Header("Foot Correction")]
    [Tooltip("How far feet can drift from ideal before correcting")]
    public float footCorrectionThreshold = 0.15f;
    [Tooltip("Time idle before correcting stance")]
    public float idleCorrectionDelay = 1.0f;
    [Tooltip("Velocity threshold for 'idle' state")]
    public float idleVelocityThreshold = 0.2f;

    [Header("Physics")]
    public int balanceForce = 10;
    public int balanceDamping = 1;
    public LayerMask ragdollLayer;
    public LayerMask groundLayer;

    // Private State
    private Vector3 _leftFootGroundTarget, _rightFootGroundTarget;
    private bool _isStepping = false;
    private float _lastStepTime = 0f;
    private bool _lastStepWasLeft = false;
    private int _consecutiveSameFootSteps = 0;
    private Rigidbody _hipsRigidbody;
    private List<Rigidbody> _allRigidbodies = new List<Rigidbody>();

    [HideInInspector] public ConfigurableJoint balanceJoint;
    [HideInInspector] public Rigidbody balanceTargetBody;
    [HideInInspector] public bool isAirborne;

    // Center of Mass tracking
    private Vector3 _centerOfMass;
    private Vector3 _centerOfMassVelocity;
    private Vector3 _lastCenterOfMass;

    void Awake()
    {
        _hipsRigidbody = hipsTransform.GetComponent<Rigidbody>();
        _allRigidbodies.AddRange(GetComponentsInChildren<Rigidbody>());

        balanceTargetBody = new GameObject("Balance Target Body").AddComponent<Rigidbody>();
        balanceTargetBody.transform.parent = transform;
        balanceTargetBody.isKinematic = true;
        SetupBalanceJoint();
    }

    void Start()
    {
        stepGuide.position = hipsTransform.position;

        // Initialize foot positions
        _leftFootGroundTarget = GetGroundPoint(hipsTransform.position - hipsTransform.right * footSpacing);
        _rightFootGroundTarget = GetGroundPoint(hipsTransform.position + hipsTransform.right * footSpacing);

        leftFootIKTarget.position = _leftFootGroundTarget;
        rightFootIKTarget.position = _rightFootGroundTarget;

        _lastCenterOfMass = CalculateCenterOfMass();
    }

    void Update()
    {
        // Only handle visual/input stuff here
        if (!_isStepping && !isAirborne)
        {
            // Visualization updates only
        }

        // Align foot rotations with hips
        AlignFootRotations();
    }

    void FixedUpdate()
    {
        // ALL physics-related calculations must be in FixedUpdate

        // Update CoM tracking with FIXED timestep
        _centerOfMass = CalculateCenterOfMass();
        _centerOfMassVelocity = (_centerOfMass - _lastCenterOfMass) / Time.fixedDeltaTime;
        _lastCenterOfMass = _centerOfMass;

        CheckAirborne();
        UpdateStepGuidePosition();

        // Handle stepping logic
        if (!_isStepping && !isAirborne)
        {
            if (ShouldTakeStep(out StepInfo stepInfo))
            {
                ExecuteStep(stepInfo);
            }
            else if (IsIdle() && ShouldCorrectStance(out bool correctLeft))
            {
                ExecuteStanceCorrection(correctLeft);
            }
        }

        // Update balance target
        if (!isAirborne)
        {
            UpdateBalanceTarget();
        }
    }

    #region Center of Mass

    private Vector3 CalculateCenterOfMass()
    {
        Vector3 com = Vector3.zero;
        float totalMass = 0f;

        foreach (Rigidbody rb in _allRigidbodies)
        {
            if (rb != null && rb != balanceTargetBody)
            {
                com += rb.worldCenterOfMass * rb.mass;
                totalMass += rb.mass;
            }
        }

        return totalMass > 0 ? com / totalMass : hipsTransform.position;
    }

    #endregion

    #region Step Decision Logic

    private struct StepInfo
    {
        public bool stepLeft;
        public Vector3 targetPosition;
        public float urgency; // 0-1, affects step speed/height
    }

    private bool ShouldTakeStep(out StepInfo stepInfo)
    {
        stepInfo = new StepInfo();

        // Get current support center and CoM projection
        Vector3 supportCenter = (_leftFootGroundTarget + _rightFootGroundTarget) / 2f;
        Vector3 comGroundProj = new Vector3(_centerOfMass.x, supportCenter.y, _centerOfMass.z);

        // Calculate current offset
        Vector3 comOffset = comGroundProj - supportCenter;
        comOffset.y = 0;

        // Predict future CoM position
        Vector3 horizontalVelocity = new Vector3(_centerOfMassVelocity.x, 0, _centerOfMassVelocity.z);
        Vector3 predictedComOffset = comOffset + horizontalVelocity * predictionTime;

        // FIXED: Use absolute distance, not ratio of footSpacing
        float stabilityRadius = stabilityThreshold;

        // Check if we need to step
        float instability = predictedComOffset.magnitude;

        // Also check current instability (without prediction) to avoid over-predicting
        float currentInstability = comOffset.magnitude;

        // Only step if EITHER predicted OR current instability exceeds threshold
        // This prevents over-eager stepping from velocity prediction
        if (instability <= stabilityRadius && currentInstability <= stabilityRadius * 1.2f)
            return false;

        // Determine urgency based on how far out of balance we are
        stepInfo.urgency = Mathf.Clamp01((instability - stabilityRadius) / stabilityRadius);

        // Determine which foot should step
        stepInfo.stepLeft = DetermineSteppingFoot(predictedComOffset, horizontalVelocity);

        // ANTI-STUCK MECHANISM: If same foot has stepped 3+ times in a row, force alternate
        if (stepInfo.stepLeft == _lastStepWasLeft && _consecutiveSameFootSteps >= 2)
        {
            // Debug.LogWarning($"[ANTI-STUCK] Same foot stepped {_consecutiveSameFootSteps + 1} times! Forcing alternate to {(!stepInfo.stepLeft ? "LEFT" : "RIGHT")}");
            stepInfo.stepLeft = !stepInfo.stepLeft;
        }

        // Calculate target position
        stepInfo.targetPosition = CalculateStepTarget(
            stepInfo.stepLeft,
            predictedComOffset,
            horizontalVelocity,
            stepInfo.urgency
        );

        // Debug.Log($"[STEP DECISION] Current: {currentInstability:F2} | Predicted: {instability:F2} | Threshold: {stabilityRadius:F2} | Urgency: {stepInfo.urgency:F2} | Stepping: {(stepInfo.stepLeft ? "LEFT" : "RIGHT")} | ConsecutiveSameFoot: {_consecutiveSameFootSteps}");

        return true;
    }

    private bool DetermineSteppingFoot(Vector3 comOffset, Vector3 velocity)
    {
        // Calculate distances from each foot to the predicted CoM position
        Vector3 supportCenter = (_leftFootGroundTarget + _rightFootGroundTarget) / 2f;
        Vector3 predictedComPos = supportCenter + comOffset + new Vector3(velocity.x, 0, velocity.z) * predictionTime;

        float leftDistToCom = Vector3.Distance(_leftFootGroundTarget, predictedComPos);
        float rightDistToCom = Vector3.Distance(_rightFootGroundTarget, predictedComPos);

        // Primary rule: Step with the foot that's FARTHER from where the CoM is going
        // This allows the closer foot to stay planted while the far foot reaches out
        bool stepLeftByDistance = leftDistToCom > rightDistToCom;

        // Check if one foot is significantly behind the other in the direction of motion
        Vector3 motionDir = velocity.magnitude > 0.1f ? velocity.normalized : comOffset.normalized;
        motionDir.y = 0;

        float leftProgressInMotion = Vector3.Dot(_leftFootGroundTarget - supportCenter, motionDir);
        float rightProgressInMotion = Vector3.Dot(_rightFootGroundTarget - supportCenter, motionDir);

        // If one foot is significantly behind, prioritize stepping with that foot
        float trailingDifference = Mathf.Abs(leftProgressInMotion - rightProgressInMotion);
        if (trailingDifference > baseStrideLength * 0.4f)
        {
            bool leftIsBehind = leftProgressInMotion < rightProgressInMotion;
            //  Debug.Log($"[FOOT SELECTION] Trailing detected - Left progress: {leftProgressInMotion:F2} | Right progress: {rightProgressInMotion:F2} | Stepping {(leftIsBehind ? "LEFT" : "RIGHT")} (trailing foot)");
            return leftIsBehind;
        }

        // Check lateral component - are we falling to one side?
        float lateralComponent = Vector3.Dot(comOffset, hipsTransform.right);

        // If falling strongly to one side, prefer stepping with that side's foot
        if (Mathf.Abs(lateralComponent) > footSpacing * 0.5f)
        {
            bool stepLeftByLateral = lateralComponent < 0;
            // Debug.Log($"[FOOT SELECTION] Lateral fall detected: {lateralComponent:F2} | Stepping {(stepLeftByLateral ? "LEFT" : "RIGHT")}");

            // But don't step with a foot that just stepped recently
            float timeSinceLastStep = Time.time - _lastStepTime;
            if (timeSinceLastStep < stepDuration * 1.5f)
            {
                // Very recent step - prefer alternating unless absolutely necessary
                bool lastStepWasLeft = Vector3.Distance(leftFootIKTarget.position, _leftFootGroundTarget) < 0.01f;
                if (lastStepWasLeft == stepLeftByLateral)
                {
                    // Debug.Log($"[FOOT SELECTION] Overriding - same foot stepped recently, alternating to {(!stepLeftByLateral ? "LEFT" : "RIGHT")}");
                    return !stepLeftByLateral;
                }
            }

            return stepLeftByLateral;
        }

        // Default: step with foot farther from predicted CoM
        // Debug.Log($"[FOOT SELECTION] Distance-based - Left dist: {leftDistToCom:F2} | Right dist: {rightDistToCom:F2} | Stepping {(stepLeftByDistance ? "LEFT" : "RIGHT")}");
        return stepLeftByDistance;
    }

    private Vector3 CalculateStepTarget(bool stepLeft, Vector3 comOffset, Vector3 velocity, float urgency)
    {
        // Determine stride length based on urgency and velocity
        float velocityMagnitude = velocity.magnitude;
        float strideLength = Mathf.Lerp(
            baseStrideLength,
            maxStrideLength,
            Mathf.Clamp01(velocityMagnitude / urgentVelocityThreshold)
        );
        strideLength = Mathf.Max(strideLength, minStepDistance);

        // Calculate where the new support center should be
        // It should be ahead of current CoM in the direction of motion/fall
        Vector3 desiredDirection = (comOffset + velocity * predictionTime).normalized;
        Vector3 newSupportCenter = _centerOfMass + desiredDirection * (strideLength * 0.5f);
        newSupportCenter.y = (_leftFootGroundTarget.y + _rightFootGroundTarget.y) / 2f;

        // Calculate the stepping foot's target position
        Vector3 lateralOffset = hipsTransform.right * (stepLeft ? -footSpacing : footSpacing);

        // Add forward bias based on velocity
        Vector3 forwardDir = new Vector3(velocity.x, 0, velocity.z).normalized;
        if (velocity.magnitude > idleVelocityThreshold)
        {
            newSupportCenter += forwardDir * (strideLength * 0.3f);
        }

        Vector3 targetPos = newSupportCenter + lateralOffset;

        // **START MODIFICATION: Wall Check - Prevent target from going through walls**

        // Use the hips' current horizontal position as the ray origin for the wall check.
        Vector3 rayStart = hipsTransform.position;
        rayStart.y = targetPos.y; // Match the target's height for a horizontal check

        Vector3 directionToTarget = targetPos - rayStart;
        float distanceToTarget = directionToTarget.magnitude;

        // Raycast horizontally from the hips towards the proposed target, ignoring ragdoll layers
        if (distanceToTarget > 0.01f && Physics.Raycast(rayStart, directionToTarget.normalized, out RaycastHit wallHit, distanceToTarget, ~ragdollLayer))
        {
            // If a non-ragdoll object is hit, clamp the target position just before the wall
            float wallPadding = 0.05f; // Small buffer distance from the wall
            // Clamp the target position to the hit point, pulled back by the padding
            targetPos = rayStart + directionToTarget.normalized * (wallHit.distance - wallPadding);
            // Debug.Log($"[STEP CLAMP] Clamped step target to {wallHit.distance:F2}m from hips due to obstacle."); // Uncomment for debug
        }

        // **END MODIFICATION**

        // Raycast to find actual ground
        targetPos = GetGroundPoint(targetPos);

        return targetPos;
    }

    #endregion

    #region Stance Correction

    private bool IsIdle()
    {
        Vector3 horizontalVelocity = new Vector3(_centerOfMassVelocity.x, 0, _centerOfMassVelocity.z);
        bool hasLowVelocity = horizontalVelocity.magnitude < idleVelocityThreshold;
        bool hasBeenStill = Time.time - _lastStepTime > idleCorrectionDelay;

        return hasLowVelocity && hasBeenStill;
    }

    private bool ShouldCorrectStance(out bool correctLeft)
    {
        correctLeft = false;

        Vector3 idealLeft = GetIdealFootPosition(true);
        Vector3 idealRight = GetIdealFootPosition(false);

        float leftDrift = Vector3.Distance(_leftFootGroundTarget, idealLeft);
        float rightDrift = Vector3.Distance(_rightFootGroundTarget, idealRight);

        if (leftDrift > footCorrectionThreshold || rightDrift > footCorrectionThreshold)
        {
            correctLeft = leftDrift > rightDrift;
            //  Debug.Log($"[STANCE CORRECTION] {(correctLeft ? "LEFT" : "RIGHT")} foot drift: {(correctLeft ? leftDrift : rightDrift):F3}");
            return true;
        }

        return false;
    }

    private Vector3 GetIdealFootPosition(bool isLeftFoot)
    {
        Vector3 offset = hipsTransform.right * (isLeftFoot ? -footSpacing : footSpacing);
        Vector3 idealPos = hipsTransform.position + offset;
        return GetGroundPoint(idealPos);
    }

    #endregion

    #region Step Execution

    private void ExecuteStep(StepInfo stepInfo)
    {
        // Track consecutive same-foot steps
        if (stepInfo.stepLeft == _lastStepWasLeft)
        {
            _consecutiveSameFootSteps++;
        }
        else
        {
            _consecutiveSameFootSteps = 0;
        }
        _lastStepWasLeft = stepInfo.stepLeft;

        if (stepInfo.stepLeft)
        {
            _leftFootGroundTarget = stepInfo.targetPosition;
            StartCoroutine(PerformStep(leftFootIKTarget, _leftFootGroundTarget, stepInfo.urgency));
        }
        else
        {
            _rightFootGroundTarget = stepInfo.targetPosition;
            StartCoroutine(PerformStep(rightFootIKTarget, _rightFootGroundTarget, stepInfo.urgency));
        }

        _lastStepTime = Time.time;
    }

    private void ExecuteStanceCorrection(bool correctLeft)
    {
        Vector3 target = GetIdealFootPosition(correctLeft);

        if (correctLeft)
        {
            _leftFootGroundTarget = target;
            StartCoroutine(PerformStep(leftFootIKTarget, target, 0f));
        }
        else
        {
            _rightFootGroundTarget = target;
            StartCoroutine(PerformStep(rightFootIKTarget, target, 0f));
        }

        _lastStepTime = Time.time;
    }

    private IEnumerator PerformStep(Transform foot, Vector3 target, float urgency)
    {
        _isStepping = true;

        Vector3 startPoint = foot.position;
        float distance = Vector3.Distance(startPoint, target);

        // Adjust step height and duration based on urgency
        float actualStepHeight = stepHeight * Mathf.Lerp(0.5f, 1.2f, urgency);
        float actualDuration = stepDuration * Mathf.Lerp(1.2f, 0.8f, urgency);

        // Debug.Log($"[STEP] {foot.name}: {distance:F2}m | Urgency: {urgency:F2} | Height: {actualStepHeight:F2} | Duration: {actualDuration:F2}s");

        Vector3 centerPoint = (startPoint + target) / 2;
        centerPoint.y = Mathf.Max(startPoint.y, target.y) + actualStepHeight;

        float timeElapsed = 0;

        while (timeElapsed < actualDuration)
        {
            timeElapsed += Time.deltaTime;
            float t = timeElapsed / actualDuration;

            // Quadratic bezier curve for smooth step arc
            foot.position = Vector3.Lerp(
                Vector3.Lerp(startPoint, centerPoint, t),
                Vector3.Lerp(centerPoint, target, t),
                t
            );

            yield return null;
        }

        foot.position = target;
        _isStepping = false;
    }

    #endregion

    #region Utility

    private Vector3 GetGroundPoint(Vector3 position)
    {
        if (Physics.Raycast(position + Vector3.up * 2f, Vector3.down, out RaycastHit hit, 5f, groundLayer))
        {
            return hit.point;
        }

        position.y = hipsTransform.position.y - standingHeight;
        return position;
    }

    private void CheckAirborne()
    {
        bool leftGrounded = Physics.Raycast(leftFootIKTarget.position + Vector3.up * 0.1f, Vector3.down, 0.3f, groundLayer);
        bool rightGrounded = Physics.Raycast(rightFootIKTarget.position + Vector3.up * 0.1f, Vector3.down, 0.3f, groundLayer);
        isAirborne = !leftGrounded && !rightGrounded;
    }

    private void AlignFootRotations()
    {
        leftFootIKTarget.eulerAngles = new Vector3(leftFootIKTarget.eulerAngles.x, hipsTransform.eulerAngles.y, leftFootIKTarget.eulerAngles.z);
        rightFootIKTarget.eulerAngles = new Vector3(rightFootIKTarget.eulerAngles.x, hipsTransform.eulerAngles.y, rightFootIKTarget.eulerAngles.z);
    }

    private void UpdateBalanceTarget()
    {
        Vector3 avgFootPos = (_leftFootGroundTarget + _rightFootGroundTarget) / 2f;
        balanceTargetBody.transform.position = new Vector3(
            hipsTransform.position.x,
            avgFootPos.y + standingHeight,
            hipsTransform.position.z
        );
    }

    public void UpdateStepGuidePosition()
    {
        Vector3 comHorizontal = _centerOfMass;
        comHorizontal.y = hipsTransform.position.y;
        stepGuide.position = Vector3.Lerp(stepGuide.position, comHorizontal, Time.deltaTime * 5f);
        stepGuide.eulerAngles = new Vector3(0, hipsTransform.eulerAngles.y, 0);
    }

    #endregion

    #region Physics Setup

    public void SetupBalanceJoint()
    {
        if (balanceJoint != null)
            Destroy(balanceJoint);

        balanceJoint = hipsTransform.gameObject.AddComponent<ConfigurableJoint>();

        JointDrive driveX = balanceJoint.angularXDrive;
        JointDrive driveYZ = balanceJoint.angularYZDrive;
        JointDrive driveY = balanceJoint.yDrive;

        driveX.positionSpring = balanceForce;
        driveX.positionDamper = balanceDamping;

        driveYZ.positionSpring = balanceForce;
        driveYZ.positionDamper = balanceDamping;

        driveY.positionSpring = 300;
        driveY.positionDamper = 10;

        balanceJoint.angularXDrive = driveX;
        balanceJoint.angularYZDrive = driveYZ;
        balanceJoint.yDrive = driveY;

        balanceTargetBody.transform.position = hipsTransform.position;
        balanceTargetBody.transform.rotation = hipsTransform.rotation;
        balanceJoint.connectedBody = balanceTargetBody;
        balanceTargetBody.transform.rotation = Quaternion.identity;
    }

    #endregion

    #region Debug Visualization

    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        // Center of Mass
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(_centerOfMass, 0.1f);

        // CoM Velocity
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(_centerOfMass, _centerOfMass + _centerOfMassVelocity * 0.5f);

        // Support base
        Gizmos.color = Color.green;
        Gizmos.DrawLine(_leftFootGroundTarget, _rightFootGroundTarget);

        // Stability circle
        Vector3 supportCenter = (_leftFootGroundTarget + _rightFootGroundTarget) / 2f;
        float stabilityRadius = stabilityThreshold;
        Gizmos.color = new Color(0, 1, 0, 0.3f);
        DrawCircle(supportCenter, stabilityRadius, 32);

        // CoM ground projection
        Vector3 comGroundProj = new Vector3(_centerOfMass.x, supportCenter.y, _centerOfMass.z);
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(_centerOfMass, comGroundProj);
        Gizmos.DrawSphere(comGroundProj, 0.08f);

        // Predicted CoM
        Vector3 predictedCom = comGroundProj + new Vector3(_centerOfMassVelocity.x, 0, _centerOfMassVelocity.z) * predictionTime;
        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(predictedCom, 0.08f);
        Gizmos.DrawLine(comGroundProj, predictedCom);
    }

    void DrawCircle(Vector3 center, float radius, int segments)
    {
        float angleStep = 360f / segments;
        Vector3 prevPoint = center + new Vector3(radius, 0, 0);

        for (int i = 1; i <= segments; i++)
        {
            float angle = angleStep * i * Mathf.Deg2Rad;
            Vector3 newPoint = center + new Vector3(Mathf.Cos(angle) * radius, 0, Mathf.Sin(angle) * radius);
            Gizmos.DrawLine(prevPoint, newPoint);
            prevPoint = newPoint;
        }
    }

    #endregion
}