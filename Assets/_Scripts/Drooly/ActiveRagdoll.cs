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
    public float footSpacing = 0.3f;  // INCREASED: was 0.2
    public float standingHeight = 0.9f;

    [Header("Step Values")]
    public float stepDuration = 0.2f;
    public float stepHeight = 0.3f;
    public float minStepDistance = 0.15f;
    public float maxStepDistance = 1.2f;

    [Header("Center of Mass Balance")]
    [Tooltip("Distance CoM can be from support center before stepping (as ratio of foot spacing)")]
    public float stabilityThreshold = 1.2f;  // INCREASED: was 0.7
    [Tooltip("How far ahead to place step based on CoM velocity")]
    public float stepPredictionTime = 0.15f;  // REDUCED: was 0.3
    [Tooltip("Minimum CoM velocity to trigger velocity-based stepping")]
    public float minVelocityForPrediction = 0.5f;

    [Header("Foot Correction")]
    [Tooltip("How far feet can be from ideal position before correcting")]
    public float footCorrectionThreshold = 0.15f;
    [Tooltip("Time to wait before correcting foot positions when idle")]
    public float idleCorrectionDelay = 1.0f;
    [Tooltip("Velocity threshold to consider character as 'moving' (disables correction)")]
    public float movementVelocityThreshold = 0.3f;

    [Header("Physics")]
    public int balanceForce = 10;
    public int balanceDamping = 1;
    public LayerMask ragdollLayer;

    // Private State
    private Vector3 _leftFootGroundTarget, _rightFootGroundTarget;
    private bool _isStepping = false;
    private float _lastMovementTime = 0f;
    private Rigidbody _hipsRigidbody;
    private List<Rigidbody> _allRigidbodies = new List<Rigidbody>();

    [HideInInspector]
    public ConfigurableJoint balanceJoint;
    [HideInInspector]
    public Rigidbody balanceTargetBody;
    [HideInInspector]
    public bool isAirborne;

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

        if (Physics.Raycast(hipsTransform.position + hipsTransform.right * -footSpacing, Vector3.down, out RaycastHit hitL, 5, ~ragdollLayer))
            _leftFootGroundTarget = hitL.point;
        else
            _leftFootGroundTarget = hipsTransform.position + hipsTransform.right * -footSpacing + Vector3.down * standingHeight;

        if (Physics.Raycast(hipsTransform.position + hipsTransform.right * footSpacing, Vector3.down, out RaycastHit hitR, 5, ~ragdollLayer))
            _rightFootGroundTarget = hitR.point;
        else
            _rightFootGroundTarget = hipsTransform.position + hipsTransform.right * footSpacing + Vector3.down * standingHeight;

        leftFootIKTarget.position = _leftFootGroundTarget;
        rightFootIKTarget.position = _rightFootGroundTarget;

        _lastCenterOfMass = CalculateCenterOfMass();
    }

    void Update()
    {
        _centerOfMass = CalculateCenterOfMass();
        _centerOfMassVelocity = (_centerOfMass - _lastCenterOfMass) / Time.deltaTime;
        _lastCenterOfMass = _centerOfMass;

        CheckAirborne();
        UpdateStepGuidePosition();

        if (!_isStepping && !isAirborne)
        {
            HandleCenterOfMassStepping();
        }

        leftFootIKTarget.eulerAngles = new Vector3(leftFootIKTarget.eulerAngles.x, hipsTransform.eulerAngles.y, leftFootIKTarget.eulerAngles.z);
        rightFootIKTarget.eulerAngles = new Vector3(rightFootIKTarget.eulerAngles.x, hipsTransform.eulerAngles.y, rightFootIKTarget.eulerAngles.z);

        if (!isAirborne)
        {
            Vector3 avgFootPos = (_leftFootGroundTarget + _rightFootGroundTarget) / 2f;
            balanceTargetBody.transform.position = new Vector3(
                hipsTransform.position.x,
                avgFootPos.y + standingHeight,
                hipsTransform.position.z
            );
        }
    }

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

    private void CheckAirborne()
    {
        bool leftGrounded = Physics.Raycast(leftFootIKTarget.position + Vector3.up * 0.1f, Vector3.down, 0.3f, ~ragdollLayer);
        bool rightGrounded = Physics.Raycast(rightFootIKTarget.position + Vector3.up * 0.1f, Vector3.down, 0.3f, ~ragdollLayer);

        isAirborne = !leftGrounded && !rightGrounded;
    }

    private void HandleCenterOfMassStepping()
    {
        Vector3 comGroundProjection = _centerOfMass;
        comGroundProjection.y = (_leftFootGroundTarget.y + _rightFootGroundTarget.y) / 2f;

        Vector3 supportCenter = (_leftFootGroundTarget + _rightFootGroundTarget) / 2f;

        Vector3 comOffset = comGroundProjection - supportCenter;
        comOffset.y = 0;

        // FIXED: Use intended foot spacing, not current distance between feet
        float stabilityRadius = footSpacing * stabilityThreshold;

        Vector3 predictedComOffset = comOffset;
        if (_centerOfMassVelocity.magnitude > minVelocityForPrediction)
        {
            Vector3 velocityContribution = _centerOfMassVelocity;
            velocityContribution.y = 0;
            // DAMPED: Limit prediction to avoid overshooting
            float predictionMagnitude = Mathf.Min(velocityContribution.magnitude * stepPredictionTime, stabilityRadius * 1.2f);
            predictedComOffset += velocityContribution.normalized * predictionMagnitude;
        }

        float instabilityDistance = predictedComOffset.magnitude;

        Vector3 horizontalComVel = _centerOfMassVelocity;
        horizontalComVel.y = 0;
        Debug.Log($"[STEPPING CHECK] CoM Offset: {instabilityDistance:F2} | Threshold: {stabilityRadius:F2} | Stable: {instabilityDistance <= stabilityRadius} | CoM Velocity: {horizontalComVel.magnitude:F2}");

        if (instabilityDistance > stabilityRadius)
        {
            Debug.Log($">>> INSTABILITY DETECTED - Taking CoM-based step");
            
            Vector3 stepDirection = predictedComOffset.normalized;
            Vector3 desiredStepPosition = supportCenter + stepDirection * Mathf.Min(instabilityDistance, maxStepDistance);

            if (Physics.Raycast(desiredStepPosition + Vector3.up * 2f, Vector3.down, out RaycastHit hit, 5f, ~ragdollLayer))
            {
                desiredStepPosition = hit.point;
            }

            Vector3 newSupportCenterTarget = desiredStepPosition;

            float leftDist = Vector3.Distance(_leftFootGroundTarget, newSupportCenterTarget);
            float rightDist = Vector3.Distance(_rightFootGroundTarget, newSupportCenterTarget);
            float leftAlignment = Vector3.Dot((_leftFootGroundTarget - supportCenter).normalized, stepDirection);
            float rightAlignment = Vector3.Dot((_rightFootGroundTarget - supportCenter).normalized, stepDirection);
            bool moveLeft = (leftDist > rightDist) || (leftAlignment < rightAlignment && leftDist > minStepDistance);

            Vector3 finalStepTarget;

            if (moveLeft)
            {
                finalStepTarget = newSupportCenterTarget - (hipsTransform.right * footSpacing / 2f);
                Debug.Log($">>> STEPPING LEFT FOOT to {finalStepTarget}");
            }
            else
            {
                finalStepTarget = newSupportCenterTarget + (hipsTransform.right * footSpacing / 2f);
                Debug.Log($">>> STEPPING RIGHT FOOT to {finalStepTarget}");
            }

            if (Physics.Raycast(finalStepTarget + Vector3.up, Vector3.down, out RaycastHit finalHit, 3f, ~ragdollLayer))
            {
                finalStepTarget = finalHit.point;
            }

            if (moveLeft)
            {
                _leftFootGroundTarget = finalStepTarget;
                StartCoroutine(PerformStep(leftFootIKTarget, _leftFootGroundTarget));
            }
            else
            {
                _rightFootGroundTarget = finalStepTarget;
                StartCoroutine(PerformStep(rightFootIKTarget, _rightFootGroundTarget));
            }

            _lastMovementTime = Time.time;
        }
        // FIXED: Only correct feet when truly idle (low velocity)
        else if (IsCharacterIdle())
        {
            HandleIdleFootCorrection();
        }
    }

    // NEW: Check if character is actually idle
    private bool IsCharacterIdle()
    {
        // Get horizontal velocity
        Vector3 horizontalVelocity = _centerOfMassVelocity;
        horizontalVelocity.y = 0;
        
        // Character is idle if velocity is low AND enough time has passed
        bool hasLowVelocity = horizontalVelocity.magnitude < movementVelocityThreshold;
        bool hasBeenStill = Time.time - _lastMovementTime > idleCorrectionDelay;
        
        Debug.Log($"Idle Check: Velocity={horizontalVelocity.magnitude:F3} (threshold={movementVelocityThreshold}) | TimeSinceMove={Time.time - _lastMovementTime:F2}s (delay={idleCorrectionDelay}) | IsIdle={hasLowVelocity && hasBeenStill}");
        
        return hasLowVelocity && hasBeenStill;
    }

    private void HandleIdleFootCorrection()
    {
        Debug.Log("=== IDLE FOOT CORRECTION TRIGGERED ===");
        
        Vector3 idealLeftPos = GetIdealFootPosition(true);
        Vector3 idealRightPos = GetIdealFootPosition(false);

        float leftDrift = Vector3.Distance(_leftFootGroundTarget, idealLeftPos);
        float rightDrift = Vector3.Distance(_rightFootGroundTarget, idealRightPos);

        Debug.Log($"Left Drift: {leftDrift:F3} | Right Drift: {rightDrift:F3} | Threshold: {footCorrectionThreshold:F3}");
        Debug.Log($"Left Target: {_leftFootGroundTarget} -> Ideal: {idealLeftPos}");
        Debug.Log($"Right Target: {_rightFootGroundTarget} -> Ideal: {idealRightPos}");

        if (leftDrift > footCorrectionThreshold || rightDrift > footCorrectionThreshold)
        {
            if (rightDrift > leftDrift)
            {
                Debug.Log($">>> CORRECTING RIGHT FOOT (drift: {rightDrift:F3})");
                _rightFootGroundTarget = idealRightPos;
                StartCoroutine(PerformStep(rightFootIKTarget, _rightFootGroundTarget));
            }
            else
            {
                Debug.Log($">>> CORRECTING LEFT FOOT (drift: {leftDrift:F3})");
                _leftFootGroundTarget = idealLeftPos;
                StartCoroutine(PerformStep(leftFootIKTarget, _leftFootGroundTarget));
            }
        }
        else
        {
            Debug.Log("No correction needed - drift within threshold");
        }
    }

    private Vector3 GetIdealFootPosition(bool isLeftFoot)
    {
        Vector3 rootPos = hipsTransform.position;
        Vector3 offset = hipsTransform.right * (isLeftFoot ? -footSpacing : footSpacing);
        Vector3 idealPos = rootPos + offset;

        if (Physics.Raycast(idealPos + Vector3.up * 2f, Vector3.down, out RaycastHit hit, 5f, ~ragdollLayer))
        {
            return hit.point;
        }
        else
        {
            idealPos.y = rootPos.y - standingHeight;
            return idealPos;
        }
    }

    private IEnumerator PerformStep(Transform foot, Vector3 target)
    {
        _isStepping = true;
        Debug.Log($"[STEP START] {foot.name} from {foot.position} to {target} (distance: {Vector3.Distance(foot.position, target):F2})");

        Vector3 startPoint = foot.position;
        Vector3 centerPoint = (startPoint + target) / 2;
        centerPoint.y = Mathf.Max(startPoint.y, target.y) + stepHeight;

        float timeElapsed = 0;

        while (timeElapsed < stepDuration)
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / stepDuration;

            foot.position = Vector3.Lerp(
                Vector3.Lerp(startPoint, centerPoint, normalizedTime),
                Vector3.Lerp(centerPoint, target, normalizedTime),
                normalizedTime
            );

            yield return null;
        }

        foot.position = target;
        Debug.Log($"[STEP COMPLETE] {foot.name} at {target}");
        _isStepping = false;
    }

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

    public void UpdateStepGuidePosition()
    {
        Vector3 comHorizontal = _centerOfMass;
        comHorizontal.y = hipsTransform.position.y;

        stepGuide.position = Vector3.Lerp(stepGuide.position, comHorizontal, Time.deltaTime * 5f);
        stepGuide.eulerAngles = new Vector3(0, hipsTransform.eulerAngles.y, 0);
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(_centerOfMass, 0.1f);

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(_centerOfMass, _centerOfMass + _centerOfMassVelocity * 0.5f);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(_leftFootGroundTarget, _rightFootGroundTarget);

        Vector3 supportCenter = (_leftFootGroundTarget + _rightFootGroundTarget) / 2f;
        float currentFootSpacing = Vector3.Distance(_leftFootGroundTarget, _rightFootGroundTarget);
        float stabilityRadius = currentFootSpacing * stabilityThreshold;

        Gizmos.color = new Color(0, 1, 0, 0.3f);
        DrawCircle(supportCenter, stabilityRadius, 32);

        Vector3 comGroundProj = _centerOfMass;
        comGroundProj.y = supportCenter.y;
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(_centerOfMass, comGroundProj);
        Gizmos.DrawSphere(comGroundProj, 0.08f);
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
}