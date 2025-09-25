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
    public float footSpacing;
    public float standingHeight;

    [Header("Step Values")]
    public float stepDuration = 0.2f;
    public float stepHeight = 1f;

    [Header("Directional Step Values")]
    [Tooltip("Step distance for forward/backward movement")]
    public float stepThresholdForward = 0.6f;
    [Tooltip("Step distance for left/right movement")]
    public float stepThresholdStrafe = 0.4f;
    [Tooltip("Step prediction distance for forward/backward movement")]
    public float stepPredictionForward = 0.4f;
    [Tooltip("Step prediction distance for left/right movement")]
    public float stepPredictionStrafe = 0.3f;

    [Header("Foot Correction")]
    [Tooltip("How far feet can be from ideal position before correcting")]
    public float footCorrectionThreshold = 0.15f;
    [Tooltip("Time to wait before correcting foot positions when idle")]
    public float idleCorrectionDelay = 1.0f;

    [Header("Physics")]
    public bool useVelocityForMovementDetection = false;
    public int balanceForce = 10;
    public int balanceDamping = 1;
    public LayerMask ragdollLayer;

    // Private State
    private Vector3 _lastHipsPosition;
    private Transform _leftStepRaycastOrigin, _rightStepRaycastOrigin;
    private Vector3 _leftFootGroundTarget, _rightFootGroundTarget;
    private bool _isStepping = false;
    private float _lastMovementTime = 0f;
    private Vector3 _lastHipsRotation;
    
    [HideInInspector]
    public ConfigurableJoint balanceJoint;
    [HideInInspector]
    public Rigidbody balanceTargetBody;

    private Rigidbody _hipsRigidbody;

    [HideInInspector]
    public bool isAirborne;

    void Awake()
    {
        _hipsRigidbody = hipsTransform.GetComponent<Rigidbody>();

        balanceTargetBody = new GameObject("Balance Target Body").AddComponent<Rigidbody>();
        balanceTargetBody.transform.parent = transform;
        balanceTargetBody.isKinematic = true;
        SetupBalanceJoint();
    }

    void Start()
    {
        stepGuide.position = hipsTransform.position;
        _lastHipsPosition = hipsTransform.position;
        _lastHipsRotation = hipsTransform.eulerAngles;

        _leftStepRaycastOrigin = new GameObject("Left Step Raycast Origin").transform;
        _rightStepRaycastOrigin = new GameObject("Right Step Raycast Origin").transform;

        _leftStepRaycastOrigin.parent = stepGuide;
        _rightStepRaycastOrigin.parent = stepGuide;

        _leftStepRaycastOrigin.localPosition = new Vector3(-footSpacing, 0, 0);
        _rightStepRaycastOrigin.localPosition = new Vector3(footSpacing, 0, 0);

        // Initialize targets
        Physics.Raycast(_leftStepRaycastOrigin.position, Vector3.down, out RaycastHit hitL, 5, ~ragdollLayer);
        Physics.Raycast(_rightStepRaycastOrigin.position, Vector3.down, out RaycastHit hitR, 5, ~ragdollLayer);
        _leftFootGroundTarget = hitL.point;
        _rightFootGroundTarget = hitR.point;
    }

    void Update()
    {
        UpdateStepGuidePosition();

        Physics.Raycast(_leftStepRaycastOrigin.position, Vector3.down, out RaycastHit hitL, 5, ~ragdollLayer);
        Physics.Raycast(_rightStepRaycastOrigin.position, Vector3.down, out RaycastHit hitR, 5, ~ragdollLayer);

        if (hitL.collider == null || hitR.collider == null)
        {
            isAirborne = true;
        }
        else
        {
            isAirborne = false;
        }

        bool isMoving = IsCharacterMoving();
        bool isRotating = IsCharacterRotating();
        
        if (isMoving || isRotating)
        {
            _lastMovementTime = Time.time;
        }

        if (!_isStepping)
        {
            HandleMovementStepping(hitL, hitR);
            
            if (!isMoving && (isRotating || Time.time - _lastMovementTime > idleCorrectionDelay))
            {
                HandleIdleFootCorrection(hitL, hitR);
            }
        }

        leftFootIKTarget.eulerAngles = new Vector3(leftFootIKTarget.eulerAngles.x, hipsTransform.eulerAngles.y, leftFootIKTarget.eulerAngles.z);
        rightFootIKTarget.eulerAngles = new Vector3(rightFootIKTarget.eulerAngles.x, hipsTransform.eulerAngles.y, rightFootIKTarget.eulerAngles.z);

        stepGuide.eulerAngles = new Vector3(stepGuide.eulerAngles.x, hipsTransform.eulerAngles.y, stepGuide.eulerAngles.z);

        if (!isAirborne)
        {
            balanceTargetBody.transform.position = new Vector3(
                hipsTransform.position.x,
                (hitL.point.y + hitR.point.y) / 2 + standingHeight,
                hipsTransform.position.z
            );
        }

        _lastHipsRotation = hipsTransform.eulerAngles;
    }

    private bool IsCharacterMoving()
    {
        if (useVelocityForMovementDetection)
        {
            return _hipsRigidbody.velocity.magnitude > 0.1f;
        }
        else
        {
            return Vector3.Distance(_lastHipsPosition, hipsTransform.position) > 0.005f;
        }
    }

    private bool IsCharacterRotating()
    {
        float rotationDiff = Mathf.Abs(Mathf.DeltaAngle(_lastHipsRotation.y, hipsTransform.eulerAngles.y));
        return rotationDiff > 2f;
    }

    private void HandleMovementStepping(RaycastHit hitL, RaycastHit hitR)
    {
        float leftDist = Vector3.Distance(hitL.point, _leftFootGroundTarget);
        float rightDist = Vector3.Distance(hitR.point, _rightFootGroundTarget);

        Vector3 movementDir = GetMovementDirection();
        float currentStepThreshold = GetCurrentStepThreshold(movementDir);

        if (leftDist >= currentStepThreshold || rightDist >= currentStepThreshold)
        {
            if (rightDist > leftDist)
            {
                _rightFootGroundTarget = hitR.point;
                StartCoroutine(PerformStep(rightFootIKTarget, _rightFootGroundTarget));
            }
            else
            {
                _leftFootGroundTarget = hitL.point;
                StartCoroutine(PerformStep(leftFootIKTarget, _leftFootGroundTarget));
            }
        }
    }

    private void HandleIdleFootCorrection(RaycastHit hitL, RaycastHit hitR)
    {
        Vector3 idealLeftPos = GetIdealFootPosition(true);
        Vector3 idealRightPos = GetIdealFootPosition(false);
        
        float leftDrift = Vector3.Distance(leftFootIKTarget.position, idealLeftPos);
        float rightDrift = Vector3.Distance(rightFootIKTarget.position, idealRightPos);

        if (leftDrift > footCorrectionThreshold || rightDrift > footCorrectionThreshold)
        {
            if (rightDrift > leftDrift)
            {
                _rightFootGroundTarget = hitR.point;
                StartCoroutine(PerformStep(rightFootIKTarget, _rightFootGroundTarget));
            }
            else
            {
                _leftFootGroundTarget = hitL.point;
                StartCoroutine(PerformStep(leftFootIKTarget, _leftFootGroundTarget));
            }
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

    private Vector3 GetMovementDirection()
    {
        if (useVelocityForMovementDetection)
        {
            Vector3 vel = _hipsRigidbody.velocity;
            vel.y = 0;
            return vel.normalized;
        }
        else
        {
            Vector3 dir = hipsTransform.position - _lastHipsPosition;
            dir.y = 0;
            return dir.normalized;
        }
    }

    private float GetCurrentStepThreshold(Vector3 movementDir)
    {
        if (movementDir.magnitude < 0.1f) return stepThresholdForward;

        float forwardDot = Mathf.Abs(Vector3.Dot(movementDir, hipsTransform.forward));
        float rightDot = Mathf.Abs(Vector3.Dot(movementDir, hipsTransform.right));
        
        float forwardInfluence = forwardDot / (forwardDot + rightDot);
        return Mathf.Lerp(stepThresholdStrafe, stepThresholdForward, forwardInfluence);
    }

    private float GetCurrentStepPrediction(Vector3 movementDir)
    {
        if (movementDir.magnitude < 0.1f) return stepPredictionForward;

        float forwardDot = Mathf.Abs(Vector3.Dot(movementDir, hipsTransform.forward));
        float rightDot = Mathf.Abs(Vector3.Dot(movementDir, hipsTransform.right));
        
        float forwardInfluence = forwardDot / (forwardDot + rightDot);
        return Mathf.Lerp(stepPredictionStrafe, stepPredictionForward, forwardInfluence);
    }

    private IEnumerator PerformStep(Transform foot, Vector3 target)
    {
        _isStepping = true;

        Vector3 startPoint = foot.position;
        Vector3 centerPoint = (startPoint + target) / 2;
        centerPoint.y = target.y + stepHeight;

        float timeElapsed = 0;

        do
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
        while (timeElapsed < stepDuration);

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
        Vector3 movementDir;
        float currentStepPrediction = stepPredictionForward; // Default
        
        if (useVelocityForMovementDetection)
        {
            if (_hipsRigidbody.velocity.magnitude > 0.3f)
            {
                Vector3 dir = _hipsRigidbody.velocity;
                dir.Normalize();
                dir.y = 0;
                movementDir = dir;
                currentStepPrediction = GetCurrentStepPrediction(movementDir);
                stepGuide.position = hipsTransform.position + dir * currentStepPrediction;
            }
            else
            {
                stepGuide.position = hipsTransform.position;
            }
        }
        else
        {
            if (Vector3.Distance(_lastHipsPosition, hipsTransform.position) > 0.005f)
            {
                Vector3 dir = hipsTransform.position - _lastHipsPosition;
                dir.Normalize();
                dir.y = 0;
                movementDir = dir;
                currentStepPrediction = GetCurrentStepPrediction(movementDir);
                stepGuide.position = hipsTransform.position + dir * currentStepPrediction;
            }
            else
            {
                stepGuide.position = hipsTransform.position;
            }

            _lastHipsPosition = hipsTransform.position;
        }
    }
}