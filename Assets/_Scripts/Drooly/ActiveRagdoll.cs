using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActiveRagdoll : MonoBehaviour
{
    [Header("Core References")]
    public Transform rootBone;
    public Transform moveDir;

    [Header("IK Targets")]
    public Transform leftFoot;
    public Transform rightFoot;

    [Header("Standing Position Values")]
    public float footGap;
    public float standHeight;

    [Header("Step Values")]
    public float stepDuration = 0.2f;
    public float stepHeight = 1f;

    [Header("Directional Step Values")]
    [Tooltip("Step distance for forward/backward movement")]
    public float stepDistForward = 0.6f;
    [Tooltip("Step distance for left/right movement")]  
    public float stepDistStrafe = 0.4f;
    [Tooltip("Step size for forward/backward movement")]
    public float stepSizeForward = 0.4f;
    [Tooltip("Step size for left/right movement")]
    public float stepSizeStrafe = 0.3f;

    [Header("Foot Correction")]
    [Tooltip("How far feet can be from ideal position before correcting")]
    public float footCorrectionThreshold = 0.15f;
    [Tooltip("Time to wait before correcting foot positions when idle")]
    public float idleCorrectionDelay = 1.0f;

    [Header("Physics")]
    public bool physics = false;
    public int rotationForce = 10;
    public int rotationSmoothness = 1;
    public LayerMask ragdollLayer;

    private Vector3 _lastFrame;
    private Transform _leftSide, _rightSide;
    private Vector3 _leftTarget, _rightTarget;
    private bool _isCurrentlyStepping = false;
    private float _lastMovementTime = 0f;
    private Vector3 _lastRootRotation;

    [HideInInspector]
    public ConfigurableJoint joint;
    [HideInInspector]
    public Rigidbody connectedBody;

    private Rigidbody _rb;

    [HideInInspector]
    public bool falling;

    void Awake()
    {
        _rb = rootBone.GetComponent<Rigidbody>();

        connectedBody = new GameObject("joint").AddComponent<Rigidbody>();
        connectedBody.transform.parent = transform;
        connectedBody.isKinematic = true;
        setupJoint();
    }

    void Start()
    {
        moveDir.position = rootBone.position;
        _lastFrame = rootBone.position;
        _lastRootRotation = rootBone.eulerAngles;

        _leftSide = new GameObject("left side").transform;
        _rightSide = new GameObject("right side").transform;

        _leftSide.parent = moveDir;
        _rightSide.parent = moveDir;

        _leftSide.localPosition = new Vector3(-footGap, 0, 0);
        _rightSide.localPosition = new Vector3(footGap, 0, 0);

        // Initialize targets
        Physics.Raycast(_leftSide.position, Vector3.down, out RaycastHit hitL, 5, ~ragdollLayer);
        Physics.Raycast(_rightSide.position, Vector3.down, out RaycastHit hitR, 5, ~ragdollLayer);
        _leftTarget = hitL.point;
        _rightTarget = hitR.point;
    }

    void Update()
    {
        setMoveDir();

        Physics.Raycast(_leftSide.position, Vector3.down, out RaycastHit hitL, 5, ~ragdollLayer);
        Physics.Raycast(_rightSide.position, Vector3.down, out RaycastHit hitR, 5, ~ragdollLayer);

        if (hitL.collider == null || hitR.collider == null)
        {
            falling = true;
        }
        else
        {
            falling = false;
        }

        // Check if we're moving or rotating
        bool isMoving = IsCharacterMoving();
        bool isRotating = IsCharacterRotating();
        
        if (isMoving || isRotating)
        {
            _lastMovementTime = Time.time;
        }

        if (!_isCurrentlyStepping)
        {
            // Regular stepping with directional step distances
            HandleRegularStepping(hitL, hitR);

            // Foot correction when idle or rotating
            if (!isMoving && (isRotating || Time.time - _lastMovementTime > idleCorrectionDelay))
            {
                HandleFootCorrection(hitL, hitR);
            }
        }

        // Keep feet rotation synchronized with torso
        leftFoot.eulerAngles = new Vector3(leftFoot.eulerAngles.x, rootBone.eulerAngles.y, leftFoot.eulerAngles.z);
        rightFoot.eulerAngles = new Vector3(rightFoot.eulerAngles.x, rootBone.eulerAngles.y, rightFoot.eulerAngles.z);

        moveDir.eulerAngles = new Vector3(moveDir.eulerAngles.x, rootBone.eulerAngles.y, moveDir.eulerAngles.z);

        if (!falling)
        {
            connectedBody.transform.position = new Vector3(
                rootBone.position.x,
                (hitL.point.y + hitR.point.y) / 2 + standHeight,
                rootBone.position.z
            );
        }

        _lastRootRotation = rootBone.eulerAngles;
    }

    private bool IsCharacterMoving()
    {
        if (physics)
        {
            return _rb.velocity.magnitude > 0.1f;
        }
        else
        {
            return Vector3.Distance(_lastFrame, rootBone.position) > 0.005f;
        }
    }

    private bool IsCharacterRotating()
    {
        float rotationDiff = Mathf.Abs(Mathf.DeltaAngle(_lastRootRotation.y, rootBone.eulerAngles.y));
        return rotationDiff > 2f; // 2 degrees per frame threshold
    }

    private void HandleRegularStepping(RaycastHit hitL, RaycastHit hitR)
    {
        float leftDist = Vector3.Distance(hitL.point, _leftTarget);
        float rightDist = Vector3.Distance(hitR.point, _rightTarget);

        // Get movement direction to determine step distance and size
        Vector3 movementDir = GetMovementDirection();
        float currentStepDist = GetStepDistanceForDirection(movementDir);

        if (leftDist >= currentStepDist || rightDist >= currentStepDist)
        {
            if (rightDist > leftDist)
            {
                _rightTarget = hitR.point;
                StartCoroutine(step(rightFoot, _rightTarget));
            }
            else
            {
                _leftTarget = hitL.point;
                StartCoroutine(step(leftFoot, _leftTarget));
            }
        }
    }

    private void HandleFootCorrection(RaycastHit hitL, RaycastHit hitR)
    {
        // Calculate ideal foot positions based on current stance
        Vector3 idealLeftPos = GetIdealFootPosition(true);
        Vector3 idealRightPos = GetIdealFootPosition(false);

        // Check if feet are too far from ideal positions
        float leftDrift = Vector3.Distance(leftFoot.position, idealLeftPos);
        float rightDrift = Vector3.Distance(rightFoot.position, idealRightPos);

        if (leftDrift > footCorrectionThreshold || rightDrift > footCorrectionThreshold)
        {
            // Step the foot that's furthest from its ideal position
            if (rightDrift > leftDrift)
            {
                _rightTarget = hitR.point;
                StartCoroutine(step(rightFoot, _rightTarget));
            }
            else
            {
                _leftTarget = hitL.point;
                StartCoroutine(step(leftFoot, _leftTarget));
            }
        }
    }

    private Vector3 GetIdealFootPosition(bool isLeftFoot)
    {
        Vector3 rootPos = rootBone.position;
        Vector3 offset = rootBone.right * (isLeftFoot ? -footGap : footGap);
        Vector3 idealPos = rootPos + offset;

        // Project to ground
        if (Physics.Raycast(idealPos + Vector3.up * 2f, Vector3.down, out RaycastHit hit, 5f, ~ragdollLayer))
        {
            return hit.point;
        }
        else
        {
            idealPos.y = rootPos.y - standHeight;
            return idealPos;
        }
    }

    private Vector3 GetMovementDirection()
    {
        if (physics)
        {
            Vector3 vel = _rb.velocity;
            vel.y = 0;
            return vel.normalized;
        }
        else
        {
            Vector3 dir = rootBone.position - _lastFrame;
            dir.y = 0;
            return dir.normalized;
        }
    }

    private float GetStepDistanceForDirection(Vector3 movementDir)
    {
        if (movementDir.magnitude < 0.1f) return stepDistForward; // Default to forward when not moving

        // Calculate dot products to determine direction
        float forwardDot = Mathf.Abs(Vector3.Dot(movementDir, rootBone.forward));
        float rightDot = Mathf.Abs(Vector3.Dot(movementDir, rootBone.right));

        // Blend between forward and strafe distances based on movement direction
        float forwardInfluence = forwardDot / (forwardDot + rightDot);
        return Mathf.Lerp(stepDistStrafe, stepDistForward, forwardInfluence);
    }

    private float GetStepSizeForDirection(Vector3 movementDir)
    {
        if (movementDir.magnitude < 0.1f) return stepSizeForward; // Default to forward when not moving

        // Calculate dot products to determine direction
        float forwardDot = Mathf.Abs(Vector3.Dot(movementDir, rootBone.forward));
        float rightDot = Mathf.Abs(Vector3.Dot(movementDir, rootBone.right));

        // Blend between forward and strafe sizes based on movement direction
        float forwardInfluence = forwardDot / (forwardDot + rightDot);
        return Mathf.Lerp(stepSizeStrafe, stepSizeForward, forwardInfluence);
    }

    private IEnumerator step(Transform foot, Vector3 target)
    {
        _isCurrentlyStepping = true;

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

        _isCurrentlyStepping = false;
    }

    public void setupJoint()
    {
        if (joint != null)
            Destroy(joint);

        joint = rootBone.gameObject.AddComponent<ConfigurableJoint>();

        JointDrive driveX = joint.angularXDrive;
        JointDrive driveYZ = joint.angularYZDrive;
        JointDrive driveY = joint.yDrive;

        driveX.positionSpring = rotationForce;
        driveX.positionDamper = rotationSmoothness;

        driveYZ.positionSpring = rotationForce;
        driveYZ.positionDamper = rotationSmoothness;

        driveY.positionSpring = 300;
        driveY.positionDamper = 10;

        joint.angularXDrive = driveX;
        joint.angularYZDrive = driveYZ;
        joint.yDrive = driveY;

        connectedBody.transform.position = rootBone.position;
        connectedBody.transform.rotation = rootBone.rotation;
        joint.connectedBody = connectedBody;
        connectedBody.transform.rotation = Quaternion.identity;
    }

    public void setMoveDir()
    {
        Vector3 movementDir = Vector3.zero;
        float currentStepSize = stepSizeForward; // Default
        
        if (physics)
        {
            if (_rb.velocity.magnitude > 0.3f)
            {
                Vector3 dir = _rb.velocity;
                dir.Normalize();
                dir.y = 0;
                movementDir = dir;
                currentStepSize = GetStepSizeForDirection(movementDir);
                moveDir.position = rootBone.position + dir * currentStepSize;
            }
            else
            {
                moveDir.position = rootBone.position;
            }
        }
        else
        {
            if (Vector3.Distance(_lastFrame, rootBone.position) > 0.005f)
            {
                Vector3 dir = rootBone.position - _lastFrame;
                dir.Normalize();
                dir.y = 0;
                movementDir = dir;
                currentStepSize = GetStepSizeForDirection(movementDir);
                moveDir.position = rootBone.position + dir * currentStepSize;
            }
            else
            {
                moveDir.position = rootBone.position;
            }

            _lastFrame = rootBone.position;
        }
    }
}