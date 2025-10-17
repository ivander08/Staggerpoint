using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;

public class ArmController : MonoBehaviour
{
    [Header("Core Components")]
    public ActiveRagdoll activeRagdoll;
    public CameraController cameraController;

    // --- RIGHT ARM ---
    [Header("Right Arm Rig")]
    public Rigidbody rightHandRigidbody;
    public Transform rightShoulderAnchor;
    public Transform rightArmRoot;

    // --- LEFT ARM ---
    [Header("Left Arm Rig")]
    public Rigidbody leftHandRigidbody;
    public Transform leftShoulderAnchor;
    public Transform leftArmRoot;

    [Header("IK Targets")]
    public Transform rightHandIKTarget;
    public Transform leftHandIKTarget;

    [Header("Finger Bones")]
    public ConfigurableJoint rightFingerJoint;
    public ConfigurableJoint leftFingerJoint;

    [Header("Physics Control")]
    public float followForce = 50000f;
    public float rotateTorque = 5000f;

    [Header("Finger Control")]
    [Tooltip("Target rotation for right hand closed fist (in degrees)")]
    public float rightFistCurlAngle = -120f;
    [Tooltip("Target rotation for left hand closed fist (in degrees)")]
    public float leftFistCurlAngle = 120f;
    [Tooltip("Spring force for finger joints")]
    public float fingerSpring = 100f;
    [Tooltip("Damper for finger joints")]
    public float fingerDamper = 5f;

    [Header("Hook Punch (Bent Elbow)")]
    [Tooltip("Distance from shoulder at starting side (right for right arm, left for left arm)")]
    public float hookRadiusStart = 0.5f;
    [Tooltip("Distance from shoulder at middle/forward position")]
    public float hookRadiusMiddle = 0.7f;
    [Tooltip("Distance from shoulder at ending side (left for right arm, right for left arm)")]
    public float hookRadiusEnd = 0.5f;

    [Header("Jab Punch (Extended Arm)")]
    [Tooltip("Distance from shoulder at starting side")]
    public float jabRadiusStart = 0.9f;
    [Tooltip("Distance from shoulder at middle/forward position")]
    public float jabRadiusMiddle = 1.2f;
    [Tooltip("Distance from shoulder at ending side")]
    public float jabRadiusEnd = 1.0f;

    [Header("Weapon Hold Mode")]
    public float weaponRadiusStart = 0.6f;
    public float weaponRadiusMiddle = 0.8f;
    public float weaponRadiusEnd = 0.6f;

    [Header("Weapon References")]
    public WeaponPickupManager weaponPickupManager;

    [Header("Swing Control")]
    public float swingSensitivity = 1.5f;
    public float swingAngleAcrossBody = 45f;
    public float swingAngleOutward = 90f;
    public float maxVerticalSwingAngle = 80f;

    [Header("Debug Visualization")]
    public bool showPunchPaths = true;
    public int pathResolution = 32;

    // --- Private State ---
    private bool _isRightArmSwinging = false;
    private bool _isLeftArmSwinging = false;
    private bool _isJabMode = false;
    private float _originalBalanceForce, _originalBalanceDamper;
    private Transform _cameraTransform;

    private float _currentRightSwingYaw, _currentRightSwingPitch;
    private float _currentLeftSwingYaw, _currentLeftSwingPitch;

    private List<ConfigurableJoint> _rightArmJoints = new List<ConfigurableJoint>();
    private List<JointDrive> _originalRightArmDrives = new List<JointDrive>();
    private List<ConfigurableJoint> _leftArmJoints = new List<ConfigurableJoint>();
    private List<JointDrive> _originalLeftArmDrives = new List<JointDrive>();

    private Quaternion _rightFingerStartRotation;
    private Quaternion _leftFingerStartRotation;

    void Awake()
    {
        if (cameraController != null) _cameraTransform = cameraController.transform;
        else { Debug.LogError("CameraController not assigned!"); enabled = false; return; }

        rightHandRigidbody.maxAngularVelocity = 50f;
        leftHandRigidbody.maxAngularVelocity = 50f;

        InitializeArm(rightArmRoot, _rightArmJoints, _originalRightArmDrives);
        InitializeArm(leftArmRoot, _leftArmJoints, _originalLeftArmDrives);

        _originalBalanceForce = activeRagdoll.balanceForce;
        _originalBalanceDamper = activeRagdoll.balanceDamping;

        // Store initial finger rotations
        if (rightFingerJoint != null)
            _rightFingerStartRotation = rightFingerJoint.transform.localRotation;
        if (leftFingerJoint != null)
            _leftFingerStartRotation = leftFingerJoint.transform.localRotation;
    }

    private void InitializeArm(Transform armRoot, List<ConfigurableJoint> joints, List<JointDrive> drives)
    {
        if (armRoot != null)
        {
            joints.AddRange(armRoot.GetComponentsInChildren<ConfigurableJoint>());
            foreach (var joint in joints)
            {
                drives.Add(joint.angularXDrive);
            }
        }
    }

    void Update()
    {
        HandleInput();
        if (_isRightArmSwinging) UpdateIKTargetPosition(rightHandIKTarget, rightShoulderAnchor, ref _currentRightSwingYaw, ref _currentRightSwingPitch, false);
        if (_isLeftArmSwinging) UpdateIKTargetPosition(leftHandIKTarget, leftShoulderAnchor, ref _currentLeftSwingYaw, ref _currentLeftSwingPitch, true);

        // Update finger curls - grip when holding weapon OR when swinging
        bool rightShouldCurl = _isRightArmSwinging || IsArmHoldingWeapon(true);
        bool leftShouldCurl = _isLeftArmSwinging || IsArmHoldingWeapon(false);

        UpdateFingerCurl(rightFingerJoint, rightShouldCurl, _rightFingerStartRotation, rightFistCurlAngle);
        UpdateFingerCurl(leftFingerJoint, leftShouldCurl, _leftFingerStartRotation, leftFistCurlAngle);
    }

    void FixedUpdate()
    {
        if (_isRightArmSwinging) MoveArmTowardsTarget(rightHandRigidbody, rightHandIKTarget);
        if (_isLeftArmSwinging) MoveArmTowardsTarget(leftHandRigidbody, leftHandIKTarget);
    }

    // --- Input & State Management ---

    private void HandleInput()
    {
        // Only allow jab mode if NOT holding a weapon on either arm
        bool holdingWeapon = IsArmHoldingWeapon(true) || IsArmHoldingWeapon(false);
        if (!holdingWeapon)
        {
            _isJabMode = Keyboard.current.leftAltKey.isPressed || Keyboard.current.rightAltKey.isPressed;
        }
        else
        {
            _isJabMode = false; // Force hook mode when holding weapon
        }

        bool rightMousePressed = Mouse.current.rightButton.isPressed;
        if (rightMousePressed && !_isRightArmSwinging) StartSwing(true);
        else if (!rightMousePressed && _isRightArmSwinging) EndSwing(true);

        bool leftMousePressed = Mouse.current.leftButton.isPressed;
        if (leftMousePressed && !_isLeftArmSwinging) StartSwing(false);
        else if (!leftMousePressed && _isLeftArmSwinging) EndSwing(false);
    }

    private bool IsArmHoldingWeapon(bool isRightArm)
    {
        if (weaponPickupManager == null) return false;

        if (isRightArm)
            return weaponPickupManager.equippedWeaponRight != null;
        else
            return weaponPickupManager.equippedWeaponLeft != null;
    }

    private void StartSwing(bool isRightArm)
    {
        if (!_isRightArmSwinging && !_isLeftArmSwinging)
        {
            cameraController.SetLock(true);
            BraceTorso();
        }

        if (isRightArm)
        {
            _isRightArmSwinging = true;
            RelaxArm(_rightArmJoints);
            rightHandIKTarget.position = rightHandRigidbody.position;
            InitializeSwingAngles(rightHandRigidbody, rightShoulderAnchor, ref _currentRightSwingYaw, ref _currentRightSwingPitch);
        }
        else
        {
            _isLeftArmSwinging = true;
            RelaxArm(_leftArmJoints);
            leftHandIKTarget.position = leftHandRigidbody.position;
            InitializeSwingAngles(leftHandRigidbody, leftShoulderAnchor, ref _currentLeftSwingYaw, ref _currentLeftSwingPitch);
        }
    }

    private void EndSwing(bool isRightArm)
    {
        if (isRightArm)
        {
            _isRightArmSwinging = false;
            ReTenseArm(_rightArmJoints, _originalRightArmDrives);
        }
        else
        {
            _isLeftArmSwinging = false;
            ReTenseArm(_leftArmJoints, _originalLeftArmDrives);
        }

        if (!_isRightArmSwinging && !_isLeftArmSwinging)
        {
            cameraController.SetLock(false);
            UnbraceTorso();
        }
    }

    // --- Helper Functions ---

    private void BraceTorso()
    {
        activeRagdoll.balanceForce = (int)(_originalBalanceForce * 2);
        activeRagdoll.balanceDamping = (int)(_originalBalanceDamper * 5);
        activeRagdoll.SetupBalanceJoint();
    }

    private void UnbraceTorso()
    {
        activeRagdoll.balanceForce = (int)_originalBalanceForce;
        activeRagdoll.balanceDamping = (int)_originalBalanceDamper;
        activeRagdoll.SetupBalanceJoint();
    }

    private void RelaxArm(List<ConfigurableJoint> joints)
    {
        foreach (var joint in joints)
        {
            var relaxedDrive = new JointDrive { positionSpring = 0, positionDamper = 10, maximumForce = float.MaxValue };
            joint.angularXDrive = relaxedDrive;
            joint.angularYZDrive = relaxedDrive;
        }
    }


    private void ReTenseArm(List<ConfigurableJoint> joints, List<JointDrive> originalDrives)
    {
        for (int i = 0; i < joints.Count; i++)
        {
            joints[i].angularXDrive = originalDrives[i];
            joints[i].angularYZDrive = originalDrives[i];
        }
    }

    // --- Core Swing Logic ---

    private void InitializeSwingAngles(Rigidbody handRB, Transform shoulderAnchor, ref float yaw, ref float pitch)
    {
        Transform hips = activeRagdoll.hipsTransform;
        Vector3 charUp = hips.up;
        Vector3 currentArmVector = handRB.position - shoulderAnchor.position;
        Vector3 armHorizontal = Vector3.ProjectOnPlane(currentArmVector, charUp);

        yaw = Vector3.SignedAngle(Vector3.ProjectOnPlane(hips.forward, charUp), armHorizontal, charUp);
        pitch = Vector3.Angle(armHorizontal, currentArmVector) * Mathf.Sign(Vector3.Dot(currentArmVector, charUp));
    }

    private void UpdateIKTargetPosition(Transform handIKTarget, Transform shoulderAnchor, ref float yaw, ref float pitch, bool isLeftArm)
    {
        Transform hips = activeRagdoll.hipsTransform;
        Vector2 mouseDelta = Mouse.current.delta.ReadValue();

        yaw += mouseDelta.x * swingSensitivity * 0.1f;
        pitch += mouseDelta.y * swingSensitivity * 0.1f;

        if (isLeftArm)
        {
            yaw = Mathf.Clamp(yaw, -swingAngleOutward, swingAngleAcrossBody);
        }
        else
        {
            yaw = Mathf.Clamp(yaw, -swingAngleAcrossBody, swingAngleOutward);
        }
        pitch = Mathf.Clamp(pitch, -maxVerticalSwingAngle, maxVerticalSwingAngle);

        // Calculate the radius at this yaw angle
        float radius = CalculateRadiusAtAngle(yaw, isLeftArm);

        // Calculate direction based on yaw and pitch
        Vector3 charUp = hips.up;
        Vector3 newHorizontalDir = Quaternion.AngleAxis(yaw, charUp) * hips.forward;
        Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
        Vector3 newArmVector = Quaternion.AngleAxis(pitch, pitchRotationAxis) * newHorizontalDir;

        // Apply the calculated radius
        handIKTarget.position = shoulderAnchor.position + newArmVector.normalized * radius;
        handIKTarget.rotation = Quaternion.LookRotation(_cameraTransform.forward, charUp);
    }

    private float CalculateRadiusAtAngle(float yaw, bool isLeftArm)
    {
        // Check if this arm is holding a weapon
        bool holdingWeapon = IsArmHoldingWeapon(!isLeftArm); // Note: isLeftArm in param is opposite of which arm

        // Get punch style parameters
        float radiusStart, radiusMiddle, radiusEnd;

        if (holdingWeapon)
        {
            // Use weapon punch radii
            radiusStart = weaponRadiusStart;
            radiusMiddle = weaponRadiusMiddle;
            radiusEnd = weaponRadiusEnd;
        }
        else
        {
            // Use hook/jab radii
            radiusStart = _isJabMode ? jabRadiusStart : hookRadiusStart;
            radiusMiddle = _isJabMode ? jabRadiusMiddle : hookRadiusMiddle;
            radiusEnd = _isJabMode ? jabRadiusEnd : hookRadiusEnd;
        }

        // Determine angle range
        float minYaw = isLeftArm ? -swingAngleOutward : -swingAngleAcrossBody;
        float maxYaw = isLeftArm ? swingAngleAcrossBody : swingAngleOutward;

        // Normalize yaw to 0-1 range
        float normalizedYaw = Mathf.InverseLerp(minYaw, maxYaw, yaw);

        // Interpolate radius using smooth curve
        float radius;
        if (normalizedYaw < 0.5f)
        {
            float t = normalizedYaw * 2f;
            radius = Mathf.Lerp(radiusStart, radiusMiddle, t);
        }
        else
        {
            float t = (normalizedYaw - 0.5f) * 2f;
            radius = Mathf.Lerp(radiusMiddle, radiusEnd, t);
        }

        return radius;
    }

    private void MoveArmTowardsTarget(Rigidbody handRB, Transform handIKTarget)
    {
        Vector3 positionDifference = handIKTarget.position - handRB.position;
        handRB.AddForce(positionDifference * followForce * Time.fixedDeltaTime, ForceMode.Force);

        Quaternion rotationDifference = handIKTarget.rotation * Quaternion.Inverse(handRB.rotation);
        rotationDifference.ToAngleAxis(out float angleInDegrees, out Vector3 rotationAxis);
        if (angleInDegrees > 180f) angleInDegrees -= 360f;

        Vector3 torque = rotationAxis.normalized * (angleInDegrees * Mathf.Deg2Rad * rotateTorque);
        handRB.AddTorque(torque * Time.fixedDeltaTime, ForceMode.Force);
    }

    // --- Finger Control ---

    private void UpdateFingerCurl(ConfigurableJoint fingerJoint, bool shouldCurl, Quaternion startRotation, float curlAngle)
    {
        if (fingerJoint == null) return;

        // Set up the drive
        JointDrive drive = new JointDrive
        {
            positionSpring = fingerSpring,
            positionDamper = fingerDamper,
            maximumForce = float.MaxValue
        };

        fingerJoint.angularXDrive = drive;
        fingerJoint.angularYZDrive = drive;

        if (shouldCurl)
        {
            // Curl into fist - rotate around X axis (typically)
            Quaternion targetRotation = startRotation * Quaternion.Euler(curlAngle, 0, 0);
            fingerJoint.targetRotation = Quaternion.Inverse(targetRotation) * startRotation;
        }
        else
        {
            // Open hand - return to start rotation
            fingerJoint.targetRotation = Quaternion.identity;
        }
    }

    // --- Debug Visualization ---

    void OnDrawGizmos()
    {
        if (!showPunchPaths || !Application.isPlaying) return;
        if (activeRagdoll == null) return;

        // Draw both hook and jab paths for both arms
        if (rightShoulderAnchor != null)
        {
            DrawPunchPath(rightShoulderAnchor, false, false);
            DrawPunchPath(rightShoulderAnchor, false, true);
        }

        if (leftShoulderAnchor != null)
        {
            DrawPunchPath(leftShoulderAnchor, true, false);
            DrawPunchPath(leftShoulderAnchor, true, true);
        }

        // Highlight active hand position
        if (_isRightArmSwinging && rightHandIKTarget != null)
        {
            Gizmos.color = _isJabMode ? Color.yellow : Color.red;
            Gizmos.DrawWireSphere(rightHandIKTarget.position, 0.08f);
            Gizmos.DrawLine(rightShoulderAnchor.position, rightHandIKTarget.position);
        }

        if (_isLeftArmSwinging && leftHandIKTarget != null)
        {
            Gizmos.color = _isJabMode ? Color.cyan : Color.blue;
            Gizmos.DrawWireSphere(leftHandIKTarget.position, 0.08f);
            Gizmos.DrawLine(leftShoulderAnchor.position, leftHandIKTarget.position);
        }
    }

    private void DrawPunchPath(Transform shoulderAnchor, bool isLeftArm, bool isJab)
    {
        Transform hips = activeRagdoll.hipsTransform;
        Vector3 charUp = hips.up;

        // Get punch style parameters
        float radiusStart = isJab ? jabRadiusStart : hookRadiusStart;
        float radiusMiddle = isJab ? jabRadiusMiddle : hookRadiusMiddle;
        float radiusEnd = isJab ? jabRadiusEnd : hookRadiusEnd;

        // Choose color
        Color pathColor;
        if (isLeftArm)
            pathColor = isJab ? new Color(0.3f, 0.9f, 0.9f, 0.5f) : new Color(0.3f, 0.5f, 1f, 0.5f);
        else
            pathColor = isJab ? new Color(0.9f, 0.9f, 0.3f, 0.5f) : new Color(1f, 0.5f, 0.3f, 0.5f);

        Gizmos.color = pathColor;

        // Calculate angle range
        float minYaw = isLeftArm ? -swingAngleOutward : -swingAngleAcrossBody;
        float maxYaw = isLeftArm ? swingAngleAcrossBody : swingAngleOutward;

        Vector3 previousPoint = Vector3.zero;
        bool firstPoint = true;

        // Draw the path at neutral pitch
        for (int i = 0; i <= pathResolution; i++)
        {
            float t = (float)i / pathResolution;
            float yaw = Mathf.Lerp(minYaw, maxYaw, t);

            // Calculate radius at this angle
            float radius;
            if (t < 0.5f)
            {
                float halfT = t * 2f;
                radius = Mathf.Lerp(radiusStart, radiusMiddle, halfT);
            }
            else
            {
                float halfT = (t - 0.5f) * 2f;
                radius = Mathf.Lerp(radiusMiddle, radiusEnd, halfT);
            }

            // Calculate position
            Vector3 direction = Quaternion.AngleAxis(yaw, charUp) * hips.forward;
            Vector3 point = shoulderAnchor.position + direction.normalized * radius;

            if (!firstPoint)
            {
                Gizmos.DrawLine(previousPoint, point);
            }

            previousPoint = point;
            firstPoint = false;

            // Draw markers at key points
            if (i == 0 || i == pathResolution / 2 || i == pathResolution)
            {
                Gizmos.DrawWireSphere(point, 0.04f);
            }
        }

        // Draw vertical range indicator at middle position
        float midYaw = (minYaw + maxYaw) / 2f;
        Vector3 midDirection = Quaternion.AngleAxis(midYaw, charUp) * hips.forward;
        Vector3 pitchAxis = Vector3.Cross(midDirection, charUp).normalized;

        for (int i = 0; i <= 8; i++)
        {
            float pitch = Mathf.Lerp(-maxVerticalSwingAngle, maxVerticalSwingAngle, (float)i / 8);
            Vector3 pitchedDir = Quaternion.AngleAxis(pitch, pitchAxis) * midDirection;
            Vector3 point = shoulderAnchor.position + pitchedDir.normalized * radiusMiddle;

            if (i > 0)
            {
                float prevPitch = Mathf.Lerp(-maxVerticalSwingAngle, maxVerticalSwingAngle, (float)(i - 1) / 8);
                Vector3 prevPitchedDir = Quaternion.AngleAxis(prevPitch, pitchAxis) * midDirection;
                Vector3 prevPoint = shoulderAnchor.position + prevPitchedDir.normalized * radiusMiddle;
                Gizmos.DrawLine(prevPoint, point);
            }
        }

        // Draw shoulder anchor
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(shoulderAnchor.position, 0.03f);

        // Draw label
#if UNITY_EDITOR
        Vector3 labelPos = shoulderAnchor.position + charUp * (isJab ? 0.4f : 0.25f);
        UnityEditor.Handles.Label(
            labelPos,
            $"{(isLeftArm ? "L" : "R")} {(isJab ? "Jab" : "Hook")}\nS:{radiusStart:F2} M:{radiusMiddle:F2} E:{radiusEnd:F2}"
        );
#endif
    }
}