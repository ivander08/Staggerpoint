using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;

public class ArmController : MonoBehaviour
{
    #region Components
    [SerializeField] private ActiveRagdoll activeRagdoll;
    [SerializeField] private CameraController cameraController;
    [SerializeField] private WeaponPickupManager weaponPickupManager;

    private Transform _cameraTransform;
    private Transform _hipsTransform;
    #endregion

    #region Arm Rigging
    [Header("Right Arm")]
    [SerializeField] private Rigidbody rightHandRigidbody;
    [SerializeField] private Transform rightShoulderAnchor;
    [SerializeField] private Transform rightArmRoot;
    [SerializeField] private ConfigurableJoint rightForearmJoint;
    [SerializeField] private ConfigurableJoint rightFingerJoint;

    [Header("Left Arm")]
    [SerializeField] private Rigidbody leftHandRigidbody;
    [SerializeField] private Transform leftShoulderAnchor;
    [SerializeField] private Transform leftArmRoot;
    [SerializeField] private ConfigurableJoint leftForearmJoint;
    [SerializeField] private ConfigurableJoint leftFingerJoint;

    [Header("IK Targets")]
    [SerializeField] private Transform rightHandIKTarget;
    [SerializeField] private Transform leftHandIKTarget;
    #endregion

    #region Physics Settings
    [Header("Physics")]
    [SerializeField] private float followForce = 50000f;
    [SerializeField] private float rotateTorque = 5000f;
    #endregion

    #region Hand Animation
    [Header("Finger Control")]
    [SerializeField] private float rightFistCurlAngle = -120f;
    [SerializeField] private float leftFistCurlAngle = 120f;
    [SerializeField] private float fingerSpring = 100f;
    [SerializeField] private float fingerDamper = 5f;

    private Quaternion _rightFingerStartRotation;
    private Quaternion _leftFingerStartRotation;
    #endregion

    #region Punch Styles
    [Header("Hook Punch")]
    [SerializeField] private float hookRadiusStart = 0.5f;
    [SerializeField] private float hookRadiusMiddle = 0.7f;
    [SerializeField] private float hookRadiusEnd = 0.5f;

    [Header("Jab Punch")]
    [SerializeField] private float jabRadiusStart = 0.9f;
    [SerializeField] private float jabRadiusMiddle = 1.2f;
    [SerializeField] private float jabRadiusEnd = 1.0f;

    [Header("Weapon Hold")]
    [SerializeField] private float weaponRadiusStart = 0.6f;
    [SerializeField] private float weaponRadiusMiddle = 0.8f;
    [SerializeField] private float weaponRadiusEnd = 0.6f;
    #endregion

    #region Swing Control
    [Header("Swing Control")]
    [SerializeField] private float swingSensitivity = 1.5f;
    [SerializeField] private float swingAngleAcrossBody = 45f;
    [SerializeField] private float swingAngleOutward = 90f;
    [SerializeField] private float maxVerticalSwingAngle = 80f;
    #endregion

    #region Weapon Twist
    [Header("Weapon Twist")]
    [SerializeField] private bool enableWeaponTwist = true;
    [SerializeField] [Range(0f, 1f)] private float twistActivateThreshold = 0.7f;
    [SerializeField] [Range(-1f, 1f)] private float twistReleaseThreshold = -0.5f;

    [Header("Right Arm Twist Angles")]
    [SerializeField] private float rightWeaponTwistAngleOutward = -170f;
    [SerializeField] private float rightWeaponTwistAngleInward = 170f;

    [Header("Left Arm Twist Angles")]
    [SerializeField] private float leftWeaponTwistAngleOutward = 170f;
    [SerializeField] private float leftWeaponTwistAngleInward = -170f;

    private Quaternion _originalRightForearmRotation;
    private Quaternion _originalLeftForearmRotation;
    private int _rightArmTwistState;
    private int _leftArmTwistState;
    #endregion

    #region State
    private bool _isRightArmSwinging;
    private bool _isLeftArmSwinging;
    private bool _isJabMode;

    private float _currentRightSwingYaw;
    private float _currentRightSwingPitch;
    private float _currentLeftSwingYaw;
    private float _currentLeftSwingPitch;

    private List<ConfigurableJoint> _rightArmJoints = new();
    private List<JointDrive> _originalRightArmDrives = new();
    private List<ConfigurableJoint> _leftArmJoints = new();
    private List<JointDrive> _originalLeftArmDrives = new();

    private float _originalBalanceForce;
    private float _originalBalanceDamper;
    #endregion

    #region Debug
    [Header("Debug")]
    [SerializeField] private bool showPunchPaths = true;
    [SerializeField] private int pathResolution = 32;
    #endregion

    void Awake()
    {
        ValidateComponents();
        InitializeArms();
        CacheOriginalValues();
    }

    void Update()
    {
        HandleInput();
        UpdateArmSwings();
        UpdateFingerCurls();
    }

    void FixedUpdate()
    {
        if (_isRightArmSwinging) MoveArmTowardsTarget(rightHandRigidbody, rightHandIKTarget);
        if (_isLeftArmSwinging) MoveArmTowardsTarget(leftHandRigidbody, leftHandIKTarget);
    }

    #region Initialization
    private void ValidateComponents()
    {
        if (cameraController == null)
        {
            Debug.LogError("CameraController not assigned!");
            enabled = false;
            return;
        }

        _cameraTransform = cameraController.transform;
        _hipsTransform = activeRagdoll.hipsTransform;

        rightHandRigidbody.maxAngularVelocity = 50f;
        leftHandRigidbody.maxAngularVelocity = 50f;
    }

    private void InitializeArms()
    {
        InitializeArm(rightArmRoot, _rightArmJoints, _originalRightArmDrives);
        InitializeArm(leftArmRoot, _leftArmJoints, _originalLeftArmDrives);

        _rightFingerStartRotation = rightFingerJoint.transform.localRotation;
        _leftFingerStartRotation = leftFingerJoint.transform.localRotation;
    }

    private void InitializeArm(Transform armRoot, List<ConfigurableJoint> joints, List<JointDrive> drives)
    {
        if (armRoot == null) return;

        joints.AddRange(armRoot.GetComponentsInChildren<ConfigurableJoint>());
        foreach (var joint in joints)
            drives.Add(joint.angularXDrive);
    }

    private void CacheOriginalValues()
    {
        _originalBalanceForce = activeRagdoll.balanceForce;
        _originalBalanceDamper = activeRagdoll.balanceDamping;
        _originalRightForearmRotation = rightForearmJoint.targetRotation;
        _originalLeftForearmRotation = leftForearmJoint.targetRotation;
    }
    #endregion

    #region Input Handling
    private void HandleInput()
    {
        bool holdingWeapon = IsArmHoldingWeapon(true) || IsArmHoldingWeapon(false);
        _isJabMode = !holdingWeapon && (Keyboard.current.leftAltKey.isPressed || Keyboard.current.rightAltKey.isPressed);

        HandleMouseInput(true);
        HandleMouseInput(false);
    }

    private void HandleMouseInput(bool isRightArm)
    {
        bool mousePressed = isRightArm ? Mouse.current.rightButton.isPressed : Mouse.current.leftButton.isPressed;
        bool isSwinging = isRightArm ? _isRightArmSwinging : _isLeftArmSwinging;

        if (mousePressed && !isSwinging)
            StartSwing(isRightArm);
        else if (!mousePressed && isSwinging)
            EndSwing(isRightArm);
    }

    private bool IsArmHoldingWeapon(bool isRightArm)
    {
        if (weaponPickupManager == null) return false;
        return isRightArm ? (weaponPickupManager.equippedWeaponRight != null) : (weaponPickupManager.equippedWeaponLeft != null);
    }
    #endregion

    #region Swing Management
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
            _rightArmTwistState = 0;
            rightHandIKTarget.position = rightHandRigidbody.position;
            InitializeSwingAngles(rightHandRigidbody, rightShoulderAnchor, ref _currentRightSwingYaw, ref _currentRightSwingPitch);
        }
        else
        {
            _isLeftArmSwinging = true;
            _leftArmTwistState = 0;
            leftHandIKTarget.position = leftHandRigidbody.position;
            InitializeSwingAngles(leftHandRigidbody, leftShoulderAnchor, ref _currentLeftSwingYaw, ref _currentLeftSwingPitch);
        }
    }

    private void EndSwing(bool isRightArm)
    {
        if (isRightArm)
        {
            _isRightArmSwinging = false;
            _rightArmTwistState = 0;
            ReTenseArm(_rightArmJoints, _originalRightArmDrives);
            rightForearmJoint.targetRotation = _originalRightForearmRotation;
        }
        else
        {
            _isLeftArmSwinging = false;
            _leftArmTwistState = 0;
            ReTenseArm(_leftArmJoints, _originalLeftArmDrives);
            leftForearmJoint.targetRotation = _originalLeftForearmRotation;
        }

        if (!_isRightArmSwinging && !_isLeftArmSwinging)
        {
            cameraController.SetLock(false);
            UnbraceTorso();
        }
    }

    private void UpdateArmSwings()
    {
        if (_isRightArmSwinging)
            UpdateIKTargetPosition(rightHandIKTarget, rightShoulderAnchor, ref _currentRightSwingYaw, ref _currentRightSwingPitch, false);
        if (_isLeftArmSwinging)
            UpdateIKTargetPosition(leftHandIKTarget, leftShoulderAnchor, ref _currentLeftSwingYaw, ref _currentLeftSwingPitch, true);
    }
    #endregion

    #region Torso Bracing
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
    #endregion

    #region Swing Logic
    private void InitializeSwingAngles(Rigidbody handRB, Transform shoulderAnchor, ref float yaw, ref float pitch)
    {
        Vector3 charUp = _hipsTransform.up;
        Vector3 currentArmVector = handRB.position - shoulderAnchor.position;
        Vector3 armHorizontal = Vector3.ProjectOnPlane(currentArmVector, charUp);

        yaw = Vector3.SignedAngle(Vector3.ProjectOnPlane(_hipsTransform.forward, charUp), armHorizontal, charUp);
        pitch = Vector3.Angle(armHorizontal, currentArmVector) * Mathf.Sign(Vector3.Dot(currentArmVector, charUp));
    }

    private void UpdateIKTargetPosition(Transform handIKTarget, Transform shoulderAnchor, ref float yaw, ref float pitch, bool isLeftArm)
    {
        Vector3 charUp = _hipsTransform.up;
        Vector2 mouseDelta = Mouse.current.delta.ReadValue();

        yaw += mouseDelta.x * swingSensitivity * 0.1f;
        pitch += mouseDelta.y * swingSensitivity * 0.1f;

        GetSwingAngleLimits(isLeftArm, out float minYaw, out float maxYaw);
        yaw = Mathf.Clamp(yaw, minYaw, maxYaw);
        pitch = Mathf.Clamp(pitch, -maxVerticalSwingAngle, maxVerticalSwingAngle);

        // Update IK target position
        float radius = CalculateRadiusAtAngle(yaw, isLeftArm);
        Vector3 newHorizontalDir = Quaternion.AngleAxis(yaw, charUp) * _hipsTransform.forward;
        Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
        Vector3 newArmVector = Quaternion.AngleAxis(pitch, pitchRotationAxis) * newHorizontalDir;
        handIKTarget.position = shoulderAnchor.position + newArmVector.normalized * radius;

        // Update weapon twist
        UpdateWeaponTwist(isLeftArm, minYaw, maxYaw, yaw);

        // Update hand rotation
        handIKTarget.rotation = Quaternion.LookRotation(_cameraTransform.forward, charUp);
    }

    private void GetSwingAngleLimits(bool isLeftArm, out float minYaw, out float maxYaw)
    {
        if (isLeftArm)
        {
            minYaw = -swingAngleOutward;
            maxYaw = swingAngleAcrossBody;
        }
        else
        {
            minYaw = -swingAngleAcrossBody;
            maxYaw = swingAngleOutward;
        }
    }

    private void UpdateWeaponTwist(bool isLeftArm, float minYaw, float maxYaw, float yaw)
    {
        ConfigurableJoint targetJoint = isLeftArm ? leftForearmJoint : rightForearmJoint;
        Quaternion originalRotation = isLeftArm ? _originalLeftForearmRotation : _originalRightForearmRotation;
        bool isRightArm = !isLeftArm;

        if (!enableWeaponTwist || !IsArmHoldingWeapon(isRightArm))
        {
            targetJoint.targetRotation = originalRotation;
            if (isLeftArm) _leftArmTwistState = 0;
            else _rightArmTwistState = 0;
            return;
        }

        float swingProgress = (Mathf.InverseLerp(minYaw, maxYaw, yaw) - 0.5f) * 2f;
        float activate = isRightArm ? twistActivateThreshold : -twistActivateThreshold;
        float release = isRightArm ? twistReleaseThreshold : -twistReleaseThreshold;

        int currentTwistState = isLeftArm ? _leftArmTwistState : _rightArmTwistState;

        // State machine
        if (currentTwistState == 0)
        {
            if (isRightArm ? (swingProgress > activate) : (swingProgress < activate))
                currentTwistState = 1;
            else if (isRightArm ? (swingProgress < -activate) : (swingProgress > -activate))
                currentTwistState = -1;
        }
        else if (currentTwistState == 1)
        {
            if (isRightArm ? (swingProgress < release) : (swingProgress > release))
                currentTwistState = 0;
        }
        else if (currentTwistState == -1)
        {
            if (isRightArm ? (swingProgress > -release) : (swingProgress < -release))
                currentTwistState = 0;
        }

        if (isLeftArm) _leftArmTwistState = currentTwistState;
        else _rightArmTwistState = currentTwistState;

        // Apply twist
        float targetTwistAngle = 0f;
        if (currentTwistState == 1)
            targetTwistAngle = isRightArm ? rightWeaponTwistAngleOutward : leftWeaponTwistAngleOutward;
        else if (currentTwistState == -1)
            targetTwistAngle = isRightArm ? rightWeaponTwistAngleInward : leftWeaponTwistAngleInward;

        Quaternion twist = Quaternion.Euler(0, 0, targetTwistAngle);
        targetJoint.targetRotation = originalRotation * twist;
    }

    private float CalculateRadiusAtAngle(float yaw, bool isLeftArm)
    {
        bool holdingWeapon = IsArmHoldingWeapon(!isLeftArm);
        float radiusStart, radiusMiddle, radiusEnd;

        if (holdingWeapon)
        {
            radiusStart = weaponRadiusStart;
            radiusMiddle = weaponRadiusMiddle;
            radiusEnd = weaponRadiusEnd;
        }
        else
        {
            radiusStart = _isJabMode ? jabRadiusStart : hookRadiusStart;
            radiusMiddle = _isJabMode ? jabRadiusMiddle : hookRadiusMiddle;
            radiusEnd = _isJabMode ? jabRadiusEnd : hookRadiusEnd;
        }

        GetSwingAngleLimits(isLeftArm, out float minYaw, out float maxYaw);
        float normalizedYaw = Mathf.InverseLerp(minYaw, maxYaw, yaw);

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
    #endregion

    #region Arm Movement
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

    private void ReTenseArm(List<ConfigurableJoint> joints, List<JointDrive> originalDrives)
    {
        for (int i = 0; i < joints.Count; i++)
        {
            joints[i].angularXDrive = originalDrives[i];
            joints[i].angularYZDrive = originalDrives[i];
        }
    }
    #endregion

    #region Hand Animation
    private void UpdateFingerCurls()
    {
        bool rightShouldCurl = _isRightArmSwinging || IsArmHoldingWeapon(true);
        bool leftShouldCurl = _isLeftArmSwinging || IsArmHoldingWeapon(false);

        UpdateFingerCurl(rightFingerJoint, rightShouldCurl, _rightFingerStartRotation, rightFistCurlAngle);
        UpdateFingerCurl(leftFingerJoint, leftShouldCurl, _leftFingerStartRotation, leftFistCurlAngle);
    }

    private void UpdateFingerCurl(ConfigurableJoint fingerJoint, bool shouldCurl, Quaternion startRotation, float curlAngle)
    {
        if (fingerJoint == null) return;

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
            Quaternion targetRotation = startRotation * Quaternion.Euler(curlAngle, 0, 0);
            fingerJoint.targetRotation = Quaternion.Inverse(targetRotation) * startRotation;
        }
        else
        {
            fingerJoint.targetRotation = Quaternion.identity;
        }
    }
    #endregion

    #region Debug Visualization
    void OnDrawGizmos()
    {
        if (!showPunchPaths || !Application.isPlaying || activeRagdoll == null) return;

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

        DrawActiveHandMarkers();
    }

    private void DrawActiveHandMarkers()
    {
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
        Vector3 charUp = _hipsTransform.up;

        float radiusStart = isJab ? jabRadiusStart : hookRadiusStart;
        float radiusMiddle = isJab ? jabRadiusMiddle : hookRadiusMiddle;
        float radiusEnd = isJab ? jabRadiusEnd : hookRadiusEnd;

        Color pathColor = GetPunchPathColor(isLeftArm, isJab);
        Gizmos.color = pathColor;

        GetSwingAngleLimits(isLeftArm, out float minYaw, out float maxYaw);

        Vector3 previousPoint = Vector3.zero;
        bool firstPoint = true;

        for (int i = 0; i <= pathResolution; i++)
        {
            float t = (float)i / pathResolution;
            float yaw = Mathf.Lerp(minYaw, maxYaw, t);

            float radius = (t < 0.5f)
                ? Mathf.Lerp(radiusStart, radiusMiddle, t * 2f)
                : Mathf.Lerp(radiusMiddle, radiusEnd, (t - 0.5f) * 2f);

            Vector3 direction = Quaternion.AngleAxis(yaw, charUp) * _hipsTransform.forward;
            Vector3 point = shoulderAnchor.position + direction.normalized * radius;

            if (!firstPoint)
                Gizmos.DrawLine(previousPoint, point);

            previousPoint = point;
            firstPoint = false;

            if (i == 0 || i == pathResolution / 2 || i == pathResolution)
                Gizmos.DrawWireSphere(point, 0.04f);
        }

        DrawPitchRange(shoulderAnchor, minYaw, maxYaw, radiusMiddle, charUp);
        DrawShoulderMarker(shoulderAnchor);
        DrawPathLabel(shoulderAnchor, isLeftArm, isJab, radiusStart, radiusMiddle, radiusEnd, charUp);
    }

    private Color GetPunchPathColor(bool isLeftArm, bool isJab)
    {
        if (isLeftArm)
            return isJab ? new Color(0.3f, 0.9f, 0.9f, 0.5f) : new Color(0.3f, 0.5f, 1f, 0.5f);
        else
            return isJab ? new Color(0.9f, 0.9f, 0.3f, 0.5f) : new Color(1f, 0.5f, 0.3f, 0.5f);
    }

    private void DrawPitchRange(Transform shoulderAnchor, float minYaw, float maxYaw, float radiusMiddle, Vector3 charUp)
    {
        float midYaw = (minYaw + maxYaw) / 2f;
        Vector3 midDirection = Quaternion.AngleAxis(midYaw, charUp) * _hipsTransform.forward;
        Vector3 pitchAxis = Vector3.Cross(midDirection, charUp).normalized;

        for (int i = 0; i <= 8; i++)
        {
            if (i > 0)
            {
                float pitch = Mathf.Lerp(-maxVerticalSwingAngle, maxVerticalSwingAngle, (float)i / 8);
                float prevPitch = Mathf.Lerp(-maxVerticalSwingAngle, maxVerticalSwingAngle, (float)(i - 1) / 8);

                Vector3 pitchedDir = Quaternion.AngleAxis(pitch, pitchAxis) * midDirection;
                Vector3 prevPitchedDir = Quaternion.AngleAxis(prevPitch, pitchAxis) * midDirection;

                Vector3 point = shoulderAnchor.position + pitchedDir.normalized * radiusMiddle;
                Vector3 prevPoint = shoulderAnchor.position + prevPitchedDir.normalized * radiusMiddle;

                Gizmos.DrawLine(prevPoint, point);
            }
        }
    }

    private void DrawShoulderMarker(Transform shoulderAnchor)
    {
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(shoulderAnchor.position, 0.03f);
    }

    private void DrawPathLabel(Transform shoulderAnchor, bool isLeftArm, bool isJab, float radiusStart, float radiusMiddle, float radiusEnd, Vector3 charUp)
    {
#if UNITY_EDITOR
        Vector3 labelPos = shoulderAnchor.position + charUp * (isJab ? 0.4f : 0.25f);
        string label = $"{(isLeftArm ? "L" : "R")} {(isJab ? "Jab" : "Hook")}\nS:{radiusStart:F2} M:{radiusMiddle:F2} E:{radiusEnd:F2}";
        UnityEditor.Handles.Label(labelPos, label);
#endif
    }
    #endregion
}