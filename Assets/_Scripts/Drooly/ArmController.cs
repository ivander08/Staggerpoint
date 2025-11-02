using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections;
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
    [SerializeField] private ConfigurableJoint rightWristJoint;

    [Header("Left Arm")]
    [SerializeField] private Rigidbody leftHandRigidbody;
    [SerializeField] private Transform leftShoulderAnchor;
    [SerializeField] private Transform leftArmRoot;
    [SerializeField] private ConfigurableJoint leftForearmJoint;
    [SerializeField] private ConfigurableJoint leftFingerJoint;
    [SerializeField] private ConfigurableJoint leftWristJoint;

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
    [SerializeField][Range(0f, 1f)] private float twistActivateThreshold = 0.7f;
    [SerializeField][Range(-1f, 1f)] private float twistReleaseThreshold = -0.5f;

    [Header("Right Arm Twist Angles")]
    [SerializeField] private float rightWeaponTwistAngleOutward = -170f;
    [SerializeField] private float rightWeaponTwistAngleInward = 170f;

    [Header("Left Arm Twist Angles")]
    [SerializeField] private float leftWeaponTwistAngleOutward = 170f;
    [SerializeField] private float leftWeaponTwistAngleInward = -170f;

    [Header("Wrist Flexion")]
    [SerializeField] private bool enableWristFlexion = true;
    [SerializeField] private float rightWristRadialFlexion = -50f;  // Y rotation at swing start
    [SerializeField] private float rightWristUlnarFlexion = 50f;    // Y rotation at threshold
    [SerializeField] private float leftWristRadialFlexion = -50f;   // Y rotation at swing start
    [SerializeField] private float leftWristUlnarFlexion = 50f;

    [Header("Recoil Response")]
    [Tooltip("How much to reduce follow force during recoil (0 = no force, 1 = full force)")]
    [SerializeField] private float followForceReduction = 0.05f;

    [Tooltip("How long to reduce follow force after impact")]
    [SerializeField] private float recoilRecoveryTime = 0.15f;

    [Tooltip("Show debug info for recoil system")]
    [SerializeField] private bool showRecoilDebug = true;


    private Quaternion _originalRightForearmRotation;
    private Quaternion _originalLeftForearmRotation;
    private int _rightArmTwistState;
    private int _leftArmTwistState;
    private Quaternion _originalRightWristRotation;
    private Quaternion _originalLeftWristRotation;
    #endregion

    #region State
    private bool _isRightArmSwinging;
    private bool _isLeftArmSwinging;
    private bool _isJabMode;

    private float _currentRightSwingYaw;
    private float _currentRightSwingPitch;
    private float _currentLeftSwingYaw;
    private float _currentLeftSwingPitch;

    private float _currentRightFollowForce = 1f;  // 0-1 multiplier
    private float _currentLeftFollowForce = 1f;   // 0-1 multiplier
    private Coroutine _rightRecoilCoroutine;
    private Coroutine _leftRecoilCoroutine;

    private List<ConfigurableJoint> _rightArmJoints = new();
    private List<JointDrive> _originalRightArmDrives = new();
    private List<ConfigurableJoint> _leftArmJoints = new();
    private List<JointDrive> _originalLeftArmDrives = new();

    private float _originalBalanceForce;
    private float _originalBalanceDamper;

    [HideInInspector]
    public bool isTwoHandedMode = false;
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
        if (_isRightArmSwinging) MoveArmTowardsTarget(rightHandRigidbody, rightHandIKTarget, true);
        if (_isLeftArmSwinging) MoveArmTowardsTarget(leftHandRigidbody, leftHandIKTarget, false);
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
        if (rightWristJoint != null)
            _originalRightWristRotation = rightWristJoint.targetRotation;
        if (leftWristJoint != null)
            _originalLeftWristRotation = leftWristJoint.targetRotation;
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

    // Call this in Start() or whenever a weapon is equipped
    public void SubscribeToWeaponRecoil(Weapon weapon)
    {
        if (weapon == null) return;

        WeaponImpactRecoil recoilComponent = weapon.GetComponent<WeaponImpactRecoil>();
        if (recoilComponent == null)
        {
            Debug.LogWarning($"Weapon '{weapon.name}' doesn't have WeaponImpactRecoil component!");
            return;
        }

        // Subscribe to recoil events
        recoilComponent.OnRecoilTriggered += HandleWeaponRecoil;
    }

    // Call this when weapon is dropped
    public void UnsubscribeFromWeaponRecoil(Weapon weapon)
    {
        if (weapon == null) return;

        WeaponImpactRecoil recoilComponent = weapon.GetComponent<WeaponImpactRecoil>();
        if (recoilComponent != null)
        {
            recoilComponent.OnRecoilTriggered -= HandleWeaponRecoil;
        }
    }


    // =====================================================
    // 3. ADD THIS EVENT HANDLER:
    // =====================================================

    private void HandleWeaponRecoil(WeaponImpactRecoil.RecoilData recoilData)
    {
        if (showRecoilDebug)
        {
            Debug.Log($"<color=magenta>[ARM RECOIL]</color> {(recoilData.isRightHand ? "Right" : "Left")} hand responding to {recoilData.impactForce:F0}N impact");
        }

        // Stop any existing recoil recovery for this hand
        if (recoilData.isRightHand)
        {
            if (_rightRecoilCoroutine != null) StopCoroutine(_rightRecoilCoroutine);
            _rightRecoilCoroutine = StartCoroutine(RecoilRecovery(true));
        }
        else
        {
            if (_leftRecoilCoroutine != null) StopCoroutine(_leftRecoilCoroutine);
            _leftRecoilCoroutine = StartCoroutine(RecoilRecovery(false));
        }
    }


    // =====================================================
    // 4. ADD THIS COROUTINE:
    // =====================================================

    private IEnumerator RecoilRecovery(bool isRightHand)
    {
        // Immediately reduce follow force
        if (isRightHand)
            _currentRightFollowForce = followForceReduction;
        else
            _currentLeftFollowForce = followForceReduction;

        if (showRecoilDebug)
        {
            Debug.Log($"<color=orange>[FOLLOW FORCE REDUCED]</color> {(isRightHand ? "Right" : "Left")} hand to {followForceReduction * 100f:F0}%");
        }

        // Wait for recovery time
        yield return new WaitForSeconds(recoilRecoveryTime);

        // Smoothly restore follow force over 0.1 seconds
        float elapsed = 0f;
        float restoreTime = 0.1f;
        float startForce = followForceReduction;

        while (elapsed < restoreTime)
        {
            elapsed += Time.fixedDeltaTime;
            float t = elapsed / restoreTime;

            if (isRightHand)
                _currentRightFollowForce = Mathf.Lerp(startForce, 1f, t);
            else
                _currentLeftFollowForce = Mathf.Lerp(startForce, 1f, t);

            yield return null;
        }

        // Ensure it's fully restored
        if (isRightHand)
            _currentRightFollowForce = 1f;
        else
            _currentLeftFollowForce = 1f;

        if (showRecoilDebug)
        {
            Debug.Log($"<color=green>[FOLLOW FORCE RESTORED]</color> {(isRightHand ? "Right" : "Left")} hand back to 100%");
        }
    }

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
            if (rightWristJoint != null)
                rightWristJoint.targetRotation = _originalRightWristRotation;
        }
        else
        {
            _isLeftArmSwinging = false;
            _leftArmTwistState = 0;
            ReTenseArm(_leftArmJoints, _originalLeftArmDrives);
            leftForearmJoint.targetRotation = _originalLeftForearmRotation;
            if (leftWristJoint != null)
                leftWristJoint.targetRotation = _originalLeftWristRotation;
        }

        if (!_isRightArmSwinging && !_isLeftArmSwinging)
        {
            cameraController.SetLock(false);
            UnbraceTorso();
        }
    }

    // This is the NEW UpdateArmSwings method for ArmController.cs
    private void UpdateArmSwings()
    {
        if (isTwoHandedMode)
        {
            // --- TWO-HANDED LOGIC ---
            // The right arm is the PILOT. It moves based on mouse input.
            UpdateIKTargetPosition(rightHandIKTarget, rightShoulderAnchor, ref _currentRightSwingYaw, ref _currentRightSwingPitch, false);

            // The left arm is the CO-PILOT. Its target is calculated based on the right arm's target.
            // We need a reference to the currently held weapon to know the grip offset.
            Weapon heldWeapon = weaponPickupManager.equippedWeaponRight; // You will need to add a reference to WeaponPickupManager

            if (heldWeapon != null)
            {
                // Calculate the position offset from the right grip to the left grip in world space.
                Vector3 gripOffset = heldWeapon.leftGripPoint.position - heldWeapon.rightGripPoint.position;

                // Set the left hand's IK target to follow the right hand's target.
                leftHandIKTarget.position = rightHandIKTarget.position + gripOffset;
                leftHandIKTarget.rotation = rightHandIKTarget.rotation; // Keep the rotation synchronized as well.
            }
        }
        else
        {
            // --- ONE-HANDED / NORMAL LOGIC ---
            // If not in two-handed mode, run the normal independent arm logic.
            if (_isRightArmSwinging)
                UpdateIKTargetPosition(rightHandIKTarget, rightShoulderAnchor, ref _currentRightSwingYaw, ref _currentRightSwingPitch, false);
            if (_isLeftArmSwinging)
                UpdateIKTargetPosition(leftHandIKTarget, leftShoulderAnchor, ref _currentLeftSwingYaw, ref _currentLeftSwingPitch, true);
        }
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

        float radius = CalculateRadiusAtAngle(yaw, isLeftArm);
        Vector3 newHorizontalDir = Quaternion.AngleAxis(yaw, charUp) * _hipsTransform.forward;
        Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
        Vector3 newArmVector = Quaternion.AngleAxis(pitch, pitchRotationAxis) * newHorizontalDir;
        handIKTarget.position = shoulderAnchor.position + newArmVector.normalized * radius;

        UpdateWeaponTwist(isLeftArm, minYaw, maxYaw, yaw);
        UpdateWristFlexion(isLeftArm, minYaw, maxYaw, yaw);

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

        float targetTwistAngle = 0f;
        if (currentTwistState == 1)
            targetTwistAngle = isRightArm ? rightWeaponTwistAngleOutward : leftWeaponTwistAngleOutward;
        else if (currentTwistState == -1)
            targetTwistAngle = isRightArm ? rightWeaponTwistAngleInward : leftWeaponTwistAngleInward;

        Quaternion twist = Quaternion.Euler(0, 0, targetTwistAngle);
        targetJoint.targetRotation = originalRotation * twist;
    }

    private void UpdateWristFlexion(bool isLeftArm, float minYaw, float maxYaw, float yaw)
    {
        if (!enableWristFlexion) return;

        ConfigurableJoint wristJoint = isLeftArm ? leftWristJoint : rightWristJoint;
        if (wristJoint == null) return;

        Quaternion originalRotation = isLeftArm ? _originalLeftWristRotation : _originalRightWristRotation;
        bool isRightArm = !isLeftArm;

        // Calculate swing progress from -1 (full left/inward) to 1 (full right/outward)
        float swingProgress = (Mathf.InverseLerp(minYaw, maxYaw, yaw) - 0.5f) * 2f;

        // Determine which side of the threshold we're on
        float activate = isRightArm ? twistActivateThreshold : -twistActivateThreshold;

        // Calculate target flexion angle based on threshold
        float targetFlexionAngle;

        if (isRightArm)
        {
            // Right arm: ulnar flexion when past EITHER threshold (0.7 or -0.7)
            if (swingProgress >= twistActivateThreshold || swingProgress <= -twistActivateThreshold)
                targetFlexionAngle = rightWristUlnarFlexion;
            else
                targetFlexionAngle = rightWristRadialFlexion;
        }
        else
        {
            // Left arm: ulnar flexion when past EITHER threshold (-0.7 or 0.7)
            if (swingProgress <= -twistActivateThreshold || swingProgress >= twistActivateThreshold)
                targetFlexionAngle = leftWristUlnarFlexion;
            else
                targetFlexionAngle = leftWristRadialFlexion;
        }

        // Apply the Y-axis rotation for wrist flexion
        Quaternion flexion = Quaternion.Euler(0, targetFlexionAngle, 0);
        wristJoint.targetRotation = originalRotation * flexion;
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
    private void MoveArmTowardsTarget(Rigidbody handRB, Transform handIKTarget, bool isRightHand)
    {
        // Get current follow force multiplier for this hand
        float forceMultiplier = isRightHand ? _currentRightFollowForce : _currentLeftFollowForce;

        // Apply position force (with recoil reduction)
        Vector3 positionDifference = handIKTarget.position - handRB.position;
        handRB.AddForce(positionDifference * followForce * forceMultiplier, ForceMode.Force);

        // Apply rotation torque (with recoil reduction)
        Quaternion rotationDifference = handIKTarget.rotation * Quaternion.Inverse(handRB.rotation);
        rotationDifference.ToAngleAxis(out float angleInDegrees, out Vector3 rotationAxis);
        if (angleInDegrees > 180f) angleInDegrees -= 360f;

        Vector3 torque = rotationAxis.normalized * (angleInDegrees * Mathf.Deg2Rad * rotateTorque);
        handRB.AddTorque(torque * forceMultiplier, ForceMode.Force);
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

        if (!Application.isPlaying || !showRecoilDebug)
            return;

        // Show current follow force as text above hands
        if (rightHandRigidbody != null)
        {
            Vector3 rightPos = rightHandRigidbody.position + Vector3.up * 0.3f;
            Color rightColor = _currentRightFollowForce < 0.5f ? Color.red : Color.green;
            Gizmos.color = rightColor;
            Gizmos.DrawWireSphere(rightHandRigidbody.position, 0.05f);

#if UNITY_EDITOR
            UnityEditor.Handles.Label(rightPos, $"R Force: {_currentRightFollowForce * 100f:F0}%");
#endif
        }

        if (leftHandRigidbody != null)
        {
            Vector3 leftPos = leftHandRigidbody.position + Vector3.up * 0.3f;
            Color leftColor = _currentLeftFollowForce < 0.5f ? Color.red : Color.green;
            Gizmos.color = leftColor;
            Gizmos.DrawWireSphere(leftHandRigidbody.position, 0.05f);

#if UNITY_EDITOR
            UnityEditor.Handles.Label(leftPos, $"L Force: {_currentLeftFollowForce * 100f:F0}%");
#endif
        }

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