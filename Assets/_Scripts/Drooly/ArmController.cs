using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections;
using System.Collections.Generic;

public class ArmController : MonoBehaviour
{
    [SerializeField] private ActiveRagdoll activeRagdoll;
    [SerializeField] private CameraController cameraController;
    [SerializeField] private WeaponPickupManager weaponPickupManager;

    private Transform _cameraTransform;
    private Transform _hipsTransform;

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

    [Header("Physics")]
    [SerializeField] private float followForce = 2000f;
    [SerializeField] private float rotateTorque = 1000f;

    [Header("Finger Control")]
    [SerializeField] private float rightFistCurlAngle = -120f;
    [SerializeField] private float leftFistCurlAngle = 120f;
    [SerializeField] private float fingerSpring = 100f;
    [SerializeField] private float fingerDamper = 5f;

    private Quaternion _rightFingerStartRotation;
    private Quaternion _leftFingerStartRotation;

    [Header("Hook Punch")]
    [SerializeField] private float hookRadiusStart = 0.5f;
    [SerializeField] private float hookRadiusMiddle = 0.7f;
    [SerializeField] private float hookRadiusEnd = 0.5f;

    [Header("Jab Punch")]
    [SerializeField] private float jabRadiusStart = 0.9f;
    [SerializeField] private float jabRadiusMiddle = 1.2f;
    [SerializeField] private float jabRadiusEnd = 1.0f;

    [Header("One-Handed Weapon Hold")]
    [SerializeField] private float weaponRadiusStart = 0.6f;
    [SerializeField] private float weaponRadiusMiddle = 0.8f;
    [SerializeField] private float weaponRadiusEnd = 0.6f;//

    [Header("Two-Handed Weapon Hold")]
    [SerializeField] private float twoHandedWeaponRadiusStart = 0.4f;
    [SerializeField] private float twoHandedWeaponRadiusMiddle = 0.6f;
    [SerializeField] private float twoHandedWeaponRadiusEnd = 0.4f;

    [Header("Two-Handed Forearm Flexion")]
    [SerializeField] private float twoHandedForearmFlexionMax = 45f;

    [Header("Swing Control")]
    [SerializeField] private float swingSensitivity = 1.5f;
    [SerializeField] private float swingAngleAcrossBody = 45f;
    [SerializeField] private float swingAngleOutward = 90f;
    [SerializeField] private float maxVerticalSwingAngle = 80f;

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
    [SerializeField] private float rightWristRadialFlexion = -50f;
    [SerializeField] private float rightWristUlnarFlexion = 50f;
    [SerializeField] private float leftWristRadialFlexion = -50f;
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

    private bool _isRightArmSwinging;
    private bool _isLeftArmSwinging;
    private bool _isJabMode;

    private float _currentRightSwingYaw;
    private float _currentRightSwingPitch;
    private float _currentLeftSwingYaw;
    private float _currentLeftSwingPitch;

    private float _currentRightFollowForce = 1f;
    private float _currentLeftFollowForce = 1f;
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

    [Header("Debug")]
    [SerializeField] private bool showPunchPaths = true;
    [SerializeField] private int pathResolution = 32;

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

    public void SubscribeToWeaponRecoil(Weapon weapon)
    {
        if (weapon == null) return;

        WeaponImpactRecoil recoilComponent = weapon.GetComponent<WeaponImpactRecoil>();
        if (recoilComponent == null)
        {
            Debug.LogWarning($"Weapon '{weapon.name}' doesn't have WeaponImpactRecoil component!");
            return;
        }

        recoilComponent.OnRecoilTriggered += HandleWeaponRecoil;
    }

    public void UnsubscribeFromWeaponRecoil(Weapon weapon)
    {
        if (weapon == null) return;

        WeaponImpactRecoil recoilComponent = weapon.GetComponent<WeaponImpactRecoil>();
        if (recoilComponent != null)
        {
            recoilComponent.OnRecoilTriggered -= HandleWeaponRecoil;
        }
    }

    private void HandleWeaponRecoil(WeaponImpactRecoil.RecoilData recoilData)
    {
        if (showRecoilDebug)
        {
            Debug.Log($"<color=magenta>[ARM RECOIL]</color> {(recoilData.isRightHand ? "Right" : "Left")} hand responding to {recoilData.impactForce:F0}N impact");
        }

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

    private IEnumerator RecoilRecovery(bool isRightHand)
    {
        if (isRightHand)
            _currentRightFollowForce = followForceReduction;
        else
            _currentLeftFollowForce = followForceReduction;

        if (showRecoilDebug)
        {
            Debug.Log($"<color=orange>[FOLLOW FORCE REDUCED]</color> {(isRightHand ? "Right" : "Left")} hand to {followForceReduction * 100f:F0}%");
        }

        yield return new WaitForSeconds(recoilRecoveryTime);

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

        if (isRightHand)
            _currentRightFollowForce = 1f;
        else
            _currentLeftFollowForce = 1f;

        if (showRecoilDebug)
        {
            Debug.Log($"<color=green>[FOLLOW FORCE RESTORED]</color> {(isRightHand ? "Right" : "Left")} hand back to 100%");
        }
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

    private void BraceTorso()
    {
        activeRagdoll.balanceForce = (int)(_originalBalanceForce * 20); // 2 
        activeRagdoll.balanceDamping = (int)(_originalBalanceDamper * 50); // 5
        activeRagdoll.SetupBalanceJoint();
    }

    private void UnbraceTorso()
    {
        activeRagdoll.balanceForce = (int)_originalBalanceForce;
        activeRagdoll.balanceDamping = (int)_originalBalanceDamper;
        activeRagdoll.SetupBalanceJoint();
    }

    private void InitializeSwingAngles(Rigidbody handRB, Transform shoulderAnchor, ref float yaw, ref float pitch)
    {
        Vector3 charUp = _hipsTransform.up;
        Vector3 currentArmVector = handRB.position - shoulderAnchor.position;
        Vector3 armHorizontal = Vector3.ProjectOnPlane(currentArmVector, charUp);

        yaw = Vector3.SignedAngle(Vector3.ProjectOnPlane(_hipsTransform.forward, charUp), armHorizontal, charUp);
        pitch = Vector3.Angle(armHorizontal, currentArmVector) * Mathf.Sign(Vector3.Dot(currentArmVector, charUp));
    }

    private void UpdateArmSwings()
    {
        if (isTwoHandedMode)
        {
            bool bothButtonsHeld = Mouse.current.leftButton.isPressed && Mouse.current.rightButton.isPressed;

            UpdateWeaponSwing(true, isTwoHanded: true, bothButtonsHeld: bothButtonsHeld);

            // NEW: Calculate hand positions with crossover logic
            CalculateTwoHandedHandPositions();
        }
        else
        {
            // ONE-HANDED/EMPTY-HANDED MODE: Independent arm control
            if (_isRightArmSwinging)
                UpdateWeaponSwing(true, isTwoHanded: false, bothButtonsHeld: false);
            if (_isLeftArmSwinging)
                UpdateWeaponSwing(false, isTwoHanded: false, bothButtonsHeld: false);
        }
    }

    private void CalculateTwoHandedHandPositions()
    {
        Weapon heldWeapon = weaponPickupManager.equippedWeaponRight;
        if (heldWeapon == null) return;

        // Calculate swing progress from right arm  
        float swingProgress = (_currentRightSwingYaw + swingAngleAcrossBody) / (swingAngleOutward + swingAngleAcrossBody);
        swingProgress = (swingProgress - 0.5f) * 2f; // Convert to -1 to 1 range

        float crossoverThreshold = 0.7f; // Hardcoded threshold
        Vector3 gripOffset = heldWeapon.leftGripPoint.position - heldWeapon.rightGripPoint.position;

        if (swingProgress > crossoverThreshold || swingProgress < -crossoverThreshold)
        {
            // OUTER ZONES (1.0 to 0.7 and -0.7 to -1.0): Both hands same position
            leftHandIKTarget.position = rightHandIKTarget.position;
        }
        else
        {
            // MIDDLE ZONE (-0.7 to 0.7): Right hand leads, left hand behind (toward body)
            // Calculate direction from right hand toward body center
            Vector3 rightHandToBody = _hipsTransform.position - rightHandIKTarget.position;
            rightHandToBody.y = 0; // Keep it horizontal
            Vector3 toBodyDirection = rightHandToBody.normalized;

            // Move left hand toward body by the grip offset distance
            leftHandIKTarget.position = rightHandIKTarget.position + toBodyDirection * (gripOffset.magnitude * 1.2f);
        }

        leftHandIKTarget.rotation = rightHandIKTarget.rotation;
    }

    private void UpdateWeaponSwing(bool isRightArm, bool isTwoHanded = false, bool bothButtonsHeld = false)
    {
        Vector3 charUp = _hipsTransform.up;
        Transform shoulderAnchor = isRightArm ? rightShoulderAnchor : leftShoulderAnchor;
        Transform handIKTarget = isRightArm ? rightHandIKTarget : leftHandIKTarget;
        ref float yaw = ref (isRightArm ? ref _currentRightSwingYaw : ref _currentLeftSwingYaw);
        ref float pitch = ref (isRightArm ? ref _currentRightSwingPitch : ref _currentLeftSwingPitch);

        // STEP 1: Update yaw/pitch from mouse input
        Vector2 mouseDelta = Mouse.current.delta.ReadValue();
        yaw += mouseDelta.x * swingSensitivity * 0.1f;
        pitch += mouseDelta.y * swingSensitivity * 0.1f;

        // STEP 2: Get swing angle limits based on arm side
        float minYaw, maxYaw;
        if (isRightArm)
        {
            minYaw = -swingAngleAcrossBody;
            maxYaw = swingAngleOutward;
        }
        else
        {
            minYaw = -swingAngleOutward;
            maxYaw = swingAngleAcrossBody;
        }

        // STEP 3: Clamp angles
        yaw = Mathf.Clamp(yaw, minYaw, maxYaw);
        pitch = Mathf.Clamp(pitch, -maxVerticalSwingAngle, maxVerticalSwingAngle);

        // STEP 4: Calculate arm reach distance (radius) based on punch type/weapon/two-handed status
        bool holdingWeapon = IsArmHoldingWeapon(isRightArm);
        float radiusStart, radiusMiddle, radiusEnd;

        if (holdingWeapon && isTwoHanded)
        {
            // TWO-HANDED WEAPON: Shorter reach, especially to the sides
            radiusStart = twoHandedWeaponRadiusStart;
            radiusMiddle = twoHandedWeaponRadiusMiddle;
            radiusEnd = twoHandedWeaponRadiusEnd;
        }
        else if (holdingWeapon)
        {
            // ONE-HANDED WEAPON
            radiusStart = weaponRadiusStart;
            radiusMiddle = weaponRadiusMiddle;
            radiusEnd = weaponRadiusEnd;
        }
        else
        {
            // EMPTY-HANDED: Jab or Hook
            radiusStart = _isJabMode ? jabRadiusStart : hookRadiusStart;
            radiusMiddle = _isJabMode ? jabRadiusMiddle : hookRadiusMiddle;
            radiusEnd = _isJabMode ? jabRadiusEnd : hookRadiusEnd;
        }

        float normalizedYaw = Mathf.InverseLerp(minYaw, maxYaw, yaw);
        float radius = (normalizedYaw < 0.5f)
            ? Mathf.Lerp(radiusStart, radiusMiddle, normalizedYaw * 2f)
            : Mathf.Lerp(radiusMiddle, radiusEnd, (normalizedYaw - 0.5f) * 2f);

        // STEP 5: Calculate hand position in 3D space
        Vector3 newHorizontalDir = Quaternion.AngleAxis(yaw, charUp) * _hipsTransform.forward;
        Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
        Vector3 newArmVector = Quaternion.AngleAxis(pitch, pitchRotationAxis) * newHorizontalDir;
        handIKTarget.position = shoulderAnchor.position + newArmVector.normalized * radius;

        // STEP 6: Handle FOREARM FLEXION
        ConfigurableJoint forearmJoint = isRightArm ? rightForearmJoint : leftForearmJoint;
        Quaternion originalForearmRotation = isRightArm ? _originalRightForearmRotation : _originalLeftForearmRotation;

        if (holdingWeapon && isTwoHanded)
        {
            // TWO-HANDED: Threshold-based elbow flexion (both arms synchronized using right arm's swing progress)
            float swingProgress = (_currentRightSwingYaw - minYaw) / (maxYaw - minYaw); // Use right arm's progress for both arms
            swingProgress = (swingProgress - 0.5f) * 2f; // Convert to -1 to 1 range

            float flexionAngle = 0f;
            float bendThreshold = 0.7f; // Hardcoded threshold

            if (swingProgress >= bendThreshold) // 0.7 to 1.0: bend
            {
                float bendAmount = (swingProgress - bendThreshold) / (1f - bendThreshold);
                flexionAngle = -Mathf.Lerp(0f, twoHandedForearmFlexionMax, bendAmount);
            }
            else if (swingProgress <= -bendThreshold) // -0.7 to -1.0: bend
            {
                float bendAmount = (-swingProgress - bendThreshold) / (1f - bendThreshold);
                flexionAngle = -Mathf.Lerp(0f, twoHandedForearmFlexionMax, bendAmount);
            }
            // Between -0.7 and 0.7: stay straight (flexionAngle = 0)

            // Add Z rotation: +150 for right arm, -150 for left arm
            float zRotation = isRightArm ? 150f : -150f;
            Quaternion flexion = Quaternion.Euler(flexionAngle, 0, zRotation);
            forearmJoint.targetRotation = originalForearmRotation * flexion;

            // ALSO apply flexion to the LEFT arm if we're processing the right arm
            if (isRightArm)
            {
                Quaternion leftFlexion = Quaternion.Euler(flexionAngle, 0, -150f); // Left arm gets -150 Z rotation
                leftForearmJoint.targetRotation = _originalLeftForearmRotation * leftFlexion;
            }
        }
        else if (enableWeaponTwist && holdingWeapon && !isTwoHanded)
        {
            // ONE-HANDED WEAPON: Original twist logic
            ref int twistState = ref (isRightArm ? ref _rightArmTwistState : ref _leftArmTwistState);

            float swingProgress = (normalizedYaw - 0.5f) * 2f;
            float activateThreshold = isRightArm ? twistActivateThreshold : -twistActivateThreshold;
            float releaseThreshold = isRightArm ? twistReleaseThreshold : -twistReleaseThreshold;

            // State machine: 0 = neutral, 1 = outward twist, -1 = inward twist
            if (twistState == 0)
            {
                if (isRightArm ? (swingProgress > activateThreshold) : (swingProgress < activateThreshold))
                    twistState = 1;
                else if (isRightArm ? (swingProgress < -activateThreshold) : (swingProgress > -activateThreshold))
                    twistState = -1;
            }
            else if (twistState == 1)
            {
                if (isRightArm ? (swingProgress < releaseThreshold) : (swingProgress > releaseThreshold))
                    twistState = 0;
            }
            else if (twistState == -1)
            {
                if (isRightArm ? (swingProgress > -releaseThreshold) : (swingProgress < -releaseThreshold))
                    twistState = 0;
            }

            float targetTwistAngle = 0f;
            if (twistState == 1)
                targetTwistAngle = isRightArm ? rightWeaponTwistAngleOutward : leftWeaponTwistAngleOutward;
            else if (twistState == -1)
                targetTwistAngle = isRightArm ? rightWeaponTwistAngleInward : leftWeaponTwistAngleInward;

            Quaternion twist = Quaternion.Euler(0, 0, targetTwistAngle);
            forearmJoint.targetRotation = originalForearmRotation * twist;
        }
        else
        {
            forearmJoint.targetRotation = originalForearmRotation;
        }

        // STEP 7: Handle WRIST FLEXION
        ConfigurableJoint wristJoint = isRightArm ? rightWristJoint : leftWristJoint;
        if (enableWristFlexion && wristJoint != null)
        {
            Quaternion originalWristRotation = isRightArm ? _originalRightWristRotation : _originalLeftWristRotation;

            // if (holdingWeapon && isTwoHanded)
            // {
            //     // TWO-HANDED: Mirror wrist flexion using right arm's swing progress
            //     float swingProgress = (_currentRightSwingYaw - minYaw) / (maxYaw - minYaw);
            //     swingProgress = (swingProgress - 0.5f) * 2f; // Convert to -1 to 1 range

            //     float wristThreshold = 0.7f; // Hardcoded threshold
            //     float targetFlexionAngle = 0f;

            //     if (isRightArm)
            //     {
            //         // Right wrist: ulnar when swinging out (positive), radial when swinging in (negative)
            //         if (swingProgress >= wristThreshold || swingProgress <= -wristThreshold)
            //             targetFlexionAngle = rightWristUlnarFlexion;
            //         else
            //             targetFlexionAngle = rightWristRadialFlexion;
            //     }
            //     else
            //     {
            //         // Left wrist: MIRROR the right wrist's behavior
            //         if (swingProgress >= wristThreshold || swingProgress <= -wristThreshold)
            //             targetFlexionAngle = leftWristUlnarFlexion;
            //         else
            //             targetFlexionAngle = leftWristRadialFlexion;
            //     }

            //     Quaternion flexion = Quaternion.Euler(0, targetFlexionAngle, 0);
            //     wristJoint.targetRotation = originalWristRotation * flexion;
            // }
            // else 
            if (!isTwoHanded)
            {
                // ONE-HANDED: Original wrist flexion logic
                float swingProgress = (normalizedYaw - 0.5f) * 2f;

                float targetFlexionAngle;
                if (isRightArm)
                {
                    if (swingProgress >= twistActivateThreshold || swingProgress <= -twistActivateThreshold)
                        targetFlexionAngle = rightWristUlnarFlexion;
                    else
                        targetFlexionAngle = rightWristRadialFlexion;
                }
                else
                {
                    if (swingProgress <= -twistActivateThreshold || swingProgress >= twistActivateThreshold)
                        targetFlexionAngle = leftWristUlnarFlexion;
                    else
                        targetFlexionAngle = leftWristRadialFlexion;
                }

                Quaternion flexion = Quaternion.Euler(0, targetFlexionAngle, 0);
                wristJoint.targetRotation = originalWristRotation * flexion;
            }
        }
        else if (wristJoint != null)
        {
            Quaternion originalWristRotation = isRightArm ? _originalRightWristRotation : _originalLeftWristRotation;
            wristJoint.targetRotation = originalWristRotation;
        }

        // STEP 8: Set hand orientation to face camera
        handIKTarget.rotation = Quaternion.LookRotation(_cameraTransform.forward, charUp);
    }

    private void MoveArmTowardsTarget(Rigidbody handRB, Transform handIKTarget, bool isRightHand)
    {
        float forceMultiplier = isRightHand ? _currentRightFollowForce : _currentLeftFollowForce;

        // ADDED: Reduce force for left hand in two-handed mode
        if (isTwoHandedMode && !isRightHand)
        {
            forceMultiplier *= 0.3f; // Only 30% force for secondary hand
        }

        Vector3 positionDifference = handIKTarget.position - handRB.position;
        handRB.AddForce(positionDifference * followForce * forceMultiplier, ForceMode.Force);

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

        float minYaw = isLeftArm ? -swingAngleOutward : -swingAngleAcrossBody;
        float maxYaw = isLeftArm ? swingAngleAcrossBody : swingAngleOutward;

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
}