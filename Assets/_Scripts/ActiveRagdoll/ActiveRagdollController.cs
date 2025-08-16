using UnityEngine;
using UnityEngine.InputSystem;

namespace ActiveRagdoll
{
    public class ActiveRagdollController : MonoBehaviour
    {
        // --- Inspector: Core Object References ---
        [Header("Core References")]
        [SerializeField] private Transform _animatedTorso;
        [SerializeField] private Rigidbody _physicalTorso;
        [SerializeField] private Animator _animatedAnimator;
        [SerializeField] private AnimatorIKHelper _ikHelper;
        [SerializeField] private Transform _cameraTransform;
        [SerializeField] private CameraController _cameraController;

        // --- Inspector: Movement & Turning ---
        [Header("Movement Settings")]
        [SerializeField] private float _walkSpeedMultiplier = 1.5f;
        [SerializeField] private float _turnSpeed = 2f;

        [Header("Manual Torque Balance")]
        [SerializeField] private float _uprightTorque = 20000f;
        [SerializeField] private float _rotationTorque = 500f;
        [SerializeField] private AnimationCurve _uprightTorqueFunction;
        [SerializeField] private float _customTorsoAngularDrag = 0.05f;

        // --- Inspector: IK Targets & Settings ---
        [Header("IK Targets")]
        [SerializeField] private Transform _rightHandTarget;
        [SerializeField] private Transform _rightElbowHint;
        [SerializeField] private Transform _leftHandTarget;
        [SerializeField] private Transform _leftElbowHint;

        [Header("IK Control")]
        [SerializeField] private Transform _rightShoulderAnchor;
        [SerializeField] private Transform _leftShoulderAnchor;
        [SerializeField] private Transform _rightArmReadyTarget; // ADD THIS
        [SerializeField] private Transform _leftArmReadyTarget;  // ADD THIS
        [SerializeField] private float _swingSensitivity = 1f; // A value of 1 might be a better default now
        [SerializeField] private float _swingRadius = 1.2f;
        [SerializeField] private float _swingAngleAcrossBody = 45f; // ADD THIS
        [SerializeField] private float _swingAngleOutward = 90f;   // ADD THIS
        [SerializeField] private float _maxVerticalSwingAngle = 80f;

        // --- Private Class Members ---
        private PlayerControls _playerControls;
        private Rigidbody _rootRigidbody;
        private ConfigurableJoint[] _physicalJoints;
        private Transform[] _animatedBones;
        private Quaternion[] _initialJointsRotation;
        private int _speedParameterId;
        private bool _rightArmIKActive = false;
        private bool _leftArmIKActive = false;
        private bool _justPressedRightArm = false;
        private bool _justPressedLeftArm = false;

        void Awake()
        {
            _playerControls = new PlayerControls();
        }

        void Start()
        {
            _rootRigidbody = GetComponent<Rigidbody>();

            _physicalJoints = _physicalTorso.GetComponentsInChildren<ConfigurableJoint>();
            _animatedBones = _animatedTorso.GetComponentsInChildren<Transform>();

            var allRigidbodies = _physicalTorso.GetComponentsInChildren<Rigidbody>();
            foreach (Rigidbody rb in allRigidbodies)
            {
                // Prevents physics from clamping the ragdoll's rotation speed.
                rb.maxAngularVelocity = 50;
            }

            _initialJointsRotation = new Quaternion[_physicalJoints.Length];
            for (int i = 0; i < _physicalJoints.Length; i++)
            {
                // Cache the default rotation of each joint for motion matching.
                _initialJointsRotation[i] = _physicalJoints[i].transform.localRotation;
            }

            _speedParameterId = Animator.StringToHash("Speed");
        }

        void OnEnable()
        {
            _playerControls.Gameplay.Enable();

            // When the RightArmIK action is first performed (left-click), set both flags to true.
            _playerControls.Gameplay.RightArmIK.performed += ctx => { _rightArmIKActive = true; _justPressedRightArm = true; };
            _playerControls.Gameplay.RightArmIK.canceled += ctx => _rightArmIKActive = false;

            // Do the same for the Left Arm IK (right-click).
            _playerControls.Gameplay.LeftArmIK.performed += ctx => { _leftArmIKActive = true; _justPressedLeftArm = true; };
            _playerControls.Gameplay.LeftArmIK.canceled += ctx => _leftArmIKActive = false;
        }

        void OnDisable()
        {
            _playerControls.Gameplay.Disable();

            // Make sure to update the unsubscription to match the new format.
            _playerControls.Gameplay.RightArmIK.performed -= ctx => { _rightArmIKActive = true; _justPressedRightArm = true; };
            _playerControls.Gameplay.RightArmIK.canceled -= ctx => _rightArmIKActive = false;

            _playerControls.Gameplay.LeftArmIK.performed -= ctx => { _leftArmIKActive = true; _justPressedLeftArm = true; };
            _playerControls.Gameplay.LeftArmIK.canceled -= ctx => _leftArmIKActive = false;
        }

        void Update()
        {
            float verticalInput = Input.GetAxis("Vertical");
            _animatedAnimator.SetFloat(_speedParameterId, verticalInput * _walkSpeedMultiplier);

            if (_cameraController != null)
            {
                if (_rightArmIKActive || _leftArmIKActive)
                {
                    _cameraController.SetLock(true);
                }
                else
                {
                    _cameraController.SetLock(false);
                }
            }
        }

        void FixedUpdate()
        {
            // --- STEP 1: APPLY STABILITY FORCES ---
            ApplyCustomDrag();

            // The character should always try to face where the camera is looking.
            // We flatten this direction onto the horizontal plane.
            Vector3 targetDirection = Vector3.ProjectOnPlane(_cameraTransform.forward, Vector3.up).normalized;

            // --- Upright Torque (Prevents Falling) ---
            float tiltPercent = Vector3.Angle(_physicalTorso.transform.up, Vector3.up) / 180f;
            float forceMultiplier = _uprightTorqueFunction.Evaluate(tiltPercent);
            var rot = Quaternion.FromToRotation(_physicalTorso.transform.up, Vector3.up);
            var torque = new Vector3(rot.x, rot.y, rot.z) * (_uprightTorque * forceMultiplier);
            _physicalTorso.AddTorque(torque);

            // --- Rotation Torque (Handles Turning) ---
            // This torque now makes the physical body turn towards the camera's direction.
            float directionAnglePercent = Vector3.SignedAngle(_physicalTorso.transform.forward, targetDirection, Vector3.up) / 180f;
            _physicalTorso.AddRelativeTorque(0, directionAnglePercent * _rotationTorque, 0);


            // --- STEP 2: HANDLE MOUSE-DRIVEN ROOT TURNING ---
            // The root controller also smoothly turns to align with the camera.
            Quaternion targetRotation = Quaternion.LookRotation(targetDirection);
            Quaternion newRotation = Quaternion.Slerp(_rootRigidbody.rotation, targetRotation, Time.fixedDeltaTime * _turnSpeed);
            _rootRigidbody.MoveRotation(newRotation);


            // --- STEP 3: UPDATE ROOT MOVEMENT & SYNC ---
            // This logic remains the same.
            Vector3 positionDifference = _physicalTorso.position - _rootRigidbody.position;
            Vector3 velocityToTarget = positionDifference / Time.fixedDeltaTime;
            _rootRigidbody.velocity = velocityToTarget;

            SyncAnimatedBody();
            MatchAnimation(); // I've created a helper function for clarity
        }

        private void MatchAnimation()
        {
            for (int i = 0; i < _physicalJoints.Length; i++)
            {
                Transform animatedBone = _animatedBones[i + 1];
                ConfigurableJointExtensions.SetTargetRotationLocal(_physicalJoints[i], animatedBone.localRotation, _initialJointsRotation[i]);
            }
        }

        void LateUpdate()
        {
            if (_ikHelper == null || _cameraTransform == null) return;

            // --- READY POSITION LOGIC ---
            if (_justPressedRightArm && _rightArmReadyTarget != null && _rightShoulderAnchor != null)
            {
                Vector3 readyDirection = (_rightArmReadyTarget.position - _rightShoulderAnchor.position).normalized;
                _rightHandTarget.position = _rightShoulderAnchor.position + readyDirection * _swingRadius;
                _justPressedRightArm = false; // Consume the flag
            }

            if (_justPressedLeftArm && _leftArmReadyTarget != null && _leftShoulderAnchor != null)
            {
                Vector3 readyDirection = (_leftArmReadyTarget.position - _leftShoulderAnchor.position).normalized;
                _leftHandTarget.position = _leftShoulderAnchor.position + readyDirection * _swingRadius;
                _justPressedLeftArm = false; // Consume the flag
            }

            // The aim direction for the hands is now simply the camera's forward direction.
            Vector3 aimDirection = _cameraTransform.forward;

            // Disable Head IK by setting its weight to zero.
            _ikHelper.LookAtWeight = 0.0f;

            // Update Right Arm IK
            HandleRightArmIK(aimDirection);

            // Update Left Arm IK
            HandleLeftArmIK(aimDirection);
        }

        private void ApplyCustomDrag()
        {
            // This adds a custom, velocity-dependent drag to the torso.
            var angVel = _physicalTorso.angularVelocity;
            angVel -= (Mathf.Pow(angVel.magnitude, 2) * _customTorsoAngularDrag) * angVel.normalized;
            _physicalTorso.angularVelocity = angVel;
        }

        private void HandleRightArmIK(Vector3 aimDirection)
        {
            _ikHelper.RightHandWeight = _rightArmIKActive ? 1.0f : 0.0f;
            if (!_rightArmIKActive || _rightShoulderAnchor == null) return;

            // --- NEW DECOUPLED AXIS LOGIC ---
            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();
            Vector3 charUp = _physicalTorso.transform.up;

            // 1. DECONSTRUCT: Find the current angles of the arm.
            Vector3 currentArmVector = _rightHandTarget.position - _rightShoulderAnchor.position;
            Vector3 armHorizontal = Vector3.ProjectOnPlane(currentArmVector, charUp);

            float currentYaw = Vector3.SignedAngle(Vector3.ProjectOnPlane(_physicalTorso.transform.forward, charUp), armHorizontal, charUp);
            float currentPitch = Vector3.Angle(armHorizontal, currentArmVector) * Mathf.Sign(Vector3.Dot(currentArmVector, charUp));

            // 2. APPLY INPUT: Add mouse input to the current angles.
            float desiredYaw = currentYaw + mouseDelta.x * _swingSensitivity;
            // *** THIS IS THE FIX: Changed - to + ***
            float desiredPitch = currentPitch + mouseDelta.y * _swingSensitivity;

            // 3. CLAMP: Clamp each angle independently.
            float clampedYaw = Mathf.Clamp(desiredYaw, -_swingAngleAcrossBody, _swingAngleOutward);
            float clampedPitch = Mathf.Clamp(desiredPitch, -_maxVerticalSwingAngle, _maxVerticalSwingAngle);

            // 4. RECONSTRUCT: Build the new arm vector from the clamped angles.
            Vector3 newHorizontalDir = Quaternion.AngleAxis(clampedYaw, charUp) * _physicalTorso.transform.forward;
            // To prevent flipping, we need a stable pitch axis. Using the camera's right is more reliable here.
            Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
            Vector3 newArmVector = Quaternion.AngleAxis(clampedPitch, pitchRotationAxis) * newHorizontalDir;

            // Set the final position, ensuring it has the correct radius.
            _rightHandTarget.position = _rightShoulderAnchor.position + newArmVector.normalized * _swingRadius;

            // The elbow hint logic remains the same.
            Vector3 midpoint = (_rightShoulderAnchor.position + _rightHandTarget.position) / 2;
            Vector3 armDirection = (_rightHandTarget.position - _rightShoulderAnchor.position).normalized;
            Vector3 elbowOutDirection = Vector3.Cross(charUp, armDirection).normalized;
            _rightElbowHint.position = midpoint + elbowOutDirection * 0.3f;

            _ikHelper.RightHandPosition = _rightHandTarget.position;
            _ikHelper.RightHandRotation = Quaternion.LookRotation(aimDirection, _cameraTransform.up);
            _ikHelper.RightElbowHint = _rightElbowHint.position;
        }

        private void HandleLeftArmIK(Vector3 aimDirection)
        {
            _ikHelper.LeftHandWeight = _leftArmIKActive ? 1.0f : 0.0f;
            if (!_leftArmIKActive || _leftShoulderAnchor == null) return;

            // --- DECOUPLED AXIS LOGIC for Left Arm ---
            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();
            Vector3 charUp = _physicalTorso.transform.up;

            // 1. DECONSTRUCT
            Vector3 currentArmVector = _leftHandTarget.position - _leftShoulderAnchor.position;
            Vector3 armHorizontal = Vector3.ProjectOnPlane(currentArmVector, charUp);

            float currentYaw = Vector3.SignedAngle(Vector3.ProjectOnPlane(_physicalTorso.transform.forward, charUp), armHorizontal, charUp);
            float currentPitch = Vector3.Angle(armHorizontal, currentArmVector) * Mathf.Sign(Vector3.Dot(currentArmVector, charUp));

            // 2. APPLY INPUT
            float desiredYaw = currentYaw + mouseDelta.x * _swingSensitivity;
            float desiredPitch = currentPitch + mouseDelta.y * _swingSensitivity;

            // 3. CLAMP (Note the inverted Yaw limits for the left arm)
            float clampedYaw = Mathf.Clamp(desiredYaw, -_swingAngleOutward, _swingAngleAcrossBody);
            float clampedPitch = Mathf.Clamp(desiredPitch, -_maxVerticalSwingAngle, _maxVerticalSwingAngle);

            // 4. RECONSTRUCT
            Vector3 newHorizontalDir = Quaternion.AngleAxis(clampedYaw, charUp) * _physicalTorso.transform.forward;
            Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
            Vector3 newArmVector = Quaternion.AngleAxis(clampedPitch, pitchRotationAxis) * newHorizontalDir;

            _leftHandTarget.position = _leftShoulderAnchor.position + newArmVector.normalized * _swingRadius;

            // Elbow hint logic
            Vector3 midpoint = (_leftShoulderAnchor.position + _leftHandTarget.position) / 2;
            Vector3 armDirection = (_leftHandTarget.position - _leftShoulderAnchor.position).normalized;
            Vector3 elbowOutDirection = Vector3.Cross(charUp, armDirection).normalized;
            _leftElbowHint.position = midpoint - elbowOutDirection * 0.3f;

            _ikHelper.LeftHandPosition = _leftHandTarget.position;
            _ikHelper.LeftHandRotation = Quaternion.LookRotation(aimDirection, _cameraTransform.up);
            _ikHelper.LeftElbowHint = _leftElbowHint.position;
        }

        private void SyncAnimatedBody()
        {
            Vector3 animatedOffset = _animatedAnimator.transform.position - _animatedTorso.position;
            _animatedAnimator.transform.position = _physicalTorso.position + animatedOffset;
            _animatedAnimator.transform.rotation = _physicalTorso.rotation;
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;

            Gizmos.color = Color.red;
            Gizmos.DrawRay(_physicalTorso.position, _physicalTorso.transform.forward * 1.5f);
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(_rootRigidbody.position, _physicalTorso.position);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(_rootRigidbody.position, _rootRigidbody.velocity);

            // Draw IK target gizmos if they exist
            if (_rightHandTarget)
            {
                Gizmos.color = _rightArmIKActive ? Color.magenta : Color.gray;
                Gizmos.DrawWireSphere(_rightHandTarget.position, 0.1f);
            }
            if (_rightElbowHint)
            {
                Gizmos.color = _rightArmIKActive ? new Color(1.0f, 0.5f, 0.0f) : Color.gray;
                Gizmos.DrawWireSphere(_rightElbowHint.position, 0.07f);
            }
            if (_leftHandTarget)
            {
                Gizmos.color = _leftArmIKActive ? Color.cyan : Color.gray;
                Gizmos.DrawWireSphere(_leftHandTarget.position, 0.1f);
            }
            if (_leftElbowHint)
            {
                Gizmos.color = _leftArmIKActive ? new Color(0.5f, 0.0f, 0.5f) : Color.gray;
                Gizmos.DrawWireSphere(_leftElbowHint.position, 0.07f);
            }

            if (_rightShoulderAnchor != null && _rightArmIKActive)
            {
                // Draw the swing sphere
                Gizmos.color = Color.magenta;
                Gizmos.DrawWireSphere(_rightShoulderAnchor.position, _swingRadius);

                // Draw the swing angle limits
                Vector3 charUp = _physicalTorso.transform.up;
                Vector3 charForward = Vector3.ProjectOnPlane(_physicalTorso.transform.forward, charUp).normalized;
                Vector3 acrossBodyDir = Quaternion.AngleAxis(-_swingAngleAcrossBody, charUp) * charForward;
                Vector3 outwardDir = Quaternion.AngleAxis(_swingAngleOutward, charUp) * charForward;
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(_rightShoulderAnchor.position, _rightShoulderAnchor.position + acrossBodyDir * _swingRadius);
                Gizmos.DrawLine(_rightShoulderAnchor.position, _rightShoulderAnchor.position + outwardDir * _swingRadius);

                Gizmos.color = Color.cyan;
                Vector3 lastArcPoint = Quaternion.AngleAxis(-_maxVerticalSwingAngle, Vector3.Cross(outwardDir, charUp)) * outwardDir;
                for (int i = -80; i <= 80; i += 10)
                {
                    float angle = Mathf.Clamp(i, -_maxVerticalSwingAngle, _maxVerticalSwingAngle);
                    Vector3 newArcPoint = Quaternion.AngleAxis(angle, Vector3.Cross(outwardDir, charUp)) * outwardDir;
                    Gizmos.DrawLine(_rightShoulderAnchor.position + lastArcPoint.normalized * _swingRadius,
                                   _rightShoulderAnchor.position + newArcPoint.normalized * _swingRadius);
                    lastArcPoint = newArcPoint;
                }
            }

            if (_leftShoulderAnchor != null && _leftArmIKActive)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(_leftShoulderAnchor.position, _swingRadius);

                Vector3 charUp = _physicalTorso.transform.up;
                Vector3 charForward = Vector3.ProjectOnPlane(_physicalTorso.transform.forward, charUp).normalized;
                // Note the swapped angles here for the left arm's visualization
                Vector3 acrossBodyDir = Quaternion.AngleAxis(_swingAngleAcrossBody, charUp) * charForward;
                Vector3 outwardDir = Quaternion.AngleAxis(-_swingAngleOutward, charUp) * charForward;

                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(_leftShoulderAnchor.position, _leftShoulderAnchor.position + acrossBodyDir * _swingRadius);
                Gizmos.DrawLine(_leftShoulderAnchor.position, _leftShoulderAnchor.position + outwardDir * _swingRadius);

                Gizmos.color = Color.magenta;
                Vector3 pitchAxis = Vector3.Cross(outwardDir, charUp);
                Vector3 lastArcPoint = Quaternion.AngleAxis(-_maxVerticalSwingAngle, pitchAxis) * outwardDir;
                for (int i = -80; i <= 80; i += 10)
                {
                    float angle = Mathf.Clamp(i, -_maxVerticalSwingAngle, _maxVerticalSwingAngle);
                    Vector3 newArcPoint = Quaternion.AngleAxis(angle, pitchAxis) * outwardDir;
                    Gizmos.DrawLine(_leftShoulderAnchor.position + lastArcPoint.normalized * _swingRadius,
                                   _leftShoulderAnchor.position + newArcPoint.normalized * _swingRadius);
                    lastArcPoint = newArcPoint;
                }
            }
        }
    }
}