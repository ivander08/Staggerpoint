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
        [SerializeField] private float _swingSensitivity = 0.5f;
        [SerializeField] private float _swingRadius = 1.2f;

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

            // --- CORRECTED ROTATIONAL LOGIC ---
            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();

            // 1. Get the current arm direction.
            Vector3 currentArmVector = _rightHandTarget.position - _rightShoulderAnchor.position;

            // 2. Create rotations from mouse input.
            Quaternion yawRotation = Quaternion.AngleAxis(mouseDelta.x * _swingSensitivity, _physicalTorso.transform.up);
            Quaternion pitchRotation = Quaternion.AngleAxis(-mouseDelta.y * _swingSensitivity, _cameraTransform.right);

            // 3. Apply the rotations to the current arm vector.
            Vector3 newArmVector = yawRotation * pitchRotation * currentArmVector;

            // 4. *** THIS IS THE FIX ***
            //    Force the new vector to have the correct length (_swingRadius).
            newArmVector = newArmVector.normalized * _swingRadius;

            // 5. Set the new hand position. It is now guaranteed to be on the sphere.
            _rightHandTarget.position = _rightShoulderAnchor.position + newArmVector;
            // --- END OF CORRECTED LOGIC ---

            // The elbow hint logic remains the same.
            Vector3 midpoint = (_rightShoulderAnchor.position + _rightHandTarget.position) / 2;
            Vector3 armDirection = (_rightHandTarget.position - _rightShoulderAnchor.position).normalized;
            Vector3 elbowOutDirection = Vector3.Cross(_physicalTorso.transform.up, armDirection).normalized;
            _rightElbowHint.position = midpoint + elbowOutDirection * 0.3f;

            _ikHelper.RightHandPosition = _rightHandTarget.position;
            _ikHelper.RightHandRotation = Quaternion.LookRotation(aimDirection, _cameraTransform.up);
            _ikHelper.RightElbowHint = _rightElbowHint.position;
        }

        private void HandleLeftArmIK(Vector3 aimDirection)
        {
            _ikHelper.LeftHandWeight = _leftArmIKActive ? 1.0f : 0.0f;
            if (!_leftArmIKActive || _leftShoulderAnchor == null) return;

            // --- ROTATIONAL LOGIC for Left Arm ---
            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();

            // 1. Get the current arm direction.
            Vector3 currentArmVector = _leftHandTarget.position - _leftShoulderAnchor.position;

            // 2. Create rotations from mouse input.
            Quaternion yawRotation = Quaternion.AngleAxis(mouseDelta.x * _swingSensitivity, _physicalTorso.transform.up);
            Quaternion pitchRotation = Quaternion.AngleAxis(-mouseDelta.y * _swingSensitivity, _cameraTransform.right);

            // 3. Apply the rotations to the current arm vector.
            Vector3 newArmVector = yawRotation * pitchRotation * currentArmVector;

            // 4. Force the new vector to have the correct length (_swingRadius).
            newArmVector = newArmVector.normalized * _swingRadius;

            // 5. Set the new hand position.
            _leftHandTarget.position = _leftShoulderAnchor.position + newArmVector;
            // --- END OF ROTATIONAL LOGIC ---

            // The elbow hint logic remains the same.
            Vector3 midpoint = (_leftShoulderAnchor.position + _leftHandTarget.position) / 2;
            Vector3 armDirection = (_leftHandTarget.position - _leftShoulderAnchor.position).normalized;
            Vector3 elbowOutDirection = Vector3.Cross(_physicalTorso.transform.up, armDirection).normalized;

            // Use subtraction here to push the left elbow outwards correctly.
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
                Gizmos.color = Color.magenta;
                Gizmos.DrawWireSphere(_rightShoulderAnchor.position, _swingRadius);
            }

            if (_leftShoulderAnchor != null && _leftArmIKActive)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(_leftShoulderAnchor.position, _swingRadius);
            }
        }
    }
}