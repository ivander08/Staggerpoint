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
        [SerializeField] private bool _limitAimAngle = true;
        [SerializeField] private float _maxAimAngle = 110f;
        [SerializeField] private float _swingSensitivity = 0.5f;
        [SerializeField] private float _maxArmReach = 1.2f;

        // --- Inspector: IK Positional Offsets ---
        [Header("Right Hand IK Offsets")]
        [SerializeField] private float _handTargetForwardOffset = 1.35f;
        [SerializeField] private float _handTargetRightOffset = 0f;
        [SerializeField] private float _elbowHintForwardOffset = 0.2f;
        [SerializeField] private float _elbowHintRightOffset = 0f;

        [Header("Left Hand IK Offsets")]
        [SerializeField] private float _leftHandForwardOffset = 1.5f;
        [SerializeField] private float _leftHandRightOffset = 0.35f;
        [SerializeField] private float _leftElbowHintForwardOffset = 0.2f;
        [SerializeField] private float _leftElbowHintRightOffset = 0f;

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

            // --- Define a shared aim direction for all IK ---
            Vector3 aimDirection = CalculateClampedAimDirection();

            // --- Update Head IK ---
            _ikHelper.LookAtWeight = 1.0f;
            _ikHelper.LookAtPoint = _cameraTransform.position + (aimDirection * 10f);

            // --- Update Right Arm IK ---
            HandleRightArmIK(aimDirection);

            // --- Update Left Arm IK ---
            HandleLeftArmIK(aimDirection);
        }

        private void ApplyCustomDrag()
        {
            // This adds a custom, velocity-dependent drag to the torso.
            var angVel = _physicalTorso.angularVelocity;
            angVel -= (Mathf.Pow(angVel.magnitude, 2) * _customTorsoAngularDrag) * angVel.normalized;
            _physicalTorso.angularVelocity = angVel;
        }

        private Vector3 CalculateClampedAimDirection()
        {
            Vector3 cameraDirection = _cameraTransform.forward;
            if (!_limitAimAngle)
            {
                return cameraDirection;
            }

            Vector3 characterForward = Vector3.ProjectOnPlane(_physicalTorso.transform.forward, Vector3.up).normalized;
            Vector3 horizontalCameraDirection = Vector3.ProjectOnPlane(cameraDirection, Vector3.up).normalized;
            float signedAngle = Vector3.SignedAngle(characterForward, horizontalCameraDirection, Vector3.up);

            Vector3 finalDirection;

            if (Mathf.Abs(signedAngle) > _maxAimAngle)
            {

                float sign = Mathf.Sign(signedAngle);
                Quaternion clampedRotation = Quaternion.AngleAxis(_maxAimAngle * sign, Vector3.up);
                Vector3 clampedFlatDirection = clampedRotation * characterForward;

                finalDirection = new Vector3(clampedFlatDirection.x, cameraDirection.y, clampedFlatDirection.z);

                finalDirection.Normalize();
            }
            else
            {
                // If we are within the limit, use the original camera direction.
                finalDirection = cameraDirection;
            }

            return finalDirection;
        }

        private void HandleRightArmIK(Vector3 aimDirection)
        {
            _ikHelper.RightHandWeight = _rightArmIKActive ? 1.0f : 0.0f;
            if (!_rightArmIKActive) return;

            // Move the hand based on mouse input.
            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();
            Vector3 movement = (_cameraTransform.right * mouseDelta.x + _cameraTransform.up * mouseDelta.y) * _swingSensitivity * Time.deltaTime;
            Vector3 goalPosition = _rightHandTarget.position + movement;

            // Apply the physical leash.
            Vector3 anchorPoint = _physicalTorso.position;
            Vector3 vectorFromAnchor = goalPosition - anchorPoint;
            if (vectorFromAnchor.magnitude > _maxArmReach)
            {
                goalPosition = anchorPoint + vectorFromAnchor.normalized * _maxArmReach;
            }
            _rightHandTarget.position = goalPosition;

            // A simple, predictable elbow hint position.
            _rightElbowHint.position = _rightHandTarget.position
                                     - (_cameraTransform.forward * 0.5f)
                                     - (_cameraTransform.right * 0.5f);

            _ikHelper.RightHandPosition = _rightHandTarget.position;
            _ikHelper.RightHandRotation = Quaternion.LookRotation(aimDirection, _cameraTransform.up);
            _ikHelper.RightElbowHint = _rightElbowHint.position;
        }

        private void HandleLeftArmIK(Vector3 aimDirection)
        {
            _ikHelper.LeftHandWeight = _leftArmIKActive ? 1.0f : 0.0f;
            if (!_leftArmIKActive) return;

            // Move the hand based on mouse input.
            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();
            Vector3 movement = (_cameraTransform.right * mouseDelta.x + _cameraTransform.up * mouseDelta.y) * _swingSensitivity * Time.deltaTime;
            Vector3 goalPosition = _leftHandTarget.position + movement;

            // Apply the physical leash.
            Vector3 anchorPoint = _physicalTorso.position;
            Vector3 vectorFromAnchor = goalPosition - anchorPoint;
            if (vectorFromAnchor.magnitude > _maxArmReach)
            {
                goalPosition = anchorPoint + vectorFromAnchor.normalized * _maxArmReach;
            }
            _leftHandTarget.position = goalPosition;

            // A simple, predictable elbow hint position for the left arm.
            _leftElbowHint.position = _leftHandTarget.position
                                    - (_cameraTransform.forward * 0.5f)
                                    + (_cameraTransform.right * 0.5f);

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
        }
    }
}