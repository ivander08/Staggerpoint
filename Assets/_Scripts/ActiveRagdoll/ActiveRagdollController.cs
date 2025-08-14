using UnityEngine;
using UnityEngine.InputSystem;

namespace ActiveRagdoll
{
    public class ActiveRagdollController : MonoBehaviour
    {
        // --- Inspector: Core Object References ---
        [Header("Core References")]
        [SerializeField] private Transform _stabilizer;
        [SerializeField] private Transform _animatedTorso;
        [SerializeField] private Rigidbody _physicalTorso;
        [SerializeField] private Animator _animatedAnimator;
        [SerializeField] private AnimatorIKHelper _ikHelper;
        [SerializeField] private Transform _cameraTransform;

        // --- Inspector: Movement & Turning ---
        [Header("Movement Settings")]
        [SerializeField] private float _walkSpeedMultiplier = 1.5f;
        [SerializeField] private float _turnSpeed = 2f;

        // --- Inspector: IK Targets & Settings ---
        [Header("IK Targets")]
        [SerializeField] private Transform _rightHandTarget;
        [SerializeField] private Transform _rightElbowHint;
        [SerializeField] private Transform _leftHandTarget;
        [SerializeField] private Transform _leftElbowHint;

        [Header("IK Control")]
        [SerializeField] private bool _limitAimAngle = true;
        [SerializeField] private float _maxAimAngle = 110f;

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

            _playerControls.Gameplay.RightArmIK.performed += ctx => _rightArmIKActive = true;
            _playerControls.Gameplay.RightArmIK.canceled += ctx => _rightArmIKActive = false;

            _playerControls.Gameplay.LeftArmIK.performed += ctx => _leftArmIKActive = true;
            _playerControls.Gameplay.LeftArmIK.canceled += ctx => _leftArmIKActive = false;
        }

        void OnDisable()
        {
            _playerControls.Gameplay.Disable();

            _playerControls.Gameplay.RightArmIK.performed -= ctx => _rightArmIKActive = true;
            _playerControls.Gameplay.RightArmIK.canceled -= ctx => _rightArmIKActive = false;

            _playerControls.Gameplay.LeftArmIK.performed -= ctx => _leftArmIKActive = true;
            _playerControls.Gameplay.LeftArmIK.canceled -= ctx => _leftArmIKActive = false;
        }

        void Update()
        {
            float verticalInput = Input.GetAxis("Vertical");
            _animatedAnimator.SetFloat(_speedParameterId, verticalInput * _walkSpeedMultiplier);
        }

        void FixedUpdate()
        {
            // Update the stabilizer to act as a balance and rotation guide.
            _stabilizer.GetComponent<Rigidbody>().MovePosition(_physicalTorso.position);
            _stabilizer.GetComponent<Rigidbody>().MoveRotation(transform.rotation);

            // Rotate the character controller based on player input.
            float horizontalInput = Input.GetAxis("Horizontal");
            transform.Rotate(0, horizontalInput * _turnSpeed, 0);

            // Pull the root controller along with the physical body's velocity.
            Vector3 positionDifference = _physicalTorso.position - _rootRigidbody.position;
            Vector3 velocityToTarget = positionDifference / Time.fixedDeltaTime;
            _rootRigidbody.velocity = velocityToTarget;

            // Keep the hidden animated body synced with the physical one for IK.
            SyncAnimatedBody();

            // The core motion matching logic.
            for (int i = 0; i < _physicalJoints.Length; i++)
            {
                // Use i+1 to account for the root bone offset in GetComponentsInChildren.
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

            Vector3 handTargetPosition = _cameraTransform.position
                                       + (_cameraTransform.right * _handTargetRightOffset)
                                       + (aimDirection * _handTargetForwardOffset);
            _rightHandTarget.position = handTargetPosition;

            _rightElbowHint.position = handTargetPosition
                                    - (aimDirection * _elbowHintForwardOffset)
                                    - (_cameraTransform.right * _elbowHintRightOffset);

            _ikHelper.RightHandPosition = _rightHandTarget.position;
            _ikHelper.RightHandRotation = Quaternion.LookRotation(aimDirection, _cameraTransform.up);
            _ikHelper.RightElbowHint = _rightElbowHint.position;
        }

        private void HandleLeftArmIK(Vector3 aimDirection)
        {
            _ikHelper.LeftHandWeight = _leftArmIKActive ? 1.0f : 0.0f;
            if (!_leftArmIKActive) return;

            Vector3 handTargetPosition = _cameraTransform.position
                                       + (-_cameraTransform.right * _leftHandRightOffset)
                                       + (aimDirection * _leftHandForwardOffset);
            _leftHandTarget.position = handTargetPosition;

            _leftElbowHint.position = handTargetPosition
                                    - (aimDirection * _leftElbowHintForwardOffset)
                                    + (_cameraTransform.right * _leftElbowHintRightOffset);

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

            // Draw physics/movement gizmos
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(_stabilizer.position, _stabilizer.forward * 2f);
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