using UnityEngine;
using UnityEngine.InputSystem;

namespace OldActiveRagdoll
{
    public class ActiveRagdollController : MonoBehaviour
    {
        // core references
        [Header("Core References")]
        [SerializeField] private Transform _animatedTorso;
        [SerializeField] private Rigidbody _physicalTorso;
        [SerializeField] private Animator _animatedAnimator;
        [SerializeField] private AnimatorIKHelper _ikHelper;
        [SerializeField] private Transform _cameraTransform;
        [SerializeField] private CameraController _cameraController;
        [SerializeField] private Rigidbody _stabilizerRigidbody;

        // movement stuff
        [Header("Movement Settings")]
        [SerializeField] private float _walkSpeedMultiplier = 1.5f;
        [SerializeField] private float _turnSpeed = 2f;

        // ik stuff
        [Header("IK Targets")]
        [SerializeField] private Transform _rightHandTarget;
        [SerializeField] private Transform _rightElbowHint;
        [SerializeField] private Transform _leftHandTarget;
        [SerializeField] private Transform _leftElbowHint;

        [Header("IK Control")]
        [SerializeField] private Transform _rightShoulderAnchor;
        [SerializeField] private Transform _leftShoulderAnchor;
        [SerializeField] private Transform _rightArmReadyTarget;
        [SerializeField] private Transform _leftArmReadyTarget;
        [SerializeField] private float _swingSensitivity = 1f;
        [SerializeField] private float _swingRadius = 1.2f;
        [SerializeField] private float _swingAngleAcrossBody = 45f;
        [SerializeField] private float _swingAngleOutward = 90f;
        [SerializeField] private float _maxVerticalSwingAngle = 80f;

        // private class
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
                rb.maxAngularVelocity = 50;
            }

            _initialJointsRotation = new Quaternion[_physicalJoints.Length];
            for (int i = 0; i < _physicalJoints.Length; i++)
            {
                _initialJointsRotation[i] = _physicalJoints[i].transform.localRotation;
            }

            foreach (ConfigurableJoint joint in _physicalJoints)
            {
                JointDrive drive = new JointDrive { maximumForce = 1000f };
                if (joint.transform.IsChildOf(transform.Find("Physical/metarig/Hips")) ||
                    joint.transform.IsChildOf(transform.Find("Physical/metarig/Hips/Spine")))
                {
                    drive.positionSpring = 20000f;
                    drive.positionDamper = 200f;
                }
                else if (joint.transform.IsChildOf(transform.Find("Physical/metarig/Hips/Thigh.L")) ||
                         joint.transform.IsChildOf(transform.Find("Physical/metarig/Hips/Thigh.R")))
                {
                    drive.positionSpring = 8000f;
                    drive.positionDamper = 500f;
                }
                else if (joint.transform.IsChildOf(transform.Find("Physical/metarig/Hips/Spine/Chest/Shoulder.L")) ||
                         joint.transform.IsChildOf(transform.Find("Physical/metarig/Hips/Spine/Chest/Shoulder.R")))
                {
                    drive.positionSpring = 12000f;
                    drive.positionDamper = 100f;
                }
                else if (joint.transform.IsChildOf(transform.Find("Physical/metarig/Hips/Spine/Chest/Neck")))
                {
                    drive.positionSpring = 5000f;
                    drive.positionDamper = 150f;
                }
                joint.angularXDrive = drive;
                joint.angularYZDrive = drive;
            }

            _speedParameterId = Animator.StringToHash("Speed");
        }

        void OnEnable()
        {
            _playerControls.Gameplay.Enable();
            _playerControls.Gameplay.RightArmIK.performed += ctx => { _rightArmIKActive = true; _justPressedRightArm = true; };
            _playerControls.Gameplay.RightArmIK.canceled += ctx => _rightArmIKActive = false;
            _playerControls.Gameplay.LeftArmIK.performed += ctx => { _leftArmIKActive = true; _justPressedLeftArm = true; };
            _playerControls.Gameplay.LeftArmIK.canceled += ctx => _leftArmIKActive = false;
        }

        void OnDisable()
        {
            _playerControls.Gameplay.Disable();
            _playerControls.Gameplay.RightArmIK.performed -= ctx => { _rightArmIKActive = true; _justPressedRightArm = true; };
            _playerControls.Gameplay.RightArmIK.canceled -= ctx => _rightArmIKActive = false;
            _playerControls.Gameplay.LeftArmIK.performed -= ctx => { _leftArmIKActive = true; _justPressedLeftArm = true; };
            _playerControls.Gameplay.LeftArmIK.canceled -= ctx => _leftArmIKActive = false;
        }

        void Update()
        {
            float verticalInput = _playerControls.Gameplay.Move.ReadValue<Vector2>().y;
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
            // make character face camera & move stabilizer
            Vector3 targetDirection = Vector3.ProjectOnPlane(_cameraTransform.forward, Vector3.up).normalized;
            Quaternion targetRotation = Quaternion.LookRotation(targetDirection);
            _stabilizerRigidbody.MoveRotation(targetRotation);

            // handle mouse
            Quaternion newRootRotation = Quaternion.Slerp(_rootRigidbody.rotation, targetRotation, Time.fixedDeltaTime * _turnSpeed);
            _rootRigidbody.MoveRotation(newRootRotation);

            // update root movement
            Vector3 positionDifference = _physicalTorso.position - _rootRigidbody.position;
            Vector3 velocityToTarget = positionDifference / Time.fixedDeltaTime;
            _rootRigidbody.velocity = velocityToTarget;

            SyncAnimatedBody();
            MatchAnimation();
        }

        private void MatchAnimation()
        {
            for (int i = 0; i < _physicalJoints.Length; i++)
            {
                // +1 cuz have root (Hips)
                Transform animatedBone = _animatedBones[i + 1];
                ConfigurableJointExtensions.SetTargetRotationLocal(_physicalJoints[i], animatedBone.localRotation, _initialJointsRotation[i]);
            }
        }

        void LateUpdate()
        {
            if (_ikHelper == null || _cameraTransform == null) return;

            if (_justPressedRightArm && _rightArmReadyTarget != null && _rightShoulderAnchor != null)
            {
                Vector3 readyDirection = (_rightArmReadyTarget.position - _rightShoulderAnchor.position).normalized;
                _rightHandTarget.position = _rightShoulderAnchor.position + readyDirection * _swingRadius;
                _justPressedRightArm = false;
            }

            if (_justPressedLeftArm && _leftArmReadyTarget != null && _leftShoulderAnchor != null)
            {
                Vector3 readyDirection = (_leftArmReadyTarget.position - _leftShoulderAnchor.position).normalized;
                _leftHandTarget.position = _leftShoulderAnchor.position + readyDirection * _swingRadius;
                _justPressedLeftArm = false;
            }

            Vector3 aimDirection = _cameraTransform.forward;
            _ikHelper.LookAtWeight = 0.0f;

            HandleRightArmIK(aimDirection);
            HandleLeftArmIK(aimDirection);
        }

        private void HandleRightArmIK(Vector3 aimDirection)
        {
            _ikHelper.RightHandWeight = _rightArmIKActive ? 1.0f : 0.0f;
            if (!_rightArmIKActive || _rightShoulderAnchor == null) return;

            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();
            Vector3 charUp = _physicalTorso.transform.up;

            Vector3 currentArmVector = _rightHandTarget.position - _rightShoulderAnchor.position;
            Vector3 armHorizontal = Vector3.ProjectOnPlane(currentArmVector, charUp);
            float currentYaw = Vector3.SignedAngle(Vector3.ProjectOnPlane(_physicalTorso.transform.forward, charUp), armHorizontal, charUp);
            float currentPitch = Vector3.Angle(armHorizontal, currentArmVector) * Mathf.Sign(Vector3.Dot(currentArmVector, charUp));

            float desiredYaw = currentYaw + mouseDelta.x * _swingSensitivity;
            float desiredPitch = currentPitch + mouseDelta.y * _swingSensitivity;

            float clampedYaw = Mathf.Clamp(desiredYaw, -_swingAngleAcrossBody, _swingAngleOutward);
            float clampedPitch = Mathf.Clamp(desiredPitch, -_maxVerticalSwingAngle, _maxVerticalSwingAngle);

            Vector3 newHorizontalDir = Quaternion.AngleAxis(clampedYaw, charUp) * _physicalTorso.transform.forward;
            Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
            Vector3 newArmVector = Quaternion.AngleAxis(clampedPitch, pitchRotationAxis) * newHorizontalDir;

            _rightHandTarget.position = _rightShoulderAnchor.position + newArmVector.normalized * _swingRadius;

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

            Vector2 mouseDelta = _playerControls.Gameplay.Look.ReadValue<Vector2>();
            Vector3 charUp = _physicalTorso.transform.up;

            Vector3 currentArmVector = _leftHandTarget.position - _leftShoulderAnchor.position;
            Vector3 armHorizontal = Vector3.ProjectOnPlane(currentArmVector, charUp);
            float currentYaw = Vector3.SignedAngle(Vector3.ProjectOnPlane(_physicalTorso.transform.forward, charUp), armHorizontal, charUp);
            float currentPitch = Vector3.Angle(armHorizontal, currentArmVector) * Mathf.Sign(Vector3.Dot(currentArmVector, charUp));

            float desiredYaw = currentYaw + mouseDelta.x * _swingSensitivity;
            float desiredPitch = currentPitch + mouseDelta.y * _swingSensitivity;

            float clampedYaw = Mathf.Clamp(desiredYaw, -_swingAngleOutward, _swingAngleAcrossBody);
            float clampedPitch = Mathf.Clamp(desiredPitch, -_maxVerticalSwingAngle, _maxVerticalSwingAngle);

            Vector3 newHorizontalDir = Quaternion.AngleAxis(clampedYaw, charUp) * _physicalTorso.transform.forward;
            Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
            Vector3 newArmVector = Quaternion.AngleAxis(clampedPitch, pitchRotationAxis) * newHorizontalDir;

            _leftHandTarget.position = _leftShoulderAnchor.position + newArmVector.normalized * _swingRadius;

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

            Vector3 charUp = _physicalTorso.transform.up;
            Vector3 charForward = Vector3.ProjectOnPlane(_physicalTorso.transform.forward, charUp).normalized;

            if (_rightShoulderAnchor != null && _rightArmIKActive)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawWireSphere(_rightShoulderAnchor.position, _swingRadius);
                Vector3 acrossBodyDir = Quaternion.AngleAxis(-_swingAngleAcrossBody, charUp) * charForward;
                Vector3 outwardDir = Quaternion.AngleAxis(_swingAngleOutward, charUp) * charForward;
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(_rightShoulderAnchor.position, _rightShoulderAnchor.position + acrossBodyDir * _swingRadius);
                Gizmos.DrawLine(_rightShoulderAnchor.position, _rightShoulderAnchor.position + outwardDir * _swingRadius);
                Gizmos.color = Color.cyan;
                Vector3 pitchAxis = Vector3.Cross(outwardDir, charUp);
                Vector3 lastArcPoint = Quaternion.AngleAxis(-_maxVerticalSwingAngle, pitchAxis) * outwardDir;
                for (int i = -80; i <= 80; i += 10)
                {
                    float angle = Mathf.Clamp(i, -_maxVerticalSwingAngle, _maxVerticalSwingAngle);
                    Vector3 newArcPoint = Quaternion.AngleAxis(angle, pitchAxis) * outwardDir;
                    Gizmos.DrawLine(_rightShoulderAnchor.position + lastArcPoint.normalized * _swingRadius,
                                   _rightShoulderAnchor.position + newArcPoint.normalized * _swingRadius);
                    lastArcPoint = newArcPoint;
                }
            }
            if (_leftShoulderAnchor != null && _leftArmIKActive)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(_leftShoulderAnchor.position, _swingRadius);
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