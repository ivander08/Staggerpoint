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
    public Transform rightArmRoot; // Assign "Shoulder.R" bone

    // --- LEFT ARM ---
    [Header("Left Arm Rig")]
    public Rigidbody leftHandRigidbody;
    public Transform leftShoulderAnchor;
    public Transform leftArmRoot; // Assign "Shoulder.L" bone

    [Header("IK Targets")]
    public Transform rightHandIKTarget;
    public Transform leftHandIKTarget;

    [Header("Physics Control")]
    [Tooltip("The 'strength' of the pull. You've found 50000 to work well.")]
    public float followForce = 50000f; 
    [Tooltip("How 'snappy' the wrist rotation is. You've found 5000 to work well.")]
    public float rotateTorque = 5000f;

    [Header("Swing Control")]
    public float swingSensitivity = 1.5f; 
    public float swingRadius = 1.2f;
    public float swingAngleAcrossBody = 45f;
    public float swingAngleOutward = 90f;
    public float maxVerticalSwingAngle = 80f;

    // --- Private State ---
    private bool _isRightArmSwinging = false;
    private bool _isLeftArmSwinging = false;
    private float _originalBalanceForce, _originalBalanceDamper;
    private Transform _cameraTransform;

    private float _currentRightSwingYaw, _currentRightSwingPitch;
    private float _currentLeftSwingYaw, _currentLeftSwingPitch;

    private List<ConfigurableJoint> _rightArmJoints = new List<ConfigurableJoint>();
    private List<JointDrive> _originalRightArmDrives = new List<JointDrive>();
    private List<ConfigurableJoint> _leftArmJoints = new List<ConfigurableJoint>();
    private List<JointDrive> _originalLeftArmDrives = new List<JointDrive>();

    void Awake()
    {
        if (cameraController != null) _cameraTransform = cameraController.transform;
        else { Debug.LogError("CameraController not assigned!"); enabled = false; return; }
        
        // Uncap rotation speed for both hands
        rightHandRigidbody.maxAngularVelocity = 50f;
        leftHandRigidbody.maxAngularVelocity = 50f;

        // Initialize Right Arm
        InitializeArm(rightArmRoot, _rightArmJoints, _originalRightArmDrives);
        // Initialize Left Arm
        InitializeArm(leftArmRoot, _leftArmJoints, _originalLeftArmDrives);
        
        _originalBalanceForce = activeRagdoll.balanceForce;
        _originalBalanceDamper = activeRagdoll.balanceDamping;
    }

    private void InitializeArm(Transform armRoot, List<ConfigurableJoint> joints, List<JointDrive> drives)
    {
        if (armRoot != null)
        {
            joints.AddRange(armRoot.GetComponentsInChildren<ConfigurableJoint>());
            foreach(var joint in joints)
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
    }

    void FixedUpdate()
    {
        if (_isRightArmSwinging) MoveArmTowardsTarget(rightHandRigidbody, rightHandIKTarget);
        if (_isLeftArmSwinging) MoveArmTowardsTarget(leftHandRigidbody, leftHandIKTarget);
    }

    // --- Input & State Management ---

    private void HandleInput()
    {
        // Right Arm
        bool rightMousePressed = Mouse.current.rightButton.isPressed;
        if (rightMousePressed && !_isRightArmSwinging) StartSwing(true);
        else if (!rightMousePressed && _isRightArmSwinging) EndSwing(true);
        // Left Arm
        bool leftMousePressed = Mouse.current.leftButton.isPressed;
        if (leftMousePressed && !_isLeftArmSwinging) StartSwing(false);
        else if (!leftMousePressed && _isLeftArmSwinging) EndSwing(false);
    }

    private void StartSwing(bool isRightArm)
    {
        // Enter combat stance if not already in it
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
        else // Left Arm
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
        else // Left Arm
        {
            _isLeftArmSwinging = false;
            ReTenseArm(_leftArmJoints, _originalLeftArmDrives);
        }

        // Exit combat stance if both arms are now idle
        if (!_isRightArmSwinging && !_isLeftArmSwinging)
        {
            cameraController.SetLock(false);
            UnbraceTorso();
        }
    }

    // --- Helper Functions for Readability ---

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
        foreach(var joint in joints)
        {
            var relaxedDrive = new JointDrive { positionSpring = 0, positionDamper = 10, maximumForce = float.MaxValue };
            joint.angularXDrive = relaxedDrive;
            joint.angularYZDrive = relaxedDrive;
        }
    }

    private void ReTenseArm(List<ConfigurableJoint> joints, List<JointDrive> originalDrives)
    {
        // This correctly restores your custom 150/1 values, or whatever you set them to.
        for(int i = 0; i < joints.Count; i++)
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
            // Mirrored clamp for the left arm
            yaw = Mathf.Clamp(yaw, -swingAngleOutward, swingAngleAcrossBody);
        }
        else // Right Arm
        {
            yaw = Mathf.Clamp(yaw, -swingAngleAcrossBody, swingAngleOutward);
        }
        pitch = Mathf.Clamp(pitch, -maxVerticalSwingAngle, maxVerticalSwingAngle);

        Vector3 charUp = hips.up;
        Vector3 newHorizontalDir = Quaternion.AngleAxis(yaw, charUp) * hips.forward;
        Vector3 pitchRotationAxis = Vector3.Cross(newHorizontalDir, charUp).normalized;
        Vector3 newArmVector = Quaternion.AngleAxis(pitch, pitchRotationAxis) * newHorizontalDir;

        handIKTarget.position = shoulderAnchor.position + newArmVector.normalized * swingRadius;
        handIKTarget.rotation = Quaternion.LookRotation(_cameraTransform.forward, charUp);
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
}