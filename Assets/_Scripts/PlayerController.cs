using UnityEngine;
using UnityEngine.InputSystem;

// If your PlayerControls class is inside a namespace, you might need to add that namespace here too.
// For example: using OldActiveRagdoll;

public class PlayerController : MonoBehaviour
{
    [Header("Required Components")]
    public ActiveRagdoll activeRagdoll;
    public Transform cameraTransform;

    [Header("Movement Settings")]
    public float maxSpeed = 5.0f;
    public float acceleration = 25.0f;
    public float rotationSpeed = 15.0f;

    // Private references
    private PlayerControls _playerControls;
    private Transform _centerOfMass;
    private Rigidbody _hipsRb;

    // Input values
    private Vector2 _moveInput;

    void Awake()
    {
        // --- Setup Input ---
        _playerControls = new PlayerControls();

        // --- Setup Ragdoll References ---
        if (activeRagdoll == null)
        {
            Debug.LogError("ActiveRagdoll reference not set on PlayerController!");
            this.enabled = false;
            return;
        }
        _centerOfMass = activeRagdoll.connectedBody.transform;
        _hipsRb = activeRagdoll.rootBone.GetComponent<Rigidbody>();
    }

    void OnEnable()
    {
        _playerControls.Gameplay.Enable();
    }

    void OnDisable()
    {
        _playerControls.Gameplay.Disable();
    }

    void Update()
    {
        // Read movement input from the Move action
        _moveInput = _playerControls.Gameplay.Move.ReadValue<Vector2>();
    }

    void FixedUpdate()
    {
        HandleRotation();
        HandleMovement();
    }

    private void HandleRotation()
    {
        // The goal is to make the character's forward direction match the camera's forward direction.
        Vector3 cameraForward = cameraTransform.forward;
        cameraForward.y = 0; // Keep the character upright

        // If there is no movement input, the character should still face the camera's direction.
        if (cameraForward.magnitude > 0.1f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(cameraForward);
            _centerOfMass.rotation = Quaternion.Slerp(_centerOfMass.rotation, targetRotation, Time.fixedDeltaTime * rotationSpeed);
        }
    }

    private void HandleMovement()
    {
        // --- 1. Calculate Desired Velocity ---
        Vector3 cameraForward = cameraTransform.forward;
        Vector3 cameraRight = cameraTransform.right;

        cameraForward.y = 0f;
        cameraRight.y = 0f;
        cameraForward.Normalize();
        cameraRight.Normalize();

        // The velocity we WANT to have
        Vector3 targetVelocity = (cameraForward * _moveInput.y + cameraRight * _moveInput.x) * maxSpeed;

        // --- 2. Calculate the Necessary Change in Velocity ---
        // The velocity we currently HAVE
        Vector3 currentVelocity = _hipsRb.velocity;
        currentVelocity.y = 0; // We only care about horizontal movement

        // The difference we need to make up
        Vector3 velocityChange = (targetVelocity - currentVelocity);

        // --- 3. Apply the Force ---
        // Clamp the acceleration to not be too extreme
        velocityChange.x = Mathf.Clamp(velocityChange.x, -acceleration, acceleration);
        velocityChange.z = Mathf.Clamp(velocityChange.z, -acceleration, acceleration);

        // Apply the force using ForceMode.VelocityChange.
        // This mode ignores the Rigidbody's mass and applies an immediate velocity change.
        _hipsRb.AddForce(velocityChange, ForceMode.VelocityChange);
    }
}