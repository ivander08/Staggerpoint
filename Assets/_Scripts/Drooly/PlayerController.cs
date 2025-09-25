using UnityEngine;
using UnityEngine.InputSystem;

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
    private Transform _balanceTargetTransform;
    private Rigidbody _hipsRigidbody;

    // Input values
    private Vector2 _moveInput;

    void Awake()
    {
        _playerControls = new PlayerControls();
        
        if (activeRagdoll == null)
        {
            Debug.LogError("ActiveRagdoll reference not set on PlayerController!");
            this.enabled = false;
            return;
        }
        // Use the new, clearer variable names from ActiveRagdoll
        _balanceTargetTransform = activeRagdoll.balanceTargetBody.transform;
        _hipsRigidbody = activeRagdoll.hipsTransform.GetComponent<Rigidbody>();
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
        _moveInput = _playerControls.Gameplay.Move.ReadValue<Vector2>();
    }

    void FixedUpdate()
    {
        HandleRotation();
        HandleMovement();
    }

    private void HandleRotation()
    {
        Vector3 cameraForward = cameraTransform.forward;
        cameraForward.y = 0;

        if (cameraForward.magnitude > 0.1f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(cameraForward);
            _balanceTargetTransform.rotation = Quaternion.Slerp(_balanceTargetTransform.rotation, targetRotation, Time.fixedDeltaTime * rotationSpeed);
        }
    }

    private void HandleMovement()
    {
        Vector3 cameraForward = cameraTransform.forward;
        Vector3 cameraRight = cameraTransform.right;

        cameraForward.y = 0f;
        cameraRight.y = 0f;
        cameraForward.Normalize();
        cameraRight.Normalize();
        
        Vector3 targetVelocity = (cameraForward * _moveInput.y + cameraRight * _moveInput.x) * maxSpeed;
        
        Vector3 currentVelocity = _hipsRigidbody.velocity;
        currentVelocity.y = 0;
        
        Vector3 velocityChange = (targetVelocity - currentVelocity);
        
        velocityChange.x = Mathf.Clamp(velocityChange.x, -acceleration, acceleration);
        velocityChange.z = Mathf.Clamp(velocityChange.z, -acceleration, acceleration);
        
        _hipsRigidbody.AddForce(velocityChange, ForceMode.VelocityChange);
    }
}