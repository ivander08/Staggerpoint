using UnityEngine;

public class PlayerMovementController : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float rotationSpeed = 10f;

    private Rigidbody rb;
    private PlayerControls playerControls;
    private Vector2 moveInput;
    private Transform cameraTransform;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        playerControls = new PlayerControls();
        cameraTransform = Camera.main.transform;
    }

    private void OnEnable()
    {
        playerControls.Gameplay.Move.Enable();
        playerControls.Gameplay.Move.performed += ctx => OnMove(ctx.ReadValue<Vector2>());
        playerControls.Gameplay.Move.canceled += ctx => OnMove(ctx.ReadValue<Vector2>());
    }

    private void OnDisable()
    {
        playerControls.Gameplay.Move.Disable();
    }

    private void OnMove(Vector2 input)
    {
        moveInput = input;
    }

    private void FixedUpdate()
    {
        Vector3 moveDirection = cameraTransform.forward * moveInput.y + cameraTransform.right * moveInput.x;
        moveDirection.y = 0f;
        moveDirection.Normalize();

        Vector3 newVelocity = moveDirection * moveSpeed;
        rb.velocity = new Vector3(newVelocity.x, rb.velocity.y, newVelocity.z);

        if (moveDirection != Vector3.zero)
        {
            Quaternion targetRotation = Quaternion.LookRotation(moveDirection);
            Quaternion smoothedRotation = Quaternion.Slerp(rb.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);
            rb.MoveRotation(smoothedRotation);
        }
    }
}