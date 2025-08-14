using UnityEngine;
using UnityEngine.InputSystem;

namespace ActiveRagdoll
{
    public class CameraController : MonoBehaviour
    {
        [Header("Target To Follow")]
        [Tooltip("Drag your ActiveRagdoll's 'physicalTorso' here")]
        [SerializeField] private Transform target;

        [Header("Camera Control")]
        [SerializeField] private float lookSensitivity = 1.0f;
        [SerializeField] private float smoothSpeed = 10.0f;
        [SerializeField] private float minVerticalAngle = -35.0f;
        [SerializeField] private float maxVerticalAngle = 60.0f;

        [Header("Positioning")]
        [SerializeField] private float distance = 1.0f;
        [Tooltip("This Vector3 now controls everything: X=side, Y=up, Z=forward offset from target")]
        [SerializeField] private Vector3 lookAtPointOffset = new Vector3(0.2f, 0.3f, 0);

        [Header("Collision")]
        [SerializeField] private LayerMask collisionLayers;
        [SerializeField] private float collisionPadding = 0.25f;

        private PlayerControls playerControls;
        private Vector2 lookInput;
        private float currentYaw;
        private float currentPitch;

        private void Awake()
        {
            // 1. Set up the input system.
            playerControls = new PlayerControls();
            // 2. Lock the cursor to the center of the screen.
            Cursor.lockState = CursorLockMode.Locked;
        }

        private void OnEnable()
        {
            playerControls.Gameplay.Enable();
            playerControls.Gameplay.ShoulderToggle.performed += _ => ToggleShoulder();
        }

        private void OnDisable()
        {
            playerControls.Gameplay.ShoulderToggle.performed -= _ => ToggleShoulder();
            playerControls.Gameplay.Disable();
        }

        private void Update()
        {
            // 1. Read the look input (mouse movement).
            lookInput = playerControls.Gameplay.Look.ReadValue<Vector2>();
        }

        private void LateUpdate()
        {
            // 1. Calculate the camera's rotation from input.
            currentYaw += lookInput.x * lookSensitivity * Time.deltaTime * 50f;
            currentPitch -= lookInput.y * lookSensitivity * Time.deltaTime * 50f;
            currentPitch = Mathf.Clamp(currentPitch, minVerticalAngle, maxVerticalAngle);
            Quaternion rotation = Quaternion.Euler(currentPitch, currentYaw, 0);

            // 2. Determine the point in space the camera should look at.
            Vector3 lookAtBase = target.position;
            Vector3 lookAtPoint = lookAtBase + (rotation * lookAtPointOffset);

            // 3. Calculate the camera's ideal position behind the look-at point.
            Vector3 desiredPosition = lookAtPoint - (rotation * Vector3.forward * distance);

            // 4. Check for collisions and adjust the camera's position to avoid clipping.
            Vector3 finalPosition;
            RaycastHit hit;
            if (Physics.Linecast(lookAtBase, desiredPosition, out hit, collisionLayers))
            {
                finalPosition = hit.point + hit.normal * collisionPadding;
            }
            else
            {
                finalPosition = desiredPosition;
            }

            // 5. Smoothly move the camera to the final position and make it look at the target.
            transform.position = Vector3.Lerp(transform.position, finalPosition, smoothSpeed * Time.deltaTime);
            transform.LookAt(lookAtPoint);
        }

        private void ToggleShoulder()
        {
            // 1. Flip the camera's horizontal offset to switch shoulders.
            lookAtPointOffset.x *= -1;
        }
    }
}