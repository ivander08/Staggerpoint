using UnityEngine;
using UnityEngine.InputSystem;

namespace ActiveRagdoll
{
    public class CameraController : MonoBehaviour
    {
        [Header("Target To Follow")]
        [SerializeField] private Transform target; // Drag your ActiveRagdoll's 'physicalTorso' here

        [Header("Camera Control")]
        [SerializeField] private float lookSensitivity = 1.0f;
        [SerializeField] private float smoothSpeed = 10.0f;
        [SerializeField] private float minVerticalAngle = -35.0f;
        [SerializeField] private float maxVerticalAngle = 60.0f;

        [Header("Positioning")]
        [SerializeField] private float distance = 1.0f;
        // This Vector3 now controls everything: X=side, Y=up, Z=forward offset from target
        [SerializeField] private Vector3 lookAtPointOffset = new Vector3(0.2f, 0.3f, 0); 

        [Header("Collision")]
        [SerializeField] private LayerMask collisionLayers;
        [SerializeField] private float collisionPadding = 0.25f;

        private PlayerControls playerControls;
        private Vector2 lookInput;
        private float currentYaw;
        private float currentPitch;

        // --- SETUP ---
        private void Awake()
        {
            playerControls = new PlayerControls();
            Cursor.lockState = CursorLockMode.Locked; // Good place for this
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

        // --- INPUT & LOGIC ---
        private void Update()
        {
            lookInput = playerControls.Gameplay.Look.ReadValue<Vector2>();
        }

        private void LateUpdate()
        {
            // 1. Calculate Rotation from Input
            currentYaw += lookInput.x * lookSensitivity * Time.deltaTime * 50f;
            currentPitch -= lookInput.y * lookSensitivity * Time.deltaTime * 50f;
            currentPitch = Mathf.Clamp(currentPitch, minVerticalAngle, maxVerticalAngle);

            // Create the camera's rotation quaternion
            Quaternion rotation = Quaternion.Euler(currentPitch, currentYaw, 0);

            // 2. Calculate the Look-At Point
            // Start at the target's position...
            Vector3 lookAtBase = target.position;
            // ...then apply our offset relative to the camera's rotation.
            // This is the point the camera will actually look at.
            Vector3 lookAtPoint = lookAtBase + (rotation * lookAtPointOffset);
            
            // 3. Calculate the Camera's Ideal Position
            // The camera simply wants to be 'distance' units behind the look-at point.
            Vector3 desiredPosition = lookAtPoint - (rotation * Vector3.forward * distance);

            // 4. Handle Collisions
            Vector3 finalPosition;
            RaycastHit hit;
            // We cast from the *base* of the look-at point to prevent the camera from jittering
            // if the offset point itself is inside a wall.
            if (Physics.Linecast(lookAtBase, desiredPosition, out hit, collisionLayers))
            {
                finalPosition = hit.point + hit.normal * collisionPadding;
            }
            else
            {
                finalPosition = desiredPosition;
            }

            // 5. Apply Position and Rotation
            transform.position = Vector3.Lerp(transform.position, finalPosition, smoothSpeed * Time.deltaTime);
            transform.LookAt(lookAtPoint); // Now we look at the offset point
        }

        private void ToggleShoulder()
        {
            // This function now just flips the sign of our X offset.
            lookAtPointOffset.x *= -1;
        }
    }
}