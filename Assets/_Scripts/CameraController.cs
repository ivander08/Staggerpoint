using UnityEngine;
using UnityEngine.InputSystem;

namespace ActiveRagdoll
{
    public class CameraController : MonoBehaviour
    {
        // --- Inspector: Core References & Settings ---
        [Header("Target To Follow")]
        [SerializeField] private Transform _target;
        [SerializeField] private Vector3 _lookAtPointOffset = new Vector3(0.5f, 1.5f, 0);

        [Header("Camera Control")]
        [SerializeField] private float _lookSensitivity = 1.0f;
        [SerializeField] private float _smoothSpeed = 10.0f;
        [SerializeField] private float _minVerticalAngle = -35.0f;
        [SerializeField] private float _maxVerticalAngle = 60.0f;

        [Header("Positioning")]
        [SerializeField] private float _distance = 3.0f;

        [Header("Collision")]
        [SerializeField] private LayerMask _collisionLayers;
        [SerializeField] private float _collisionPadding = 0.25f;

        // --- Private Class Members ---
        private PlayerControls _playerControls;
        private Vector2 _lookInput;
        private float _currentYaw;
        private float _currentPitch;
        private bool _isLocked = false;

        void Awake()
        {
            _playerControls = new PlayerControls();
            Cursor.lockState = CursorLockMode.Locked;
        }

        void OnEnable()
        {
            _playerControls.Gameplay.Enable();
            _playerControls.Gameplay.ShoulderToggle.performed += _ => ToggleShoulder();
        }

        void OnDisable()
        {
            _playerControls.Gameplay.ShoulderToggle.performed -= _ => ToggleShoulder();
            _playerControls.Gameplay.Disable();
        }

        // This is the public "on/off" switch for our camera's rotation.
        public void SetLock(bool shouldLock)
        {
            _isLocked = shouldLock;
        }

        void Update()
        {
            // Read the mouse movement value from the Input Action.
            _lookInput = _playerControls.Gameplay.Look.ReadValue<Vector2>();
        }

        void LateUpdate()
        {
            // Only update the camera's rotation angles if it is NOT locked.
            if (!_isLocked)
            {
                _currentYaw += _lookInput.x * _lookSensitivity * Time.deltaTime * 50f;
                _currentPitch -= _lookInput.y * _lookSensitivity * Time.deltaTime * 50f;
                _currentPitch = Mathf.Clamp(_currentPitch, _minVerticalAngle, _maxVerticalAngle);
            }

            Quaternion rotation = Quaternion.Euler(_currentPitch, _currentYaw, 0);
            
            Vector3 lookAtBase = _target.position;
            Vector3 lookAtPoint = lookAtBase + (rotation * _lookAtPointOffset);
            
            Vector3 desiredPosition = lookAtPoint - (rotation * Vector3.forward * _distance);

            // Handle collisions to prevent clipping through walls.
            if (Physics.Linecast(lookAtBase, desiredPosition, out RaycastHit hit, _collisionLayers))
            {
                transform.position = hit.point + hit.normal * _collisionPadding;
            }
            else
            {
                transform.position = desiredPosition;
            }

            // Smoothly move and rotate the camera to its final position.
            transform.LookAt(lookAtPoint);
        }

        private void ToggleShoulder()
        {
            // Flip the sign of the horizontal offset to switch shoulders.
            _lookAtPointOffset.x *= -1;
        }
    }
}