using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Walk : MonoBehaviour
{
    [Header("Required Components")]
    public ActiveRagdoll activeRagdoll;
    public Transform cam;

    [Header("Movement Settings")]
    public float speed = 7;

    private Transform _centerOfMass;
    private Rigidbody _hipsRb;

    void Start()
    {
        if (activeRagdoll == null)
        {
            Debug.LogError("ActiveRagdoll reference is not set in the Walk script!");
            this.enabled = false;
            return;
        }

        _centerOfMass = activeRagdoll.connectedBody.transform;
        _hipsRb = activeRagdoll.rootBone.GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        // Get input axes. "Horizontal" is A/D, "Vertical" is W/S.
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");

        // Calculate movement direction relative to the camera's orientation
        Vector3 forward = cam.forward;
        Vector3 right = cam.right;

        // Make directions parallel to the ground plane
        forward.y = 0f;
        right.y = 0f;
        forward.Normalize();
        right.Normalize();

        // Combine the inputs to get the final desired movement direction
        Vector3 desiredMoveDirection = (forward * verticalInput + right * horizontalInput).normalized;

        // Apply force only if there is input
        if (desiredMoveDirection.magnitude > 0.1f)
        {
            // THE KEY CHANGE: We have removed the line that forces rotation.
            // The character will no longer turn to face its movement direction.

            // Apply force to the hips to make the character step in the desired direction
            _hipsRb.AddForce(desiredMoveDirection * speed, ForceMode.Acceleration);
        }
    }
}