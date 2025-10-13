using UnityEngine;

public class PunchDetector : MonoBehaviour
{
    [Header("Detection Settings")]
    [Tooltip("Minimum velocity to register as a punch (m/s)")]
    public float minimumPunchVelocity = 1.0f;

    [Header("Debug")]
    public bool showDebugInfo = true;

    private void OnCollisionEnter(Collision collision)
    {
        // Check if it's a hand rigidbody that hit us
        Rigidbody hitRigidbody = collision.rigidbody;
        
        if (hitRigidbody == null) return;

        // Check if the colliding object is a hand
        string objectName = hitRigidbody.gameObject.name.ToLower();
        bool isHand = objectName.Contains("hand");

        if (!isHand) return;

        // Get the velocity at impact
        Vector3 impactVelocity = hitRigidbody.velocity;
        float speed = impactVelocity.magnitude;

        // Only register if velocity is high enough
        if (speed < minimumPunchVelocity) return;

        // Determine which hand
        string handName = objectName.Contains("r") ? "RIGHT" : "LEFT";

        // Get contact point info
        ContactPoint contact = collision.GetContact(0);
        Vector3 impactPoint = contact.point;
        Vector3 impactNormal = contact.normal;

        // Calculate impact force (approximation)
        float impactForce = collision.impulse.magnitude / Time.fixedDeltaTime;

        // Print detailed info
        if (showDebugInfo)
        {
            Debug.Log($"<color=yellow>=== PUNCH DETECTED ===</color>");
            Debug.Log($"<color=cyan>Hand:</color> {handName}");
            Debug.Log($"<color=cyan>Speed:</color> {speed:F2} m/s");
            // Debug.Log($"<color=cyan>Velocity:</color> {impactVelocity}");
            // Debug.Log($"<color=cyan>Impact Force:</color> {impactForce:F2} N");
            // Debug.Log($"<color=cyan>Impact Point:</color> {impactPoint}");
            // Debug.Log($"<color=cyan>Impact Normal:</color> {impactNormal}");
            Debug.Log($"<color=yellow>=====================</color>");
        }

        // Call this for additional custom behavior
        OnPunchDetected(handName, speed, impactVelocity, impactForce, impactPoint);
    }

    // Override this method in derived classes for custom punch behavior
    protected virtual void OnPunchDetected(string handName, float speed, Vector3 velocity, float force, Vector3 point)
    {
        // Custom behavior can be added here or in derived classes
    }

    // Visualize detection in scene view
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(transform.position, transform.localScale);
    }
}