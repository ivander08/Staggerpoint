using UnityEngine;

public class PunchDetector : MonoBehaviour
{
    [Header("Detection Settings")]
    [Tooltip("Which physics layers should trigger a hit detection (e.g., Weapons, Hands)")]
    public LayerMask hittableLayers; // This is the new field
    [Tooltip("Minimum velocity of the hitting object to register as a valid hit (m/s)")]
    public float minimumHitVelocity = 1.0f;

    [Header("Debug")]
    public bool showDebugInfo = true;

    private void OnCollisionEnter(Collision collision)
    {
        // Check if the object that hit us has a Rigidbody
        Rigidbody hitRigidbody = collision.rigidbody;
        if (hitRigidbody == null) return;

        // --- MODIFICATION START ---
        // Check if the colliding object's layer is in our hittable layers mask.
        // This is more flexible than checking the object's name.
        if ((hittableLayers.value & (1 << collision.gameObject.layer)) == 0)
        {
            // If the layer doesn't match the mask, ignore this collision.
            return;
        }
        // --- MODIFICATION END ---

        // Get the velocity at impact
        Vector3 impactVelocity = hitRigidbody.velocity;
        float speed = impactVelocity.magnitude;

        // Only register if velocity is high enough
        if (speed < minimumHitVelocity) return;

        // Get the name of the object that hit us for the debug log
        string hittingObjectName = hitRigidbody.gameObject.name;

        // Get contact point info
        ContactPoint contact = collision.GetContact(0);
        Vector3 impactPoint = contact.point;
        Vector3 impactNormal = contact.normal;

        // Calculate impact force (approximation)
        float impactForce = collision.impulse.magnitude / Time.fixedDeltaTime;

        // Print detailed info
        if (showDebugInfo)
        {
            Debug.Log($"<color=yellow>=== HIT DETECTED ON {gameObject.name} ===</color>");
            Debug.Log($"<color=cyan>Hitting Object:</color> {hittingObjectName}");
            // Debug.Log($"<color=cyan>Layer:</color> {LayerMask.LayerToName(collision.gameObject.layer)}");
            Debug.Log($"<color=cyan>Speed:</color> {speed:F2} m/s");
            // Debug.Log($"<color=cyan>Impact Force (Approx):</color> {impactForce:F2} N");
        }

        // Call this for additional custom behavior, now passing the entire GameObject
        OnHitDetected(hitRigidbody.gameObject, speed, impactVelocity, impactForce, impactPoint);
    }

    // Renamed and updated to be more generic. Override this in other scripts for custom behavior.
    protected virtual void OnHitDetected(GameObject hittingObject, float speed, Vector3 velocity, float force, Vector3 point)
    {
        // For example, you could add logic here to apply damage based on the object's speed or mass.
        // if(hittingObject.CompareTag("Sword")) { TakeDamage(force * 1.5f); }
    }

    // Visualize detection in scene view
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(transform.position, transform.localScale);
    }
}