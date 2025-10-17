using UnityEngine;

// Attach this to each weapon
public class Weapon : MonoBehaviour
{
    [SerializeField] private Transform weaponGripPoint;
    private bool isEquipped = false;
    private FixedJoint fixedJoint;
    private Rigidbody rb;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        
        // If no explicit grip point, create one at the current position
        if (weaponGripPoint == null)
        {
            GameObject gripObj = new GameObject("WeaponGripPoint");
            gripObj.transform.SetParent(transform);
            gripObj.transform.localPosition = Vector3.zero;
            weaponGripPoint = gripObj.transform;
        }
    }

    public void PickupWeapon(Transform handGripPoint, Transform hand)
    {
        if (isEquipped) return;

        isEquipped = true;

        // Position the weapon so grip points align
        AlignGripPoints(handGripPoint);

        // Make it a child of the hand (optional, but helps with tracking)
        // transform.SetParent(hand);

        // Add FixedJoint
        if (fixedJoint != null)
            Destroy(fixedJoint);

        fixedJoint = gameObject.AddComponent<FixedJoint>();
        fixedJoint.connectedBody = hand.GetComponent<Rigidbody>();
    }

    public void DropWeapon()
    {
        if (!isEquipped) return;

        isEquipped = false;

        // Remove FixedJoint
        if (fixedJoint != null)
            Destroy(fixedJoint);

        // Optionally unparent if you parented it
        // transform.SetParent(null);
    }

    private void AlignGripPoints(Transform handGripPoint)
    {
        // Calculate rotation needed to align grip point rotations
        Quaternion currentGripWorldRot = transform.rotation * Quaternion.Euler(weaponGripPoint.localEulerAngles);
        Quaternion rotationDifference = handGripPoint.rotation * Quaternion.Inverse(currentGripWorldRot);
        
        // Apply rotation to weapon
        transform.rotation = rotationDifference * transform.rotation;
        
        // Move weapon so grip point position aligns with hand grip point
        Vector3 currentGripWorldPos = transform.TransformPoint(weaponGripPoint.localPosition);
        Vector3 offset = handGripPoint.position - currentGripWorldPos;
        transform.position += offset;
    }

    public bool IsEquipped => isEquipped;
}

