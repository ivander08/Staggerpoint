using UnityEngine;

public class Weapon : MonoBehaviour
{
    [Header("Weapon Type")]
    public bool isTwoHanded = false;

    [Header("Grip Points")]
    public Transform rightGripPoint;
    public Transform leftGripPoint;

    private ConfigurableJoint rightHandJoint;
    private ConfigurableJoint leftHandJoint;

    public bool IsEquipped => rightHandJoint != null || leftHandJoint != null;
    public bool IsHeldByRightHand => rightHandJoint != null;
    public bool IsHeldByLeftHand => leftHandJoint != null;

    private Rigidbody weaponRB;



    private void Awake()
    {
        // rb = GetComponent<Rigidbody>();
        weaponRB = GetComponent<Rigidbody>();

        // Error checking to help you find setup problems in the Editor
        if (rightGripPoint == null)
        {
            Debug.LogError($"Weapon '{name}' is missing its 'rightGripPoint'. This is required.", this);
        }
        if (isTwoHanded && leftGripPoint == null)
        {
            Debug.LogError($"Weapon '{name}' is marked as 'isTwoHanded' but is missing its 'leftGripPoint'.", this);
        }
    }

    void FixedUpdate()
    {
        if (IsEquipped && weaponRB != null)
        {
            weaponRB.velocity = Vector3.ClampMagnitude(weaponRB.velocity, 10f);
            weaponRB.angularVelocity = Vector3.ClampMagnitude(weaponRB.angularVelocity, 10f);
        }
    }

    // REPLACE your existing Pickup() method in Weapon.cs with this version:

    public void Pickup(Transform handGripPoint, Rigidbody handRigidbody, bool isRightHand)
    {
        Transform weaponGrip;

        // If the weapon is NOT two-handed, it's ambidextrous.
        // It should ALWAYS use its primary grip point (rightGripPoint),
        // regardless of which hand is picking it up.
        if (!isTwoHanded)
        {
            weaponGrip = rightGripPoint;
        }
        else
        {
            // Only if it IS a two-handed weapon do we choose between left and right grips.
            weaponGrip = isRightHand ? rightGripPoint : leftGripPoint;
        }

        if (weaponGrip == null)
        {
            Debug.LogError($"Weapon '{name}' is missing a valid grip point for the selected hand.", this);
            return;
        }

        // The "snap" for the first hand is unchanged and correct.
        if (!IsEquipped)
        {
            AlignGripPoints(handGripPoint, weaponGrip);
        }

        // Create the joint
        ConfigurableJoint joint;
        if (isRightHand)
        {
            if (rightHandJoint != null) Destroy(rightHandJoint);
            rightHandJoint = gameObject.AddComponent<ConfigurableJoint>();
            joint = rightHandJoint;
        }
        else
        {
            if (leftHandJoint != null) Destroy(leftHandJoint);
            leftHandJoint = gameObject.AddComponent<ConfigurableJoint>();
            joint = leftHandJoint;
        }

        // Basic joint setup
        joint.connectedBody = handRigidbody;
        joint.autoConfigureConnectedAnchor = true;
        joint.anchor = transform.InverseTransformPoint(weaponGrip.position);

        // --- CHANGED: Use FREE motion with DRIVES instead of LOCKED ---
        // This allows the joint to "give" under extreme force, preventing freezes
        joint.xMotion = ConfigurableJointMotion.Free;
        joint.yMotion = ConfigurableJointMotion.Free;
        joint.zMotion = ConfigurableJointMotion.Free;
        joint.angularXMotion = ConfigurableJointMotion.Free;
        joint.angularYMotion = ConfigurableJointMotion.Free;
        joint.angularZMotion = ConfigurableJointMotion.Free;

        // --- NEW: Strong drives that make it FEEL locked, but can give under impact ---
        // These values are managed by WeaponImpactRecoil component
        JointDrive strongDrive = new JointDrive
        {
            positionSpring = 800000f,    // Very strong - feels rigid
            positionDamper = 8000f,     // High damping - prevents wobble
            maximumForce = float.MaxValue      // Safety cap
        };

        joint.xDrive = strongDrive;
        joint.yDrive = strongDrive;
        joint.zDrive = strongDrive;
        joint.angularXDrive = strongDrive;
        joint.angularYZDrive = strongDrive;
        // --- END NEW CODE ---

        // Projection settings (keep your existing values)
        joint.projectionMode = JointProjectionMode.PositionAndRotation;
        joint.projectionAngle = 3.0f;
        joint.projectionDistance = 0.01f;
    }

    /// <summary>
    /// Detaches the weapon from a specified hand.
    /// </summary>
    public void Drop(bool isRightHand)
    {
        if (isRightHand)
        {
            if (rightHandJoint != null)
            {
                Destroy(rightHandJoint);
                rightHandJoint = null;
            }
        }
        else
        {
            if (leftHandJoint != null)
            {
                Destroy(leftHandJoint);
                leftHandJoint = null;
            }
        }
    }

    /// <summary>
    /// Instantly teleports and rotates the weapon to perfectly align its grip with the hand's grip.
    /// </summary>
    private void AlignGripPoints(Transform handGripPoint, Transform weaponGripPoint)
    {
        // Calculate the rotation needed to align the grips.
        Quaternion rotationDifference = handGripPoint.rotation * Quaternion.Inverse(weaponGripPoint.rotation);
        // Apply the rotation to the weapon.
        transform.rotation = rotationDifference * transform.rotation;

        // Calculate the position offset needed to align the grips.
        Vector3 positionDifference = handGripPoint.position - weaponGripPoint.position;
        // Apply the offset to the weapon.
        transform.position += positionDifference;
    }
}