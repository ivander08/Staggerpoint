using UnityEngine;

public class Weapon : MonoBehaviour
{
    [Header("Weapon Type")]
    public bool isTwoHanded = false;

    [Header("Grip Points")]
    public Transform rightGripPoint;
    public Transform leftGripPoint;

    // --- We will now store ConfigurableJoints instead of FixedJoints ---
    private ConfigurableJoint rightHandJoint;
    private ConfigurableJoint leftHandJoint;

    // --- Public properties remain the same ---
    public bool IsEquipped => rightHandJoint != null || leftHandJoint != null;
    public bool IsHeldByRightHand => rightHandJoint != null;
    public bool IsHeldByLeftHand => leftHandJoint != null;


    private void Awake()
    {
        // rb = GetComponent<Rigidbody>();

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

    // This is the new, corrected Pickup method for Weapon.cs
    public void Pickup(Transform handGripPoint, Rigidbody handRigidbody, bool isRightHand)
    {
        Transform weaponGrip;

        // --- THIS IS THE NEW LOGIC ---
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
        // --- END OF NEW LOGIC ---

        if (weaponGrip == null)
        {
            // This error will now correctly catch if a 1H weapon is missing its main grip.
            Debug.LogError($"Weapon '{name}' is missing a valid grip point for the selected hand.", this);
            return;
        }

        // The "snap" for the first hand is unchanged and correct.
        if (!IsEquipped)
        {
            AlignGripPoints(handGripPoint, weaponGrip);
        }

        // ... THE REST OF THE METHOD (creating the joint) IS UNCHANGED ...
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

        joint.connectedBody = handRigidbody;
        joint.autoConfigureConnectedAnchor = true;
        joint.anchor = transform.InverseTransformPoint(weaponGrip.position);
        joint.xMotion = ConfigurableJointMotion.Locked;
        joint.yMotion = ConfigurableJointMotion.Locked;
        joint.zMotion = ConfigurableJointMotion.Locked;
        joint.angularXMotion = ConfigurableJointMotion.Locked;
        joint.angularYMotion = ConfigurableJointMotion.Locked;
        joint.angularZMotion = ConfigurableJointMotion.Locked;
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