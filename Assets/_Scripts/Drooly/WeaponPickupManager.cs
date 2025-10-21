using System.Collections;
using UnityEngine;

public class WeaponPickupManager : MonoBehaviour
{
    [Header("Hand References")]
    [SerializeField] private Transform handGripPointRight;
    [SerializeField] private Transform handGripPointLeft;
    [SerializeField] private Rigidbody handRight;
    [SerializeField] private Rigidbody handLeft;
    // Add this reference at the top of WeaponPickupManager.cs if you don't have it
    [SerializeField] private ArmController armController;

    [Header("Settings")]
    [SerializeField] private float pickupRange = 2f;

    [Header("Weapon State")]
    public Weapon equippedWeaponRight;
    public Weapon equippedWeaponLeft;

    private int grabbingHandLayer;
    private int defaultLayer;

    private void Start()
    {
        grabbingHandLayer = LayerMask.NameToLayer("GrabbingHand");
        defaultLayer = LayerMask.NameToLayer("Default");
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.E)) { HandlePickup(true); }
        if (Input.GetKeyDown(KeyCode.Q)) { HandlePickup(false); }
    }

    private void HandlePickup(bool isRight)
    {
        // --- SCENARIO 1: HOLDING A 2H WEAPON WITH BOTH HANDS ---
        if (equippedWeaponRight != null && equippedWeaponRight == equippedWeaponLeft)
        {
            armController.isTwoHandedMode = false; // Turn off two-handed mode
            equippedWeaponRight.Drop(true);
            equippedWeaponRight.Drop(false);
            equippedWeaponRight = null;
            equippedWeaponLeft = null;
            return;
        }

        Weapon currentEquipped = isRight ? equippedWeaponRight : equippedWeaponLeft;

        // --- SCENARIO 2: THE PRESSED HAND IS ALREADY HOLDING SOMETHING ---
        if (currentEquipped != null)
        {
            currentEquipped.Drop(isRight);
            if (isRight) equippedWeaponRight = null;
            else equippedWeaponLeft = null;
            return;
        }

        // --- SCENARIO 3 (THE SPECIAL CASE): HOLDING A 2H WEAPON IN RIGHT HAND, 'Q' IS PRESSED ---
        if (!isRight && equippedWeaponRight != null && equippedWeaponRight.isTwoHanded)
        {
            StartCoroutine(AttachSecondHandRoutine());
            return;
        }

        // --- SCENARIO 4: HAND IS EMPTY, PICK UP A NEW WEAPON ---
        Transform handGrip = isRight ? handGripPointRight : handGripPointLeft;
        Rigidbody hand = isRight ? handRight : handLeft;

        Collider[] nearby = Physics.OverlapSphere(handGrip.position, pickupRange);
        foreach (Collider col in nearby)
        {
            Weapon weapon = col.GetComponent<Weapon>();
            if (weapon != null && !weapon.IsEquipped)
            {
                weapon.Pickup(handGrip, hand, isRight);
                if (isRight) equippedWeaponRight = weapon;
                else equippedWeaponLeft = weapon;
                return;
            }
        }
    }

    private IEnumerator AttachSecondHandRoutine()
    {
        // Store references to avoid confusion
        Transform handToMove = handLeft.transform;
        Transform handGrip = handGripPointLeft;
        Transform weaponGrip = equippedWeaponRight.leftGripPoint;

        // --- STEP 1: GHOST THE HAND ---
        int originalLayer = handToMove.gameObject.layer;
        handToMove.gameObject.layer = grabbingHandLayer;
        handLeft.isKinematic = true;

        // --- STEP 2: TELEPORT THE GHOST HAND (WITH CORRECT MATH) ---
        // Calculate the rotation needed to align the grips.
        Quaternion rotationDifference = weaponGrip.rotation * Quaternion.Inverse(handGrip.rotation);
        // Apply the rotation to the hand's transform.
        handToMove.rotation = rotationDifference * handToMove.rotation;

        // Now that it's rotated correctly, calculate the position offset.
        Vector3 positionDifference = weaponGrip.position - handGrip.position;
        // Apply the offset to the hand's transform.
        handToMove.position = handToMove.position + positionDifference;

        // --- STEP 3: WAIT ONE FRAME ---
        // This gives Unity a moment to process the teleport before we continue.
        yield return new WaitForEndOfFrame();

        // --- STEP 4: CREATE THE JOINT ---
        equippedWeaponRight.Pickup(handGripPointLeft, handLeft, false);

        // --- STEP 5: UN-GHOST THE HAND ---
        handLeft.isKinematic = false;
        handToMove.gameObject.layer = originalLayer;

        // --- STEP 6: UPDATE STATE ---
        equippedWeaponLeft = equippedWeaponRight;

        armController.isTwoHandedMode = true;
    }
}