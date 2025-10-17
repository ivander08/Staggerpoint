// Attach this to your player (same object as PlayerController probably)
using UnityEngine;

public class WeaponPickupManager : MonoBehaviour
{
    [SerializeField] private Transform handGripPointRight;
    [SerializeField] private Transform handGripPointLeft;
    [SerializeField] private Transform handRight;
    [SerializeField] private Transform handLeft;

    [SerializeField] private float pickupRange = 2f;

    public Weapon equippedWeaponRight;
    public Weapon equippedWeaponLeft;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.E))
            HandlePickup(true);

        if (Input.GetKeyDown(KeyCode.Q))
            HandlePickup(false);
    }

    private void HandlePickup(bool isRight)
    {
        Transform handGrip = isRight ? handGripPointRight : handGripPointLeft;
        Transform hand = isRight ? handRight : handLeft;
        Weapon currentEquipped = isRight ? equippedWeaponRight : equippedWeaponLeft;

        // If already holding something, drop it
        if (currentEquipped != null)
        {
            currentEquipped.DropWeapon();
            if (isRight)
                equippedWeaponRight = null;
            else
                equippedWeaponLeft = null;
            return;
        }

        // Look for nearby weapons
        Collider[] nearby = Physics.OverlapSphere(handGrip.position, pickupRange);

        foreach (Collider col in nearby)
        {
            Weapon weapon = col.GetComponent<Weapon>();
            if (weapon != null && !weapon.IsEquipped)
            {
                weapon.PickupWeapon(handGrip, hand);

                if (isRight)
                    equippedWeaponRight = weapon;
                else
                    equippedWeaponLeft = weapon;

                return;
            }
        }
    }
}