using UnityEngine;
using UnityEngine.InputSystem;

public class RagdollTester : MonoBehaviour
{
    public Rigidbody hipsRigidbody;
    public float pushForce = 500f;

    void Update()
    {
        // Press T to push forward
        if (Keyboard.current.tKey.wasPressedThisFrame)
            hipsRigidbody.AddForce(transform.forward * pushForce, ForceMode.Impulse);
        
        // Press G to push backward
        if (Keyboard.current.gKey.wasPressedThisFrame)
            hipsRigidbody.AddForce(-transform.forward * pushForce, ForceMode.Impulse);
        
        // Press F to push left
        if (Keyboard.current.fKey.wasPressedThisFrame)
            hipsRigidbody.AddForce(-transform.right * pushForce, ForceMode.Impulse);
        
        // Press H to push right
        if (Keyboard.current.hKey.wasPressedThisFrame)
            hipsRigidbody.AddForce(transform.right * pushForce, ForceMode.Impulse);
    }
}