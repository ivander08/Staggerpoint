using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ragdoll : MonoBehaviour
{
    [Header("Core References")]
    public ActiveRagdoll activeRagdoll; // Renamed from walkScript for clarity
    public InverseKinematics leftIk, rightIk;
    public Rigidbody hipsRb;

    [Header("Balance Settings")]
    public int fallAngle = 55;

    [HideInInspector]
    public bool ragdolled = false;
    private bool _conscious = true;

    void Update()
    {
        if (!ragdolled && _conscious)
        {
            float a = Vector3.Angle(hipsRb.transform.up, Vector3.up);
            
            // Use the new isAirborne variable
            if (a > fallAngle) ToggleRagdoll();
            else if (activeRagdoll.isAirborne) ToggleRagdoll();
        }
        if (ragdolled && _conscious)
        {
            if (hipsRb.velocity.magnitude < 0.1) ToggleRagdoll();
            else if (hipsRb.velocity.magnitude < 1) StartCoroutine(SetConscious(3));
        }
    }

    // Renamed from ragdoll() to be more descriptive
    public void ToggleRagdoll()
    {
        if (!ragdolled) // fall down
        {
            // Destroy the specific joint component by its variable name
            Destroy(activeRagdoll.balanceJoint);
            hipsRb.useGravity = true;

            activeRagdoll.enabled = false;
            leftIk.enabled = false;
            rightIk.enabled = false;

            ragdolled = true;
            StartCoroutine(SetConscious(5));
        }
        else // get up
        {
            activeRagdoll.enabled = true;
            leftIk.enabled = true;
            rightIk.enabled = true;
            // Use the new, clearer function name
            activeRagdoll.SetupBalanceJoint();
            hipsRb.useGravity = false;

            ragdolled = false;
            StartCoroutine(SetConscious(3));
        }
    }

    private IEnumerator SetConscious(float time)
    {
        _conscious = false;
        yield return new WaitForSeconds(time);
        _conscious = true;
    }
}