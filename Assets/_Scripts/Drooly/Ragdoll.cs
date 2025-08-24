using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ragdoll : MonoBehaviour
{
    [Header("Core References")]
    public ActiveRagdoll walkScript;
    public InverseKinematics leftIk, rightIk;
    public Rigidbody hipsRb;

    [Header("Balance Settings")]
    public int fallAngle = 55;

    [HideInInspector]
    public bool ragdolled = false;
    private bool _conscious = true;

    void Start()
    {
        //ragdoll();
    }

    void Update()
    {
        if (!ragdolled && _conscious) // if its standing
        {
            float a = Vector3.Angle(hipsRb.transform.up, Vector3.up);
            
            if (a > fallAngle) ragdoll(); //if its unbalanced fall over
            else if (walkScript.falling) ragdoll(); //if there is no floor below to step on fall over
        }
        if (ragdolled && _conscious) // if its knocked down but wakes up
        {
            if (hipsRb.velocity.magnitude < 0.1) ragdoll();// if it is knocked down and not moving
            else if (hipsRb.velocity.magnitude < 1) StartCoroutine(setConscious(3)); // if it is knocked down but moving too much to get up
        }
    }

    public void ragdoll()
    {
        if (!ragdolled) // fall down
        {
            Destroy(walkScript.joint);
            hipsRb.useGravity = true;

            walkScript.enabled = false;
            leftIk.enabled = false;
            rightIk.enabled = false;

            ragdolled = true;
            StartCoroutine(setConscious(5));
        }
        else // get up
        {
            walkScript.enabled = true;
            leftIk.enabled = true;
            rightIk.enabled = true;
            walkScript.setupJoint();
            hipsRb.useGravity = false;

            ragdolled = false;
            StartCoroutine(setConscious(3));
        }
    }

    private IEnumerator setConscious(float time)// makes the ragdoll fall asleep for an amount of time
    {
        _conscious = false;
        yield return new WaitForSeconds(time);
        _conscious = true;
    }
}