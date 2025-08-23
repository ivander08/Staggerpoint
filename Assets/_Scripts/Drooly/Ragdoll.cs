using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// script to ragdoll when out of balance
/// </summary>
public class Ragdoll : MonoBehaviour
{
    public ActiveRagdoll walkScript;
    [Tooltip("The InversKinematics script of the feet bones")]
    public InverseKinematics leftIk, rightIk;
    [Tooltip("The rigidbody attached to the root bone")]
    public Rigidbody hipsRb;
    [Tooltip("The higher the angle is the harder it will be to knock over the ragdoll and the better its balance will be.")]
    public int fallAngle = 55;

    [HideInInspector]
    public bool ragdolled = false;
    private bool conscious = true;

    void Start()
    {
        //ragdoll();
    }

    // Update is called once per frame
    void Update()
    {
        if (!ragdolled && conscious) // if its standing
        {
            
            float a = Vector3.Angle(hipsRb.transform.up, Vector3.up);
            
            if (a > fallAngle) ragdoll(); //if its unbalanced fall over
            else if (walkScript.falling) ragdoll(); //if there is no floor below to step on fall over
        }
        if (ragdolled && conscious) // if its knocked down but wakes up
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

    public IEnumerator setConscious(float time)// makes the ragdoll fall asleep for an amount of time
    {
        conscious = false;
        yield return new WaitForSeconds(time);
        conscious = true;
    }
}
