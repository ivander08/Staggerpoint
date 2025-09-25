using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActiveRagdollSetUp : MonoBehaviour
{
    // Renamed for clarity
    public Transform leftFootBone, rightFootBone, hipsBone;

    public void setUp()
    {
        if(leftFootBone == null || rightFootBone == null || hipsBone == null)
        {
            Debug.LogError("Set The Paramaters");
            return;
        }

        Transform leftIkTarget = new GameObject("Left IK Target").transform;
        Transform rightIkTarget = new GameObject("Right IK Target").transform;

        leftIkTarget.parent = leftFootBone;
        leftIkTarget.localPosition = Vector3.zero;

        rightIkTarget.parent = rightFootBone;
        rightIkTarget.localPosition = Vector3.zero;

        leftIkTarget.parent = transform;
        rightIkTarget.parent = transform;
        
        Transform leftPole = new GameObject("Left IK Pole").transform;
        Transform rightPole = new GameObject("Right IK Pole").transform;

        leftPole.position = new Vector3(leftIkTarget.position.x, hipsBone.position.y, leftIkTarget.position.z + 1);
        rightPole.position = new Vector3(rightIkTarget.position.x, hipsBone.position.y, rightIkTarget.position.z + 1);

        leftPole.parent = hipsBone;
        rightPole.parent = hipsBone;
        
        InverseKinematics leftIk = leftFootBone.gameObject.AddComponent<InverseKinematics>();
        InverseKinematics rightIk = rightFootBone.gameObject.AddComponent<InverseKinematics>();

        leftIk.Target = leftIkTarget;
        rightIk.Target = rightIkTarget;

        leftIk.Pole = leftPole;
        rightIk.Pole = rightPole;
        
        // Assign to the new variable names on the ActiveRagdoll component
        ActiveRagdoll script = gameObject.AddComponent<ActiveRagdoll>();
        script.hipsTransform = hipsBone;
        script.stepGuide = new GameObject("Step Guide").transform;
        script.stepGuide.parent = transform;
        script.stepGuide.position = hipsBone.position;
        script.leftFootIKTarget = leftIkTarget;
        script.rightFootIKTarget = rightIkTarget;

        DestroyImmediate(this);
    }
}