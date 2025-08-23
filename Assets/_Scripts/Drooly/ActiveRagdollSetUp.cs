using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActiveRagdollSetUp : MonoBehaviour
{
    public Transform leftFoot, rightFoot, rootBone;


    public void setUp()
    {
        if(leftFoot == null || rightFoot == null || rootBone == null)
        {
            Debug.LogError("Set The Paramaters");
            return;
        }

        // Create Inverse Kinematics Targets
        Transform leftIkTarget = new GameObject("Left IK Target").transform;
        Transform rightIkTarget = new GameObject("Right IK Target").transform;

        leftIkTarget.parent = leftFoot;
        leftIkTarget.localPosition = Vector3.zero;

        rightIkTarget.parent = rightFoot;
        rightIkTarget.localPosition = Vector3.zero;

        leftIkTarget.parent = transform;
        rightIkTarget.parent = transform;


        // Create IK Poles
        Transform leftPole = new GameObject("Left IK Pole").transform;
        Transform rightPole = new GameObject("Right IK Pole").transform;

        leftPole.position = new Vector3(leftIkTarget.position.x, rootBone.position.y, leftIkTarget.position.z + 1);
        rightPole.position = new Vector3(rightIkTarget.position.x, rootBone.position.y, rightIkTarget.position.z + 1);

        leftPole.parent = rootBone;
        rightPole.parent = rootBone;

        // SetUp IK
        InverseKinematics leftIk = leftFoot.gameObject.AddComponent<InverseKinematics>();
        InverseKinematics rightIk = rightFoot.gameObject.AddComponent<InverseKinematics>();

        leftIk.Target = leftIkTarget;
        rightIk.Target = rightIkTarget;

        leftIk.Pole = leftPole;
        rightIk.Pole = rightPole;


        // SetUp the Script
        ActiveRagdoll script = gameObject.AddComponent<ActiveRagdoll>();

        script.rootBone = rootBone;

        script.moveDir = new GameObject("Move Direction").transform;
        script.moveDir.parent = transform;
        script.moveDir.position = rootBone.position;

        script.leftFoot = leftIkTarget;
        script.rightFoot = rightIkTarget;

        DestroyImmediate(this);
    }
}
