using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class npcBasic : MonoBehaviour
{
    [HideInInspector]
    public Transform com;

    public ActiveRagdoll activeRagdoll;

    private  Rigidbody rootBone;
    [Tooltip("The position that the ragdoll will walk toward")]
    public Transform target;
    [Tooltip("The speed it will walk")]
    public float speed = 5;
    [Tooltip("inverts the direction the ragdoll walks")]
    public bool invert;
    [Tooltip("How far away it should be from the target before it stops walking. if left at 0 it will bump into target.")]
    public float standDist;
    // Start is called before the first frame update
    void Start()
    {
       
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (!com) {com = activeRagdoll.connectedBody.transform; rootBone = activeRagdoll.rootBone.GetComponent<Rigidbody>(); }
        Vector3 targetPos = new Vector3(target.position.x, com.position.y, target.position.z);
        if(invert) com.rotation = Quaternion.LookRotation(com.position - targetPos);
        else com.rotation = Quaternion.LookRotation(targetPos - com.position);
        
        if(Vector3.Distance(target.position, com.position) > standDist) //if the ragdoll is farther away than it wants to be from the target
            rootBone.AddForce((invert ? -com.forward : com.forward)*speed); //Add a physical force to walk forward
    }
}
