using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class aiRagdoll : MonoBehaviour
{
    public Transform com;
    public  NavMeshAgent navigator;
    public Rigidbody hips;
    public Transform target;
    public float dist;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (!com) com = GameObject.Find("AI/joint").transform;

        navigator.SetDestination(target.position);
        navigator.transform.position = hips.transform.position;
        com.rotation = navigator.transform.rotation;

        if(Vector3.Distance(hips.transform.position, target.position) > dist)
        {
            hips.AddForce(com.transform.forward * 4, ForceMode.Acceleration);
        }
    }
}
