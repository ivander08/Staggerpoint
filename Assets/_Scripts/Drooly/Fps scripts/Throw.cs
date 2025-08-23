using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class Throw : MonoBehaviour
{
    public GameObject cubePrefab;
    public Material[] colors;


    public Camera cam;
    public Image aimer;

    public Transform hand;
    public ConfigurableJoint grip;
    private LineRenderer lineRenderer;
    private GameObject gripPoint;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

        if (Input.GetMouseButtonDown(0))
        {
            Rigidbody rb = Instantiate(cubePrefab, transform.position + (transform.forward * .5f), transform.rotation).GetComponent<Rigidbody>();
            rb.AddForce(transform.forward * 10, ForceMode.Impulse);
            rb.GetComponent<MeshRenderer>().material = colors[Random.Range(0, colors.Length)];
        }
        if (Input.GetMouseButtonDown(1))
        {
            RaycastHit hit = new RaycastHit();
            Ray ray = cam.ScreenPointToRay(aimer.transform.position);
            if (Physics.Raycast(ray, out hit))
            {
                Rigidbody rb = hit.transform.GetComponent<Rigidbody>();
                if (rb)
                {
                    hand.position = hit.point;
                    grip.connectedBody = rb;

                    gripPoint = new GameObject("grip point");
                    gripPoint.transform.position = hit.point;
                    gripPoint.transform.parent = rb.transform;
                    //For creating line renderer object
                    lineRenderer = new GameObject("Line").AddComponent<LineRenderer>();
                    lineRenderer.material = new Material(Shader.Find("Legacy Shaders/Particles/Alpha Blended Premultiply"));
                    lineRenderer.startColor = Color.black;
                    lineRenderer.endColor = Color.black;
                    lineRenderer.startWidth = 0.05f;
                    lineRenderer.endWidth = 0.05f;
                    lineRenderer.positionCount = 2;
                    lineRenderer.useWorldSpace = true;
                }
            }
        }
        if (Input.GetMouseButtonUp(1))
        {
            grip.connectedBody = null;
            Destroy(lineRenderer);
            Destroy(gripPoint);
        }

        if (grip.connectedBody)
        {
            //For drawing line in the world space, provide the x,y,z values
            lineRenderer.SetPosition(0, hand.position); //x,y and z position of the starting point of the line
            lineRenderer.SetPosition(1, gripPoint.transform.position); //x,y and z position of the end point of the line
        }

        if(Input.GetKeyDown(KeyCode.R)) { SceneManager.LoadScene(SceneManager.GetActiveScene().name); }
    }
}
