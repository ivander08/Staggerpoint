using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActiveRagdoll : MonoBehaviour
{
    [Tooltip("The root bone")]
    public Transform rootBone;
    [Tooltip("This object shows the direction the body is moving in.")]
    public Transform moveDir;

    [Header("IK targets")]
    public Transform leftFoot;
    public Transform rightFoot;


    [Header("standing position values")]
    [Tooltip("The amount of space between the feet")]
    public float footGap;
    [Tooltip("The height of the body relative to the ground")]
    public float standHeight;

    [Header("step values")]
    [Tooltip("The minimum distance away from the body the foot must be to take a step")]
    public float stepDist;
    [Tooltip("The distance the foot travels during a step")]
    public float stepSize;
    [Tooltip("The amount of time it will take to take a step. The lower the value the faster the feet will move")]
    public float stepDuration;
    [Tooltip("How high the foot will go off the ground while taking a step")]
    public float stepHeight;

    [Header("physics")]
    [Tooltip("This doesnt matter")]
    public bool physics = false;
    [Tooltip("The force applied to keep the ragdoll standing upright. The higher it is the harder it is to knock the ragdoll over")]
    public int rotationForce = 10;
    public int rotationSmoothness = 1;
    [Tooltip("Add a layer called ragdolls and apply it to this object and all its children")]
    public LayerMask ragdollLayer;

    private Vector3 lastFrame; // the position of the body from the last frame
    private Transform leftSide, rightSide; // the sides of the body
    private Vector3 leftTarget, rightTarget; // The position of the foot without smoothing

    // physics stuff
    [HideInInspector]
    public ConfigurableJoint joint; // The configurable joint on the hips that helps it stay upright
    [HideInInspector]
    public Rigidbody connectedBody;

    private Rigidbody rb;

    [HideInInspector]
    public bool falling;

    void Awake()
    {
        rb = rootBone.GetComponent<Rigidbody>();

        // Set up the joint
        connectedBody = new GameObject("joint").AddComponent<Rigidbody>();
        connectedBody.transform.parent = transform;
        connectedBody.isKinematic = true;
        setupJoint();
    }

    // Start is called before the first frame update
    void Start()
    {
        moveDir.position = rootBone.position;
        lastFrame = rootBone.position;
        
        // Set up the foot positions
        leftSide = new GameObject("left side").transform;
        rightSide = new GameObject("right side").transform;

        leftSide.parent = moveDir;
        rightSide.parent = moveDir;

        leftSide.localPosition = new Vector3(footGap, 0, 0);
        rightSide.localPosition = new Vector3(-footGap, 0, 0);
    }

    // Update is called once per frame
    void Update()
    {
        setMoveDir();

        // Find the point on the ground where each foot should step
        Physics.Raycast(leftSide.position, Vector3.down, out RaycastHit hitL, 5, ~ragdollLayer);
        Physics.Raycast(rightSide.position, Vector3.down, out RaycastHit hitR, 5, ~ragdollLayer);

        // If the ragdoll is trying to step off a platform the falling will be set to true
        // When falling is true the ik and this script should be turned off so that it ragdolls
        if (hitL.collider == null || hitR.collider == null) { falling = true; } else falling = false;

        // Find the distance between the feet and where it should step
        float leftDist = Vector3.Distance(hitL.point, leftTarget);
        float rightDist = Vector3.Distance(hitR.point, rightTarget);

        // if both feet are ready to step, Take a step with the farthest foot
        if (leftDist >= stepDist && rightDist >= stepDist)
        {
            if (rightDist > leftDist)
            {
                rightTarget = hitL.point;
                StartCoroutine(step(rightFoot, rightTarget));
            }
            else
            {
                leftTarget = hitR.point;
                StartCoroutine(step(leftFoot, leftTarget));
            }
        }
        // Make the feet rotate with the body
        leftFoot.eulerAngles = new Vector3(leftFoot.eulerAngles.x, rootBone.eulerAngles.y, leftFoot.eulerAngles.z);
        rightFoot.eulerAngles = new Vector3(rightFoot.eulerAngles.x, rootBone.eulerAngles.y, rightFoot.eulerAngles.z);

        moveDir.eulerAngles = new Vector3(moveDir.eulerAngles.x, rootBone.eulerAngles.y, moveDir.eulerAngles.z);

        if (!falling)
        {
            // Keep the body elevated above the floor
            connectedBody.transform.position = new Vector3(rootBone.position.x, (hitL.point.y + hitL.point.y) / 2 + standHeight, rootBone.position.z);
        }
    }

    private IEnumerator step(Transform foot, Vector3 target)
    {
        Vector3 startPoint = foot.position; // The position of the foot before the step


        Vector3 centerPoint = (foot.position + target) / 2; // The point halfway through the step

        // The middle point of the step is higher so that the foot goes up when taking a step
        centerPoint.y = target.y + stepHeight;

        float timeElapsed = 0; // The time elapsed since the begining of the step

        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / stepDuration;

            // Quadratic bezier curve
            foot.position =
              Vector3.Lerp(
                Vector3.Lerp(startPoint, centerPoint, normalizedTime),
                Vector3.Lerp(centerPoint, target, normalizedTime),
                normalizedTime
              );

            yield return null;
        }
        while (timeElapsed < stepDuration);
    }

    public void setupJoint()
    {
        if (joint != null) Destroy(joint); // Destroy the joint if it already exists

        joint = rootBone.gameObject.AddComponent<ConfigurableJoint>(); // create a new joint

        JointDrive driveX = joint.angularXDrive; // Controls the rotation on the X axis
        JointDrive driveYZ = joint.angularYZDrive; // Controls the rotation on the y and z axis
        JointDrive driveY = joint.yDrive; // Controls the position on the y axis

        // Set the joints strength
        driveX.positionSpring = rotationForce;
        driveX.positionDamper = rotationSmoothness;

        driveYZ.positionSpring = rotationForce;
        driveYZ.positionDamper = rotationSmoothness;

        driveY.positionSpring = 300;
        driveY.positionDamper = 10;

        joint.angularXDrive = driveX;
        joint.angularYZDrive = driveYZ;
        joint.yDrive = driveY;

        // Set the connected body
        connectedBody.transform.position = rootBone.position;
        connectedBody.transform.rotation = rootBone.rotation;
        joint.connectedBody = connectedBody;
        connectedBody.transform.rotation = Quaternion.identity;
    }

    public void setMoveDir()
    {
        //print(rb.velocity.magnitude);
        if (physics) // If the character has a rigidbody attached to the root bone
        {
            if (rb.velocity.magnitude > 0.3) // If the body is moving
            {
                // Calculate the direction the body is moving in
                Vector3 dir = rb.velocity;
                dir.Normalize();
                dir.y = 0;
                moveDir.position = rootBone.position + dir * stepSize;
            }
            else
            {
                moveDir.position = rootBone.position;
            }
        }
        else
        {
            if (Vector3.Distance(lastFrame, rootBone.position) > 0.005) // If the body has moved between this frame and the last
            {
                // Calculate the direction the body is moving in
                Vector3 dir = rootBone.position - lastFrame;
                dir.Normalize();
                dir.y = 0;
                moveDir.position = rootBone.position + dir * stepSize;
            }
            else
            {
                moveDir.position = rootBone.position;
            }

            lastFrame = rootBone.position;
        }
    }
}
