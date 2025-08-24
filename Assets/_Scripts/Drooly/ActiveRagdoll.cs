using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActiveRagdoll : MonoBehaviour
{
    [Header("Core References")]
    public Transform rootBone;
    public Transform moveDir;

    [Header("IK Targets")]
    public Transform leftFoot;
    public Transform rightFoot;

    [Header("Standing Position Values")]
    public float footGap;
    public float standHeight;

    [Header("Step Values")]
    public float stepDist;
    public float stepSize;
    public float stepDuration;
    public float stepHeight;

    [Header("Physics")]
    public bool physics = false;
    public int rotationForce = 10;
    public int rotationSmoothness = 1;
    public LayerMask ragdollLayer;

    private Vector3 _lastFrame;
    private Transform _leftSide, _rightSide;
    private Vector3 _leftTarget, _rightTarget;

    [HideInInspector]
    public ConfigurableJoint joint;
    [HideInInspector]
    public Rigidbody connectedBody;

    private Rigidbody _rb;

    [HideInInspector]
    public bool falling;

    void Awake()
    {
        _rb = rootBone.GetComponent<Rigidbody>();

        connectedBody = new GameObject("joint").AddComponent<Rigidbody>();
        connectedBody.transform.parent = transform;
        connectedBody.isKinematic = true;
        setupJoint();
    }

    void Start()
    {
        moveDir.position = rootBone.position;
        _lastFrame = rootBone.position;

        _leftSide = new GameObject("left side").transform;
        _rightSide = new GameObject("right side").transform;

        _leftSide.parent = moveDir;
        _rightSide.parent = moveDir;

        _leftSide.localPosition = new Vector3(footGap, 0, 0);
        _rightSide.localPosition = new Vector3(-footGap, 0, 0);
    }

    void Update()
    {
        setMoveDir();

        Physics.Raycast(_leftSide.position, Vector3.down, out RaycastHit hitL, 5, ~ragdollLayer);
        Physics.Raycast(_rightSide.position, Vector3.down, out RaycastHit hitR, 5, ~ragdollLayer);

        if (hitL.collider == null || hitR.collider == null)
        {
            falling = true;
        }
        else
        {
            falling = false;
        }

        float leftDist = Vector3.Distance(hitL.point, _leftTarget);
        float rightDist = Vector3.Distance(hitR.point, _rightTarget);

        if (leftDist >= stepDist && rightDist >= stepDist)
        {
            if (rightDist > leftDist)
            {
                _rightTarget = hitL.point;
                StartCoroutine(step(rightFoot, _rightTarget));
            }
            else
            {
                _leftTarget = hitR.point;
                StartCoroutine(step(leftFoot, _leftTarget));
            }
        }

        leftFoot.eulerAngles = new Vector3(leftFoot.eulerAngles.x, rootBone.eulerAngles.y, leftFoot.eulerAngles.z);
        rightFoot.eulerAngles = new Vector3(rightFoot.eulerAngles.x, rootBone.eulerAngles.y, rightFoot.eulerAngles.z);

        moveDir.eulerAngles = new Vector3(moveDir.eulerAngles.x, rootBone.eulerAngles.y, moveDir.eulerAngles.z);

        if (!falling)
        {
            connectedBody.transform.position = new Vector3(
                rootBone.position.x,
                (hitL.point.y + hitR.point.y) / 2 + standHeight,
                rootBone.position.z
            );
        }
    }

    private IEnumerator step(Transform foot, Vector3 target)
    {
        Vector3 startPoint = foot.position;
        Vector3 centerPoint = (startPoint + target) / 2;
        centerPoint.y = target.y + stepHeight;

        float timeElapsed = 0;

        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / stepDuration;

            foot.position = Vector3.Lerp(
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
        if (joint != null)
            Destroy(joint);

        joint = rootBone.gameObject.AddComponent<ConfigurableJoint>();

        JointDrive driveX = joint.angularXDrive;
        JointDrive driveYZ = joint.angularYZDrive;
        JointDrive driveY = joint.yDrive;

        driveX.positionSpring = rotationForce;
        driveX.positionDamper = rotationSmoothness;

        driveYZ.positionSpring = rotationForce;
        driveYZ.positionDamper = rotationSmoothness;

        driveY.positionSpring = 300;
        driveY.positionDamper = 10;

        joint.angularXDrive = driveX;
        joint.angularYZDrive = driveYZ;
        joint.yDrive = driveY;

        connectedBody.transform.position = rootBone.position;
        connectedBody.transform.rotation = rootBone.rotation;
        joint.connectedBody = connectedBody;
        connectedBody.transform.rotation = Quaternion.identity;
    }

    public void setMoveDir()
    {
        if (physics)
        {
            if (_rb.velocity.magnitude > 0.3f)
            {
                Vector3 dir = _rb.velocity;
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
            if (Vector3.Distance(_lastFrame, rootBone.position) > 0.005f)
            {
                Vector3 dir = rootBone.position - _lastFrame;
                dir.Normalize();
                dir.y = 0;
                moveDir.position = rootBone.position + dir * stepSize;
            }
            else
            {
                moveDir.position = rootBone.position;
            }

            _lastFrame = rootBone.position;
        }
    }
}