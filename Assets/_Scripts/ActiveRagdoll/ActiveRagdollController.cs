using UnityEngine;


namespace ActiveRagdoll
{
    public class ActiveRagdollController : MonoBehaviour
    {
        private Rigidbody rootRigidbody;
        [SerializeField] private Transform stabilizer;

        [SerializeField] private Transform animatedTorso;
        [SerializeField] private Rigidbody physicalTorso;
        [SerializeField] private Animator animatedAnimator;

        private int speedParameterId;
        [SerializeField] private float walkSpeedMultiplier = 1.5f;
        [SerializeField] private float turnSpeed = 2f;

        private ConfigurableJoint[] physicalJoints;
        private Transform[] animatedBones;
        private Quaternion[] initialJointsRotation;
        private Rigidbody[] allRigidbodies;

        void Start()
        {
            rootRigidbody = GetComponent<Rigidbody>();

            physicalJoints = physicalTorso.GetComponentsInChildren<ConfigurableJoint>();
            animatedBones = animatedTorso.GetComponentsInChildren<Transform>();
            allRigidbodies = physicalTorso.GetComponentsInChildren<Rigidbody>();

            foreach (Rigidbody rb in allRigidbodies)
            {
                rb.maxAngularVelocity = 50;
            }

            initialJointsRotation = new Quaternion[physicalJoints.Length];

            for (int i = 0; i < physicalJoints.Length; i++)
            {
                initialJointsRotation[i] = physicalJoints[i].transform.localRotation;
            }

            speedParameterId = Animator.StringToHash("Speed");
        }

        void Update()
        {
            float verticalInput = Input.GetAxis("Vertical");
            animatedAnimator.SetFloat(speedParameterId, verticalInput * walkSpeedMultiplier);
        }

        void FixedUpdate()
        {
            stabilizer.position = physicalTorso.position;
            stabilizer.rotation = transform.rotation;

            float horizontalInput = Input.GetAxis("Horizontal");
            transform.Rotate(0, horizontalInput * turnSpeed, 0);

            Vector3 positionDifference = physicalTorso.position - rootRigidbody.position;
            Vector3 velocityToTarget = positionDifference / Time.fixedDeltaTime;
            rootRigidbody.velocity = new Vector3(velocityToTarget.x, rootRigidbody.velocity.y, velocityToTarget.z);

            for (int i = 0; i < physicalJoints.Length; i++)
            {
                ConfigurableJoint joint = physicalJoints[i];
                Transform animatedBone = animatedBones[i + 1];
                Quaternion initialRotation = initialJointsRotation[i];

                ConfigurableJointExtensions.SetTargetRotationLocal(joint, animatedBone.localRotation, initialRotation);
            }
        }
    }
}