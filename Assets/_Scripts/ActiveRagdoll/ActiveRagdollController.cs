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
            stabilizer.GetComponent<Rigidbody>().MovePosition(physicalTorso.position);
            stabilizer.GetComponent<Rigidbody>().MoveRotation(transform.rotation);

            float horizontalInput = Input.GetAxis("Horizontal");
            transform.Rotate(0, horizontalInput * turnSpeed, 0);

            Vector3 positionDifference = physicalTorso.position - rootRigidbody.position;
            Vector3 velocityToTarget = positionDifference / Time.fixedDeltaTime;
            rootRigidbody.velocity = velocityToTarget;

            SyncAnimatedBody();

            for (int i = 0; i < physicalJoints.Length; i++)
            {
                ConfigurableJoint joint = physicalJoints[i];
                Transform animatedBone = animatedBones[i + 1];
                Quaternion initialRotation = initialJointsRotation[i];

                ConfigurableJointExtensions.SetTargetRotationLocal(joint, animatedBone.localRotation, initialRotation);
            }
        }

        private void SyncAnimatedBody()
        {
            Vector3 animatedOffset = animatedAnimator.transform.position - animatedTorso.position;
            animatedAnimator.transform.position = physicalTorso.position + animatedOffset;
            animatedAnimator.transform.rotation = physicalTorso.rotation;
        }

        // --- GIZMOS ---
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            // --- 1. Stabilizer Direction ---
            // Draw a BLUE line showing the direction the Stabilizer is aiming.
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(stabilizer.position, stabilizer.forward * 2f);


            // --- 2. Physical Torso's Actual Direction ---
            // Draw a RED line showing the direction the Physical Torso is actually facing.
            // You can see how the turning force tries to align this red line with the blue line.
            Gizmos.color = Color.red;
            Gizmos.DrawRay(physicalTorso.position, physicalTorso.transform.forward * 1.5f);


            // --- 3. Root to Physical Body Connection ---
            // Draw a YELLOW line showing the "leash" between the root controller and the physical body.
            // This line represents the 'positionDifference' vector we calculate.
            // When a cannon hits, you will see this line stretch out, and the root will quickly catch up.
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(rootRigidbody.position, physicalTorso.position);


            // --- 4. Root's Target Velocity ---
            // Draw a GREEN line showing the velocity we apply to the root.
            // This shows you how fast the controller is trying to move to keep up.
            Gizmos.color = Color.green;
            Gizmos.DrawRay(rootRigidbody.position, rootRigidbody.velocity);
        }
    }
}