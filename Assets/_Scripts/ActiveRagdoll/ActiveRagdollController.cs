using UnityEngine;

namespace ActiveRagdoll
{
    public class ActiveRagdollController : MonoBehaviour
    {
        // --- Private References ---
        private Rigidbody rootRigidbody;        // The Rigidbody of this main parent GameObject.
        private ConfigurableJoint[] physicalJoints; // Array to hold all joints of the physical body.
        private Transform[] animatedBones;      // Array to hold all bone transforms of the animated body.
        private Quaternion[] initialJointsRotation; // Stores the starting rotation of each joint.
        private Rigidbody[] allRigidbodies;     // Array to hold all rigidbodies for initial setup.
        private int speedParameterId;           // An optimized ID for the "Speed" parameter in the Animator.

        // --- Core References (Set in Inspector) ---
        [Header("Core References")]
        [Tooltip("The invisible, kinematic Rigidbody that acts as a balance and rotation target.")]
        [SerializeField] private Transform stabilizer;
        [Tooltip("The root bone of the visible, animated character (e.g., the Hips).")]
        [SerializeField] private Transform animatedTorso;
        [Tooltip("The Rigidbody of the root bone of the physical ragdoll (e.g., the Hips).")]
        [SerializeField] private Rigidbody physicalTorso;
        [Tooltip("The Animator component on the animated character.")]
        [SerializeField] private Animator animatedAnimator;

        // --- Movement Settings (Set in Inspector) ---
        [Header("Movement Settings")]
        [Tooltip("How fast the walk animation plays when moving forward.")]
        [SerializeField] private float walkSpeedMultiplier = 1.5f;
        [Tooltip("How fast the character turns left and right.")]
        [SerializeField] private float turnSpeed = 2f;


        // ##################
        // ## INITIALIZATION ##
        // ##################

        void Start()
        {
            // --- 1. Get Core Components ---
            // Get the Rigidbody attached to this parent GameObject.
            rootRigidbody = GetComponent<Rigidbody>();

            // --- 2. Gather All Ragdoll Parts ---
            // Find every joint and rigidbody in the children of the physical torso.
            physicalJoints = physicalTorso.GetComponentsInChildren<ConfigurableJoint>();
            allRigidbodies = physicalTorso.GetComponentsInChildren<Rigidbody>();
            // Find every bone transform in the children of the animated torso.
            animatedBones = animatedTorso.GetComponentsInChildren<Transform>();

            // --- 3. Configure Physics Properties ---
            // Set a higher rotation speed limit on all parts of the ragdoll.
            // This is crucial to prevent physics from clamping the ragdoll's movements.
            foreach (Rigidbody rb in allRigidbodies)
            {
                rb.maxAngularVelocity = 50;
            }

            // --- 4. Cache Initial Rotations ---
            // Store the default rotation of each joint. This is required by the extension
            // script to correctly calculate the target rotation for motion matching.
            initialJointsRotation = new Quaternion[physicalJoints.Length];
            for (int i = 0; i < physicalJoints.Length; i++)
            {
                initialJointsRotation[i] = physicalJoints[i].transform.localRotation;
            }

            // --- 5. Animator Setup ---
            // Get an optimized integer ID for the "Speed" parameter to improve performance.
            speedParameterId = Animator.StringToHash("Speed");
        }


        // #################
        // ## FRAME UPDATES ##
        // #################

        // Update is called once per frame and is best for handling player input.
        void Update()
        {
            // Read vertical input from W/S keys or a controller stick.
            float verticalInput = Input.GetAxis("Vertical");
            // Tell the animator to set the "Speed" parameter, which will trigger the walk animation.
            animatedAnimator.SetFloat(speedParameterId, verticalInput * walkSpeedMultiplier);
        }

        // FixedUpdate is called in sync with the physics engine (50 times per second by default).
        // All physics calculations and forces should be applied here.
        void FixedUpdate()
        {
            // --- STEP 1: UPDATE THE STABILIZER TARGET ---
            // Move our kinematic "guide" to the physical torso's current position and
            // rotate it to match the player's intended direction. The joint will then
            // apply forces to make the physical torso match this guide.
            stabilizer.GetComponent<Rigidbody>().MovePosition(physicalTorso.position);
            stabilizer.GetComponent<Rigidbody>().MoveRotation(transform.rotation);

            // --- STEP 2: HANDLE PLAYER TURNING ---
            // Read horizontal input and rotate the main parent object. This defines our target direction.
            float horizontalInput = Input.GetAxis("Horizontal");
            transform.Rotate(0, horizontalInput * turnSpeed, 0);

            // --- STEP 3: MOVE THE ROOT CONTROLLER ---
            // Calculate the difference between the root and the physical body.
            Vector3 positionDifference = physicalTorso.position - rootRigidbody.position;
            // Calculate the velocity needed to cover that distance in one physics step.
            Vector3 velocityToTarget = positionDifference / Time.fixedDeltaTime;
            // Apply the velocity to pull the root along with the physical character.
            rootRigidbody.velocity = velocityToTarget;

            // --- STEP 4: SYNC THE ANIMATED BODY ---
            // Keep the hidden animated body in sync with the physical one. This is
            // crucial for systems like IK that read bone positions from the animator.
            SyncAnimatedBody();

            // --- STEP 5: MATCH MOTION ---
            // This is the core of the "active" ragdoll. Loop through every joint
            // and tell it to apply force to match the rotation of its animated counterpart.
            for (int i = 0; i < physicalJoints.Length; i++)
            {
                ConfigurableJoint joint = physicalJoints[i];
                Transform animatedBone = animatedBones[i + 1]; // Use i+1 to account for the root bone offset.
                Quaternion initialRotation = initialJointsRotation[i];

                ConfigurableJointExtensions.SetTargetRotationLocal(joint, animatedBone.localRotation, initialRotation);
            }
        }

        private void SyncAnimatedBody()
        {
            // Calculate the offset from the animated model's root to its hip bone.
            Vector3 animatedOffset = animatedAnimator.transform.position - animatedTorso.position;
            // Reposition the animated model's root so that its hip bone aligns with the physical hip bone.
            animatedAnimator.transform.position = physicalTorso.position + animatedOffset;
            // Match the rotation.
            animatedAnimator.transform.rotation = physicalTorso.rotation;
        }

        // ###########
        // ## GIZMOS ##
        // ###########

        // This function draws debug lines in the Scene view to help visualize what's happening.
        private void OnDrawGizmos()
        {
            // Don't draw if the game isn't running.
            if (!Application.isPlaying)
            {
                return;
            }

            // Draw a BLUE line showing the direction the Stabilizer is aiming.
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(stabilizer.position, stabilizer.forward * 2f);

            // Draw a RED line showing the direction the Physical Torso is actually facing.
            Gizmos.color = Color.red;
            Gizmos.DrawRay(physicalTorso.position, physicalTorso.transform.forward * 1.5f);

            // Draw a YELLOW line showing the "leash" between the root and the physical body.
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(rootRigidbody.position, physicalTorso.position);

            // Draw a GREEN line showing the velocity we apply to the root.
            Gizmos.color = Color.green;
            Gizmos.DrawRay(rootRigidbody.position, rootRigidbody.velocity);
        }
    }
}