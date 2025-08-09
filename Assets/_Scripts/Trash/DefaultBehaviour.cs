using UnityEngine;

namespace ActiveRagdoll
{
    public class DefaultBehaviour : MonoBehaviour
    {
        private ActiveRagdoll activeRagdoll;
        private PhysicsModule physicsModule;

        public float walkSpeedMultiplier = 1.5f;

        void Awake()
        {
            activeRagdoll = GetComponent<ActiveRagdoll>();
            physicsModule = GetComponent<PhysicsModule>();
        }

        void Update()
        {
            float verticalInput = Input.GetAxis("Vertical");
            activeRagdoll.animatedAnimator.SetFloat("Speed", verticalInput * walkSpeedMultiplier);

            float horizontalInput = Input.GetAxis("Horizontal");
            transform.Rotate(0, horizontalInput * 2f, 0);
        }

        void FixedUpdate()
        {
            physicsModule.targetDirection = transform.forward;
            activeRagdoll.MatchMotion();
        }
    }
}