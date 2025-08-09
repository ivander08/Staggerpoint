using UnityEngine;
using UnityEngine.Video;

namespace Staggerpoint
{
    public class RagdollPhysics : Module
    {
        [SerializeField]
        private JointDrive _stabilizerJointDrive = new JointDrive
        {
            positionSpring = 5000f,
            positionDamper = 50f,
            maximumForce = Mathf.Infinity
        };
        private Rigidbody _stabilizerRigidbody;
        public Vector3 TargetDirection { get; set; }

        void Start()
        {
            GameObject stabilizerGO = new GameObject("Stabilizer");
            stabilizerGO.transform.SetParent(transform);

            _stabilizerRigidbody = stabilizerGO.AddComponent<Rigidbody>();
            _stabilizerRigidbody.isKinematic = true;

            ConfigurableJoint joint = stabilizerGO.AddComponent<ConfigurableJoint>();
            joint.connectedBody = _ragdollController.PhysicalTorso;

            joint.angularXDrive = _stabilizerJointDrive;
            joint.angularYZDrive = _stabilizerJointDrive;
        }

        void FixedUpdate()
        {
            // match anchor position to ragdoll's hip
            _stabilizerRigidbody.MovePosition(_ragdollController.PhysicalTorso.position);

            // rotate anchor to face targetdirection
            if (TargetDirection.sqrMagnitude > 0.01f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(TargetDirection, Vector3.up);
                _stabilizerRigidbody.MoveRotation(targetRotation);
            }
        }

    }
}