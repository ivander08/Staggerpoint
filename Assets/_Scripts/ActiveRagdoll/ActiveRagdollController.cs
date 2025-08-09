using UnityEngine;

namespace ActiveRagdoll
{
    public class ActiveRagdollController : MonoBehaviour
    {
        [SerializeField] private Transform animatedTorso;
        [SerializeField] private Transform physicalTorso;
        private ConfigurableJoint[] physicalJoints;
        private Transform[] animatedBones;
        private Quaternion[] initialJointsRotation;

        void Start()
        {
            physicalJoints = physicalTorso.GetComponentsInChildren<ConfigurableJoint>();
            animatedBones = animatedTorso.GetComponentsInChildren<Transform>();

            initialJointsRotation = new Quaternion[physicalJoints.Length];

            for (int i = 0; i < physicalJoints.Length; i++)
            {
                initialJointsRotation[i] = physicalJoints[i].transform.localRotation;
            }
        }

        void FixedUpdate()
        {
            for (int i = 0; i < physicalJoints.Length; i++)
            {
                // Get the corresponding joint and animated bone.
                ConfigurableJoint joint = physicalJoints[i];
                Transform animatedBone = animatedBones[i + 1]; // We use i + 1 here, I'll explain why!
                Quaternion initialRotation = initialJointsRotation[i];

                ConfigurableJointExtensions.SetTargetRotationLocal(joint, animatedBone.localRotation, initialRotation);
            }
        }
    }
}