using UnityEngine;

namespace ActiveRagdoll
{
    public class ActiveRagdoll : MonoBehaviour
    {
        public Animator animatedAnimator;
        public Rigidbody physicalTorso;
        [HideInInspector] public ConfigurableJoint[] joints;
        [HideInInspector] public Transform[] animatedBones;
        private Quaternion[] initialJointsRotation;

        void Awake()
        {
            joints = physicalTorso.GetComponentsInChildren<ConfigurableJoint>();
            animatedBones = animatedAnimator.transform.GetComponentsInChildren<Transform>();

            var allRigidbodies = GetComponentsInChildren<Rigidbody>();
            foreach (var rb in allRigidbodies)
            {
                rb.maxAngularVelocity = 50;
            }

            initialJointsRotation = new Quaternion[joints.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                initialJointsRotation[i] = joints[i].transform.localRotation;
            }
        }

        public void MatchMotion()
        {
            for (int i = 0; i < joints.Length; i++)
            {
                ConfigurableJointExtensions.SetTargetRotationLocal(joints[i], animatedBones[i + 1].localRotation, initialJointsRotation[i]);
            }
        }
    }
}