using UnityEngine;

namespace Staggerpoint
{
    public class RagdollAnimation : MonoBehaviour
    {
        private RagdollController _ragdollController;
        private Transform[] _animatedBones;
        private Quaternion[] _initialJointRotations;

        void Awake()
        {
            _ragdollController = GetComponent<RagdollController>();
        }

        void Start()
        {
            if (_ragdollController != null)
            {
                _animatedBones = _ragdollController.AnimatedTorso.GetComponentsInChildren<Transform>();
            }

            _initialJointRotations = new Quaternion[_ragdollController.Joints.Length];
            
            for (int i = 0; i < _ragdollController.Joints.Length; i++)
            {
                _initialJointRotations[i] = _ragdollController.Joints[i].targetRotation;
            }
        }

        void FixedUpdate()
        {
            if (_ragdollController.Joints.Length == 0) return;

            for (int i = 0; i < _ragdollController.Joints.Length; i++)
            {
                ConfigurableJoint joint = _ragdollController.Joints[i];
                Transform animatedBone = _animatedBones[i + 1];
                joint.SetTargetRotationLocal(animatedBone.localRotation, _initialJointRotations[i]);
            }
        }
    }
}