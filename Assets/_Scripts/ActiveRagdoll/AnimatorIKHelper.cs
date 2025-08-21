
using UnityEngine;

namespace ActiveRagdoll
{
    [RequireComponent(typeof(Animator))]
    public class AnimatorIKHelper : MonoBehaviour
    {
        private Animator animator;

        public float LookAtWeight { get; set; } = 0;
        public Vector3 LookAtPoint { get; set; } = Vector3.zero;

        public float RightHandWeight { get; set; } = 0;
        public Vector3 RightHandPosition { get; set; } = Vector3.zero;
        public Quaternion RightHandRotation { get; set; } = Quaternion.identity;
        public Vector3 RightElbowHint { get; set; } = Vector3.zero;

        public float LeftHandWeight { get; set; } = 0;
        public Vector3 LeftHandPosition { get; set; } = Vector3.zero;
        public Quaternion LeftHandRotation { get; set; } = Quaternion.identity;
        public Vector3 LeftElbowHint { get; set; } = Vector3.zero;

        void Awake()
        {
            animator = GetComponent<Animator>();
        }

        private void OnAnimatorIK(int layerIndex)
        {
            animator.SetLookAtWeight(LookAtWeight);
            animator.SetLookAtPosition(LookAtPoint);

            animator.SetIKPositionWeight(AvatarIKGoal.RightHand, RightHandWeight);
            animator.SetIKRotationWeight(AvatarIKGoal.RightHand, RightHandWeight);
            animator.SetIKHintPositionWeight(AvatarIKHint.RightElbow, RightHandWeight);
            animator.SetIKPosition(AvatarIKGoal.RightHand, RightHandPosition);
            animator.SetIKRotation(AvatarIKGoal.RightHand, RightHandRotation);
            animator.SetIKHintPosition(AvatarIKHint.RightElbow, RightElbowHint);

            animator.SetIKPositionWeight(AvatarIKGoal.LeftHand, LeftHandWeight);
            animator.SetIKRotationWeight(AvatarIKGoal.LeftHand, LeftHandWeight);
            animator.SetIKHintPositionWeight(AvatarIKHint.LeftElbow, LeftHandWeight);
            animator.SetIKPosition(AvatarIKGoal.LeftHand, LeftHandPosition);
            animator.SetIKRotation(AvatarIKGoal.LeftHand, LeftHandRotation);
            animator.SetIKHintPosition(AvatarIKHint.LeftElbow, LeftElbowHint);

        }
    }
}