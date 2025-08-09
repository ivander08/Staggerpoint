using System.Collections.Generic;
using UnityEngine;

namespace Staggerpoint
{
    public class RagdollController : MonoBehaviour
    {
        [SerializeField] private Animator _animatedAnimator;
        [SerializeField] private Animator _physicalAnimator;

        [SerializeField] private Transform _animatedTorso;
        public Transform AnimatedTorso { get { return _animatedTorso; } }

        [SerializeField] private Transform _physicalTorso;
        public Transform PhysicalTorso { get { return _physicalTorso; } }

        public ConfigurableJoint[] Joints { get; private set; }
        public Rigidbody[] Rigidbodies { get; private set; }

        private void Awake()
        {
            if (_physicalTorso != null)
            {
                Joints = _physicalTorso.GetComponentsInChildren<ConfigurableJoint>();
                Rigidbodies = _physicalTorso.GetComponentsInChildren<Rigidbody>();
            }

            foreach (Rigidbody rb in Rigidbodies)
            {
                rb.solverIterations = 12;
                rb.solverVelocityIterations = 12;
                rb.maxAngularVelocity = 50;
            }
        }

    }
}