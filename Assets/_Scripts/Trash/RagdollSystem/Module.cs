using UnityEngine;

namespace Staggerpoint
{
    [RequireComponent(typeof(RagdollController))]
    public abstract class Module : MonoBehaviour
    {
        protected RagdollController _ragdollController;

        protected virtual void Awake()
        {
            _ragdollController = GetComponent<RagdollController>();
        }
    }
}