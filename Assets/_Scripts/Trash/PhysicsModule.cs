using UnityEngine;

namespace ActiveRagdoll
{
    public class PhysicsModule : MonoBehaviour
    {
        private ActiveRagdoll activeRagdoll;
        public float uprightTorque = 10000f;
        public float turningTorque = 500f;
        public float customTorsoAngularDrag = 0.05f;
        [HideInInspector] public Vector3 targetDirection;

        void Awake()
        {
            activeRagdoll = GetComponent<ActiveRagdoll>();
        }

        void FixedUpdate()
        {
            var toUpright = Quaternion.FromToRotation(activeRagdoll.physicalTorso.transform.up, Vector3.up);
            var torque = new Vector3(toUpright.x, toUpright.y, toUpright.z) * uprightTorque;
            activeRagdoll.physicalTorso.AddTorque(torque);

            float angleToTarget = Vector3.SignedAngle(activeRagdoll.physicalTorso.transform.forward, targetDirection, Vector3.up);
            float turnTorquePercent = angleToTarget / 180f;
            activeRagdoll.physicalTorso.AddRelativeTorque(0, turnTorquePercent * turningTorque, 0);

            ApplyCustomDrag();
        }

        private void ApplyCustomDrag()
        {
            var angVel = activeRagdoll.physicalTorso.angularVelocity;
            angVel -= (Mathf.Pow(angVel.magnitude, 2) * customTorsoAngularDrag) * angVel.normalized;
            activeRagdoll.physicalTorso.angularVelocity = angVel;
        }
    }
}