using System.Collections;
using UnityEngine;

/// <summary>
/// Detects weapon impacts and triggers recoil responses.
/// Attach this to any weapon GameObject that should have impact recoil.
/// </summary>
public class WeaponImpactRecoil : MonoBehaviour
{
    [Header("Impact Detection")]
    [Tooltip("Minimum impact force (in Newtons) to trigger recoil")]
    [SerializeField] private float impactThreshold = 150f;

    [Tooltip("Multiplier for recoil force (higher = more dramatic recoil)")]
    [SerializeField] private float recoilMultiplier = 0.3f;

    [Header("Joint Weakening")]
    [Tooltip("How long the weapon joint stays weak after impact")]
    [SerializeField] private float jointWeakeningDuration = 0.15f;

    [Tooltip("Spring strength while joint is weakened")]
    [SerializeField] private float weakenedSpring = 5000f;

    [Tooltip("Damper strength while joint is weakened")]
    [SerializeField] private float weakenedDamper = 500f;

    [Tooltip("Max force while joint is weakened")]
    [SerializeField] private float weakenedMaxForce = 20000f;

    [Header("Normal Joint Strength")]
    [Tooltip("Spring strength during normal holding")]
    [SerializeField] private float normalSpring = 80000f;

    [Tooltip("Damper strength during normal holding")]
    [SerializeField] private float normalDamper = 3000f;

    [Tooltip("Max force during normal holding")]
    [SerializeField] private float normalMaxForce = 150000f;

    [Header("Debug")]
    [SerializeField] private bool showDebugInfo = true;
    [SerializeField] private bool showDebugVisuals = true;

    // [Header("Impact Amplification")]
    // [Tooltip("Multiplies impact force delivered to OTHER objects (doesn't affect recoil on you)")]
    // [SerializeField] private float outgoingImpactMultiplier = 1.0f;

    // Events
    public event System.Action<RecoilData> OnRecoilTriggered;

    // Internal state
    private Weapon _weapon;
    private Coroutine _rightHandRecoilCoroutine;
    private Coroutine _leftHandRecoilCoroutine;

    // Debug visualization
    private Vector3 _lastImpactPoint;
    private Vector3 _lastRecoilDirection;
    private float _lastImpactForce;
    private float _debugVisualizationTimer;

    /// <summary>
    /// Data structure containing all information about a recoil event
    /// </summary>
    public struct RecoilData
    {
        public bool isRightHand;
        public Rigidbody handRigidbody;
        public ConfigurableJoint weaponJoint;
        public Vector3 recoilDirection;
        public float recoilForce;
        public float impactForce;
        public Vector3 impactPoint;
    }

    void Awake()
    {
        _weapon = GetComponent<Weapon>();

        if (_weapon == null)
        {
            Debug.LogError($"WeaponImpactRecoil on '{name}' requires a Weapon component!", this);
            enabled = false;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        // Calculate impact force (Newtons)
        float impactForce = collision.impulse.magnitude / Time.fixedDeltaTime;

        // Only process impacts above threshold
        if (impactForce < impactThreshold)
            return;

        // Get collision data
        ContactPoint contact = collision.GetContact(0);
        Vector3 impactPoint = contact.point;
        Vector3 impactNormal = contact.normal;

        // Store for debug visualization
        if (showDebugVisuals)
        {
            _lastImpactPoint = impactPoint;
            _lastRecoilDirection = impactNormal;
            _lastImpactForce = impactForce;
            _debugVisualizationTimer = 0.5f;
        }

        if (showDebugInfo)
        {
            Debug.Log($"<color=yellow>[WEAPON IMPACT]</color> Force: {impactForce:F0}N | Object: {collision.gameObject.name}");
        }

        // Process recoil for each hand holding the weapon
        if (_weapon.IsHeldByRightHand)
        {
            ProcessRecoil(true, impactForce, impactNormal, impactPoint);
        }

        if (_weapon.IsHeldByLeftHand)
        {
            ProcessRecoil(false, impactForce, impactNormal, impactPoint);
        }
    }

    private void ProcessRecoil(bool isRightHand, float impactForce, Vector3 recoilDirection, Vector3 impactPoint)
    {
        // Get the relevant joint and hand
        ConfigurableJoint weaponJoint = isRightHand ? GetRightHandJoint() : GetLeftHandJoint();
        Rigidbody handRigidbody = weaponJoint?.connectedBody;

        if (weaponJoint == null || handRigidbody == null)
        {
            Debug.LogWarning($"Could not find joint/rigidbody for {(isRightHand ? "right" : "left")} hand recoil");
            return;
        }

        // Calculate recoil force
        float recoilForce = impactForce * recoilMultiplier;

        // Apply impulse to hand (pushes it away from impact)
        handRigidbody.AddForce(recoilDirection * recoilForce, ForceMode.Impulse);

        if (showDebugInfo)
        {
            Debug.Log($"<color=cyan>[RECOIL]</color> {(isRightHand ? "Right" : "Left")} hand | Force: {recoilForce:F0}N | Direction: {recoilDirection}");
        }

        // Weaken joint temporarily
        if (isRightHand)
        {
            if (_rightHandRecoilCoroutine != null) StopCoroutine(_rightHandRecoilCoroutine);
            _rightHandRecoilCoroutine = StartCoroutine(TemporaryJointWeakening(weaponJoint, isRightHand));
        }
        else
        {
            if (_leftHandRecoilCoroutine != null) StopCoroutine(_leftHandRecoilCoroutine);
            _leftHandRecoilCoroutine = StartCoroutine(TemporaryJointWeakening(weaponJoint, isRightHand));
        }

        // Fire event for ArmController to respond
        RecoilData recoilData = new RecoilData
        {
            isRightHand = isRightHand,
            handRigidbody = handRigidbody,
            weaponJoint = weaponJoint,
            recoilDirection = recoilDirection,
            recoilForce = recoilForce,
            impactForce = impactForce,
            impactPoint = impactPoint
        };

        OnRecoilTriggered?.Invoke(recoilData);
    }

    private IEnumerator TemporaryJointWeakening(ConfigurableJoint joint, bool isRightHand)
    {
        // Create weakened drive settings
        JointDrive weakDrive = new JointDrive
        {
            positionSpring = weakenedSpring,
            positionDamper = weakenedDamper,
            maximumForce = weakenedMaxForce
        };

        // Apply weakened drives
        joint.xDrive = weakDrive;
        joint.yDrive = weakDrive;
        joint.zDrive = weakDrive;
        joint.angularXDrive = weakDrive;
        joint.angularYZDrive = weakDrive;

        if (showDebugInfo)
        {
            Debug.Log($"<color=orange>[JOINT WEAKENED]</color> {(isRightHand ? "Right" : "Left")} hand for {jointWeakeningDuration:F2}s");
        }

        // Wait for duration
        yield return new WaitForSeconds(jointWeakeningDuration);

        // Restore normal strength
        JointDrive normalDrive = new JointDrive
        {
            positionSpring = normalSpring,
            positionDamper = normalDamper,
            maximumForce = normalMaxForce
        };

        joint.xDrive = normalDrive;
        joint.yDrive = normalDrive;
        joint.zDrive = normalDrive;
        joint.angularXDrive = normalDrive;
        joint.angularYZDrive = normalDrive;

        if (showDebugInfo)
        {
            Debug.Log($"<color=green>[JOINT RESTORED]</color> {(isRightHand ? "Right" : "Left")} hand back to normal strength");
        }
    }

    // Helper methods to get joints from Weapon component via reflection
    // (Since the joints are private in Weapon.cs)
    private ConfigurableJoint GetRightHandJoint()
    {
        var field = typeof(Weapon).GetField("rightHandJoint", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        return field?.GetValue(_weapon) as ConfigurableJoint;
    }

    private ConfigurableJoint GetLeftHandJoint()
    {
        var field = typeof(Weapon).GetField("leftHandJoint", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        return field?.GetValue(_weapon) as ConfigurableJoint;
    }

    void Update()
    {
        // Decay debug visualization timer
        if (_debugVisualizationTimer > 0)
        {
            _debugVisualizationTimer -= Time.fixedDeltaTime;
        }
    }

    void OnDrawGizmos()
    {
        if (!showDebugVisuals || _debugVisualizationTimer <= 0)
            return;

        // Draw impact point
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(_lastImpactPoint, 0.05f);

        // Draw recoil direction
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(_lastImpactPoint, _lastImpactPoint + _lastRecoilDirection * 0.3f);

        // Draw force magnitude indicator
        Gizmos.color = Color.cyan;
        float forceMagnitude = Mathf.Clamp(_lastImpactForce / 1000f, 0.1f, 1f);
        Gizmos.DrawWireSphere(_lastImpactPoint, forceMagnitude * 0.2f);

#if UNITY_EDITOR
        // Draw text label
        Vector3 labelPos = _lastImpactPoint + Vector3.up * 0.2f;
        UnityEditor.Handles.Label(labelPos, $"Impact: {_lastImpactForce:F0}N");
#endif
    }
}