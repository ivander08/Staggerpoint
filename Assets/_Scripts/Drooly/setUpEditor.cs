using UnityEngine;
using UnityEditor;
#if UNITY_EDITOR
[CustomEditor(typeof(ActiveRagdollSetUp))]
public class setUpEditor : Editor
{
    public override void OnInspectorGUI()
    {
        ActiveRagdollSetUp script = (ActiveRagdollSetUp)target;
        DrawDefaultInspector();

        if (GUILayout.Button("Create")) { script.setUp(); }
            
    }
}
#endif
