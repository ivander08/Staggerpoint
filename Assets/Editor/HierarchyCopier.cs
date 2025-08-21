using UnityEngine;
using UnityEditor;

public class HierarchyCopier
{
    [MenuItem("Tools/Copy Hierarchy")]
    static void CopyHierarchy()
    {
        GameObject[] roots = UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects();
        System.Text.StringBuilder sb = new System.Text.StringBuilder();

        foreach (GameObject root in roots)
        {
            AppendObject(root.transform, 0, sb);
        }

        EditorGUIUtility.systemCopyBuffer = sb.ToString();
        Debug.Log("Hierarchy copied to clipboard!");
    }

    static void AppendObject(Transform obj, int indent, System.Text.StringBuilder sb)
    {
        sb.AppendLine(new string(' ', indent * 2) + "- " + obj.name);

        foreach (Transform child in obj)
        {
            AppendObject(child, indent + 1, sb);
        }
    }
}
