using UnityEngine;
using UnityEditor;
using System.Text;

public class HierarchyPrinter : EditorWindow
{
    GameObject targetObject;
    Vector2 scrollPos;
    string output = "";

    [MenuItem("Tools/Hierarchy Printer")]
    public static void ShowWindow()
    {
        GetWindow<HierarchyPrinter>("Hierarchy Printer");
    }

    void OnGUI()
    {
        targetObject = (GameObject)EditorGUILayout.ObjectField("Target Object", targetObject, typeof(GameObject), true);

        if (GUILayout.Button("Print Hierarchy") && targetObject != null)
        { //
            StringBuilder sb = new StringBuilder();
            PrintHierarchy(targetObject.transform, 0, sb);
            output = sb.ToString();
            Debug.Log(output); // also logs to console
        }

        scrollPos = EditorGUILayout.BeginScrollView(scrollPos);
        EditorGUILayout.TextArea(output, GUILayout.ExpandHeight(true));
        EditorGUILayout.EndScrollView();
    }

    void PrintHierarchy(Transform t, int indent, StringBuilder sb)
    {
        // Draw the tree indentation
        string indentStr = "";
        for (int i = 0; i < indent; i++)
        {
            indentStr += "│  "; // vertical bar + spacing
        }

        // collect component names
        Component[] comps = t.GetComponents<Component>();
        string compList = "";
        foreach (var c in comps)
        {
            if (c == null) continue;
            compList += c.GetType().Name + ", ";
        }
        if (compList.EndsWith(", ")) compList = compList.Substring(0, compList.Length - 2);

        // print line
        sb.AppendLine($"{indentStr}└─ {t.name} ({compList})");

        // recurse for children
        foreach (Transform child in t)
        {
            PrintHierarchy(child, indent + 1, sb);
        }
    }

}
