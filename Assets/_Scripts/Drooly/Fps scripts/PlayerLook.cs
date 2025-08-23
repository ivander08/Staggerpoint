using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerLook : MonoBehaviour
{
    [SerializeField] private string mouseXInputName, mouseYInputName;
    [SerializeField] private float mouseSensitivity;
    [SerializeField] private Transform playerBody;
    private float xAxisClamp;
    private void Awake()
    {
        Cursor.lockState = CursorLockMode.Locked;
        xAxisClamp = 0.0f;
    }
    private void toggleCursor()
    {
        if (Cursor.lockState == CursorLockMode.Locked)
        {
            Cursor.lockState = CursorLockMode.None;
        }
        else
        {
            Cursor.lockState = CursorLockMode.Locked;
        }
    }
    private void Update()
    {
        CameraRotation();
        if (Input.GetKeyDown(KeyCode.E)) { toggleCursor(); }
        if (Input.GetKeyDown(KeyCode.L)) { toggleCursor(); }
    }
    private void CameraRotation()
    {
        float mouseX = Input.GetAxis(mouseXInputName) * mouseSensitivity * Time.fixedDeltaTime;
        float mouseY = Input.GetAxis(mouseYInputName) * mouseSensitivity * Time.fixedDeltaTime;
        xAxisClamp += mouseY;
        if (xAxisClamp > 90.0f)
        {
            xAxisClamp = 90.0f;
            mouseY = 0.0f;
            ClampXAxisRotationToValue(270.0f);
        }
        else if (xAxisClamp < -90.0f)
        {
            xAxisClamp = -90.0f;
            mouseY = 0.0f;
            ClampXAxisRotationToValue(90.0f);
        }
        transform.Rotate(Vector3.left * mouseY);
        playerBody.Rotate(Vector3.up * mouseX);
    }
    private void ClampXAxisRotationToValue(float value)
    {
        Vector3 eulerRotation = transform.eulerAngles;
        eulerRotation.x = value;
        transform.eulerAngles = eulerRotation;
    }
}
