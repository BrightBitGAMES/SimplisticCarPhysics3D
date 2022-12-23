using UnityEngine;
using UnityEngine.Serialization;

public class Camera2D: MonoBehaviour
{
	[FormerlySerializedAs("Area")] public Transform target;

    private Vector3 CurrentPosition;
    private Vector3 CurrentScale;

    private void Set()
	{
		var position = target.transform.position;

		var camPosition = position;
		var point = Camera.main.WorldToViewportPoint(camPosition);
		var delta = camPosition - Camera.main.ViewportToWorldPoint(new Vector3(0.5f, 0.5f, point.z));
		var destination = transform.position + delta;

		transform.position = destination;
	}

	public void LateUpdate()
    {
        if (CurrentPosition == target.transform.position) return;

        CurrentPosition = target.transform.position;
        Set();
    }
}
