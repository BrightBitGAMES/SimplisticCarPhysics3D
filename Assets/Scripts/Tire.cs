﻿using UnityEngine;

public class Tire : MonoBehaviour {

	public float RestingWeight { get; set; }
	public float ActiveWeight { get; set; }
	public float Grip { get; set; }
	public float FrictionForce { get; set; }
	public float AngularVelocity { get; set; }
	public float Torque { get; set; }

	public float Radius = 0.5f;

    private float      TrailDuration = 5;
    private bool       TrailActive;
    private GameObject Skidmark;

    public Vector3 LocalPosition => transform.parent.localPosition + transform.localPosition;

	public void SetTrailActive(bool active)
    {
		if (active && !TrailActive)
        {
			// These should be pooled and re-used
            Skidmark = Instantiate(Resources.Load ("Skidmark") as GameObject, transform.position, transform.rotation);

            var skidmarkComponent = Skidmark.GetComponent<TrailRenderer>();

            //Fix issue where skidmarks draw at 0,0,0 at slow speeds
            skidmarkComponent.Clear();
            skidmarkComponent.time         = TrailDuration;
            skidmarkComponent.sortingOrder = 0;

            Skidmark.transform.parent        = this.transform;
            Skidmark.transform.localPosition = Vector2.zero;
		}
        else if (!active && TrailActive)
        {			
            Skidmark.transform.parent = null;
            Destroy (Skidmark.gameObject, TrailDuration); 
        }

		TrailActive = active;
	}

}
