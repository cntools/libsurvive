using UnityEngine;
using System.Collections;

public class FlyCamera : MonoBehaviour {

	/*
	Writen by Windexglow 11-13-10.  Use it, edit it, steal it I don't care.
	Converted to C# 27-02-13 - no credit wanted.
	Simple flycam I made, since I couldn't find any others made public.
	Made simple to use (drag and drop, done) for regular keyboard layout
	wasd : basic movement
	shift : Makes camera accelerate
	space : Moves camera on X and Z axis only.  So camera doesn't gain any height*/

	float mainSpeed = 100.0f; // regular speed
	float shiftAdd = 250.0f;  // multiplied by how long shift is held.  Basically running
	float maxShift = 1000.0f; // Maximum speed when holdin gshift
	float camSens = 0.5f;	  // How sensitive it with mouse
	// private Vector3? lastMouse = new Vector3(255,255,255); //kind of in the middle of the screen, rather than at the
	// top (play)
	private float totalRun = 1.0f;

	void Update() {
		/*
		if (lastMouse != null)
		{
			Vector3 m = (Vector3)lastMouse;
			lastMouse = Input.mousePosition - m;
			lastMouse = new Vector3(-m.y * camSens, m.x * camSens, 0);
			lastMouse = new Vector3(transform.eulerAngles.x + m.x, transform.eulerAngles.y + m.y, 0);
			transform.eulerAngles = m;
		}

		lastMouse =  Input.mousePosition;
		*/

		// Mouse  camera angle done.

		// Keyboard commands
		Vector3 p = GetBaseInput();
		if (Input.GetKey(KeyCode.LeftShift)) {
			totalRun += Time.deltaTime;
			p = p * totalRun * shiftAdd;
			p.x = Mathf.Clamp(p.x, -maxShift, maxShift);
			p.y = Mathf.Clamp(p.y, -maxShift, maxShift);
			p.z = Mathf.Clamp(p.z, -maxShift, maxShift);
		} else {
			totalRun = Mathf.Clamp(totalRun * 0.5f, 1f, 1000f);
			p = p * mainSpeed;
		}

		p = p * Time.deltaTime;
		Vector3 newPosition = transform.position;
		if (!Input.GetKey(KeyCode.Space)) { // If player wants to move on X and Z axis only
			transform.Translate(p);
			newPosition.x = transform.position.x;
			newPosition.z = transform.position.z;
			transform.position = newPosition;
		} else {
			transform.Translate(p);
		}
	}

	private Vector3 GetBaseInput() { // returns the basic values, if it's 0 than it's not active.
		Vector3 p_Velocity = new Vector3();
		var s = .1f;
		if (Input.GetKey(KeyCode.W)) {
			p_Velocity += new Vector3(0, 0, s);
		}
		if (Input.GetKey(KeyCode.S)) {
			p_Velocity += new Vector3(0, 0, -s);
		}
		if (Input.GetKey(KeyCode.A)) {
			p_Velocity += new Vector3(-s, 0, 0);
		}
		if (Input.GetKey(KeyCode.D)) {
			p_Velocity += new Vector3(s, 0, 0);
		}
		return p_Velocity;
	}
}
