using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using libsurvive;

﻿using System.Collections;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class SurviveObject : MonoBehaviour {
	private Dictionary<string, GameObject> survive_objects;
	SurviveAPI survive;

	void InfoFn(IntPtr ctx, UInt32 loglevl, string fault) { Debug.Log(fault); }

	private GameObject prototypeObject;
	// Start is called before the first frame update
	void Start() {
		survive_objects = new Dictionary<string, GameObject>();
		string[] args = System.Environment.GetCommandLineArgs();
		var argList = new List<string>(args);
		var idx = argList.LastIndexOf("--");
		if (idx < 0) {
			argList.RemoveRange(1, argList.Count - 1);
		} else {
			argList.RemoveRange(1, idx);
		}

		argList.Add("--v");
		argList.Add("10");

		survive = new SurviveAPI(argList.ToArray(), logFunc : InfoFn);
		prototypeObject = gameObject.transform.Find("PrototypeObject") ?.gameObject;
		prototypeObject.SetActive(false);
	}

	private GameObject getObject(string name) {
		if (survive_objects.ContainsKey(name))
			return survive_objects[name];

		GameObject newObj = Instantiate(prototypeObject, gameObject.transform);
		newObj.SetActive(true);

		newObj.GetComponentInChildren<TextMesh>().text = name;

		return survive_objects[name] = newObj;
	}

	// Update is called once per frame
	void Update() {
		SurviveAPIOObject updated;
		while ((updated = survive?.GetNextUpdated()) != null) {
			var updatedObject = getObject(updated.Name);

			Vector3 newPosition = Vector3.zero;
			Quaternion newRotation = Quaternion.identity;
			SurvivePose pose = updated.LatestPose;
			newPosition.x = (float) pose.Pos[0];
			newPosition.y = (float) pose.Pos[1];
			newPosition.z = (float) pose.Pos[2];
			newRotation.w = (float) pose.Rot[0];
			newRotation.x = (float) pose.Rot[1];
			newRotation.y = (float) pose.Rot[2];
			newRotation.z = (float) pose.Rot[3];
			updatedObject.transform.localPosition = newPosition;
			updatedObject.transform.localRotation = newRotation;
		}
	}
	
	void OnApplicationQuit() {
		if (survive != null){
			survive.Close();
			survive.Dispose();		
			survive = null;
		}
	}
}
