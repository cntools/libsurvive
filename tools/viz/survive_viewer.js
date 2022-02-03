var objs = {};
var visible_tolerance = 1608200 * 2;
var downAxes = {};
var angles = {};
var rp_angles = {};
var ctx;
var canvas;
var oldDrawTime = 0;
var timecode = {};
var oldPoseTime = 0, poseCnt = 0;
var oldPose = [ 0, 0, 0 ];
var scene, camera, renderer, floor, fpv_camera;
var fov_scale = 1;

var external_group = new THREE.Group();
var report_in_imu = false;

var raycaster = new THREE.Raycaster();

var mouse = new THREE.Vector2();

var showModel = true;
var showIMU = true;
var useFPV = false;

var tooltipDisplayed = false;
var tooltip;

var fps = 0;
var frames = 0;
var bsd_map = {};

function onMouseMove(event) {
	// calculate mouse position in normalized device coordinates
	// (-1 to +1) for both components

	mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
	mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

	var foundObj = false;
	var intersects = raycaster.intersectObjects(scene.children, true);
	for (var i = 0; i < intersects.length; i++) {
		if (intersects[i].object.tooltip) {
			tooltip.text(intersects[i].object.tooltip);
			tooltip.css({top : event.clientY, left : event.clientX, position : 'absolute', display : 'block'});
			foundObj = true;
			tooltipDisplayed = true;
			break;
		}
	}

	if (!foundObj && tooltipDisplayed) {
		tooltip.css({display : 'none'});
		tooltipDisplayed = false;
	}
}

$(function() { $("#toggleBtn").click(function() { $("#cam").toggle(); }); });
$(function() {
	$("#copy-camera").click(function() {
		navigator.clipboard.writeText(JSON.stringify({'camera': [camera.position.toArray(), camera.quaternion.toArray()]}));
	})
})

document.addEventListener('paste', (event) => {
	const data = JSON.parse(event.clipboardData.getData("TEXT"))
	const cameraParams = data['camera'];
	if(cameraParams) {
		camera.position.set(...cameraParams[0]);
		camera.quaternion.set(...cameraParams[1]);
	}
});

var lhColors = [
	0xecba82, 0x4f3920, 0x8c7070, 0xf4eded, 0x4e6e5d, 0x3bc14a, 0x251351, 0xc97b84, 0xa85751, 0x251351, 0xc7eae4,
	0xa7e8bd, 0xffd972, 0xaba361, 0x732c2c, 0x773344c
];

var lighthouses = {};
function add_lighthouse(idx, p, q) {
	var trails = get_trails({tracker : "LH" + idx, position : p});
	if (lighthouses[idx]) {
		var group = lighthouses[idx];
		group.position.fromArray(p);
		group.verticesNeedUpdate = true;
		group.group_rot.quaternion.fromArray([ q[1], q[2], q[3], q[0] ]);
		group.group_rot.verticesNeedUpdate = true;

		if (trails) {
			trails.geometry.vertices.push(trails.geometry.vertices.shift()); // shift the array
			trails.geometry.vertices[MAX_LINE_POINTS - 1] = new THREE.Vector3(p[0], p[1], p[2]);
			trails.geometry.verticesNeedUpdate = true;
		}

		return;
	}

	var group = new THREE.Group();
	var group_rot = new THREE.Group();
	group.add(group_rot);
	group.group_rot = group_rot;
	lighthouses[idx] = group;

	var lh = new THREE.AxesHelper(1);

	group.position.fromArray(p);
	group_rot.quaternion.fromArray([ q[1], q[2], q[3], q[0] ]);

	var imuGeom = new THREE.Geometry();
	imuGeom.vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));
	var line = new THREE.Line(imuGeom, new THREE.LineBasicMaterial({color : 0xffffff}));
	line.visible = true;
	group.imu = imuGeom;
	group_rot.add(line);

	var height = 10;

	var geometry = new THREE.ConeGeometry(height / Math.cos(60 / 180 * Math.PI), height, 4, 1, true);
	var material = new THREE.MeshBasicMaterial({
		color : 0x1F1FFF,
		opacity : 0.02,
		transparent : true,
		blending : THREE.AdditiveBlending,
		side : THREE.DoubleSide,
		depthTest : false
	});
	var cone = new THREE.Mesh(geometry, material);

	var lhBoxGeom = new THREE.CubeGeometry(.075, .075, .075);

	lhBoxMaterial =
		new THREE.MeshBasicMaterial({color : lhColors[idx], flatShading : false, transparent : true, opacity : .75});
	var lhBox = new THREE.Mesh(lhBoxGeom, lhBoxMaterial);
	lhBox.tooltip = "Lighthouse " + (idx)
	group_rot.add(lhBox);
	console.log(idx, lhColors[idx])

	cone.translateZ(-height / 2);
	cone.rotateZ(Math.PI / 4);
	cone.rotateX(Math.PI / 2);

	group_rot.add(cone);
	group_rot.add(lh);

	{
		var geometry = new THREE.SphereGeometry(1, 16, 16);
		var material =
			new THREE.MeshBasicMaterial({color : 0x0000FF, opacity : .25, transparent : true, side : THREE.DoubleSide});

		var mesh = new THREE.Mesh(geometry, material);
		mesh.position.set(0, 0, 0);

		mesh.scale.set(1e-4, 1e-4, 1e-4)

		let ellipsoid_name = "LH" + idx;
		mesh.visible = false;
		ellipsoids[ellipsoid_name] = mesh

		group_rot.add(mesh);
	}

	scene.add(group);

	if (lighthouse_ups[idx]) {
		lhup(lighthouse_ups[idx])
	}
}

function get_bvalue(key) {
	var bvalue_array = {"WW0" : "FF", "TR0" : "00"};
	var bvalue = bvalue_array[key];

	if (bvalue === undefined) {
		bvalue = 0;
		for (var i = 0; i < key.length; i++) {
			bvalue = Math.sin(bvalue + key.codePointAt(i)) * 10000;
		}
		bvalue = Math.round((bvalue - Math.floor(bvalue)) * 1000) % 0xff;
		bvalue = bvalue.toString(16);
	}
	return bvalue;
}

function survive_timecode_difference(most_recent, least_recent) {
	var diff = 0;
	if (most_recent > least_recent) {
		diff = most_recent - least_recent;
	} else {
		diff = least_recent - most_recent;
	}

	if (diff > 0xFFFFFFFF / 2)
		return 0xFFFFFFFF - diff;
	return diff;
}

function recolorTrackers(when) {
	if (ctx == undefined)
		return;

	for (var key in angles) {
		var colors = [];

		for (var lh = 0; lh < 16; lh++) {
			var bvalue = get_bvalue(key);
			ctx.strokeStyle = "#" + lhColors[lh].toString(16);

			if (angles[key][lh])

				for (var id in angles[key][lh]) {
					var ang = angles[key][lh][id];

					colors[id] = colors[id] || 0;

					if (ang[0] === undefined || ang[1] === undefined)
						continue;

					for (var acode = 0; acode < 2; acode++) {
						if (survive_timecode_difference(when[key], ang[acode][1]) > visible_tolerance)
							continue;

						var augment = 0xf;
						if (lh)
							augment = augment << 8;
						if (acode)
							augment = augment << 4;

						colors[id] = colors[id] | augment;
					}
				}
		}

		for (var id in colors) {
			if (objs[key]) {
				var material = objs[key].sensors[id];
				if (material)
					material.color.setHex(colors[id]);
			}
		}
	}
}

function redrawCanvas(when) {
	oldDrawTime = new Date().getTime();
	if (!ctx) {
		canvas = document.getElementById("camcanvas");
		ctx = canvas.getContext("2d");
	}
	if (!$(canvas).is(":visible")) {
		return true;
	}
	ctx.clearRect(0, 0, canvas.width, canvas.height);

	var fov_degrees = 160 * fov_scale;
	var fov_radians = fov_degrees / 180 * Math.PI;

	function rad_to_x(ang) {
		var half_fov = fov_radians / 2;
		return ang / half_fov * canvas.width / 2 + canvas.width / 2;
	}
	function rad_to_y(ang) {
		var half_fov = fov_radians / 2;
		return -ang / half_fov * canvas.height / 2 + canvas.height / 2;
	}

	ctx.strokeStyle = "#ffffff";

	for (var x = -fov_degrees; x < fov_degrees; x += 10) {
		var length = 10;
		ctx.beginPath();
		ctx.setLineDash([])
		if (Math.abs(x) === 60) {
			length = canvas.width;
			ctx.setLineDash([ 5, 15 ])
		}
		else if (Math.abs(x) === 0) {
			length = canvas.width;
			ctx.setLineDash([ 1, 15 ])
		}

		ctx.moveTo(rad_to_x(x / 180 * Math.PI), 0);
		ctx.lineTo(rad_to_x(x / 180 * Math.PI), length);

		ctx.moveTo(0, rad_to_x(x / 180 * Math.PI));
		ctx.lineTo(length, rad_to_x(x / 180 * Math.PI));

		ctx.moveTo(rad_to_x(x / 180 * Math.PI), canvas.width);
		ctx.lineTo(rad_to_x(x / 180 * Math.PI), canvas.width - length);

		ctx.moveTo(canvas.width, rad_to_x(x / 180 * Math.PI));
		ctx.lineTo(canvas.width - length, rad_to_x(x / 180 * Math.PI));
		ctx.stroke();
	}

	for (var key in angles) {
		for (var lh = 0; lh < 16; lh++) {
			var bvalue = 0; // get_bvalue(key);

			if (angles[key][lh])

				for (var id in angles[key][lh]) {
					var ang = angles[key][lh][id];

					const x_valid =
						ang[0] !== undefined && survive_timecode_difference(when[key], ang[0][1]) <= visible_tolerance;
					const y_valid =
						ang[1] !== undefined && survive_timecode_difference(when[key], ang[1][1]) <= visible_tolerance;

					var half_fov = 1.0472 * 2.;

					var x = ang[0] !== undefined ? rad_to_x(ang[0][0]) : 0;
					var y = ang[1] !== undefined ? rad_to_y(ang[1][0]) : 0;

					if (!x_valid && !y_valid) {
						continue;
					}
					if (!x_valid) {
						x = lh % 2 ? 10 : canvas.width - 10;
					}
					if (!y_valid) {
						y = lh % 2 ? 10 : canvas.height - 10;
					}

					var size = (x_valid && y_valid) ? (ang[0][2] + ang[1][2]) / 2. : 5e-6;
					var radius = Math.max(1, size > 0 ? Math.sqrt(size * 2 / 2e-6) : 2);
					ctx.fillStyle = "white";
					ctx.font = "14px Arial";
					// ctx.fillText(id, x, y);

					ctx.strokeStyle = "#000000"
					ctx.beginPath();
					ctx.arc(x, y, radius, 0, 2 * Math.PI);
					ctx.stroke();

					ctx.fillStyle = "#" + lhColors[lh].toString(16);
					ctx.beginPath();
					ctx.arc(x, y, radius - 1, 0, 2 * Math.PI);
					ctx.fill();

					const rp = get_rp_angles(key, lh, id)
					const rpx = rp[0], rpy = rp[1];
					if (isFinite(rpx + rpy)) {
						scale = 10;
						var rx = rad_to_x(rpx * scale + ang[0][0])
						var ry = rad_to_y(rpy * scale + ang[1][0])

						dx = x_valid ? rx : x;
						dy = y_valid ? ry : y;

						ctx.fillStyle = "white";
						ctx.font = "14px Arial";

						ctx.strokeStyle = "#" + lhColors[lh].toString(16);
						ctx.beginPath();
						ctx.moveTo(x, y);
						ctx.lineTo(dx, dy);
						ctx.stroke();
					}
				}
		}
	}
}
function solve_vive_pose(vpose) {
	if(vpose.plus_x === undefined) {
		return [ 0, 0, 0, 1, 0, 0, 0 ];
	}

	var plus_x = new THREE.Vector3().fromArray(vpose.plus_x);
	var plus_z = new THREE.Vector3().fromArray(vpose.plus_z);
	var plus_y = new THREE.Vector3().crossVectors(plus_z, plus_x);

	var m = new THREE.Matrix4().makeBasis(plus_x, plus_y, plus_z);

	var Rot = new THREE.Quaternion().setFromRotationMatrix(m);

	var rtn = [ 0, 0, 0, 0, vpose.position[0], vpose.position[1], vpose.position[2] ];
	return Rot.toArray(rtn);
}
function invertPose(pa, quat) {
	var q = new THREE.Quaternion();
	q.fromArray(quat);

	var p = new THREE.Vector3();
	p.fromArray(pa);

	var iq = q.inverse();
	p.applyQuaternion(iq);
	p.multiplyScalar(-1);

	return [ p, iq ];
}
function removeFromParent(o) { o.parent.remove(o); }
function delete_tracked_object(tracker) {
	removeFromParent(objs[tracker].group);
	removeFromParent(objs[tracker].group_rot);
	delete objs[tracker]
}

let ellipsoids = {};
function create_tracked_object(info, external) {
	var sensorGeometry = new THREE.SphereGeometry(.01, 32, 16);
	var group = new THREE.Group();
	var group_rot = new THREE.Group();
	group.sensors = [];
	var axesLength = 1.;

	group.add(group_rot);
	if (info.config && info.config.lighthouse_config) {
		var trackref = new THREE.Group();
		var sensorref = new THREE.Group();
		var imuref = new THREE.Group();

		var trackref_from_head = info.config.trackref_from_head;
		var trackref_from_imu = info.config.trackref_from_imu;

		if (!trackref_from_head && info.config.head)
			trackref_from_head = solve_vive_pose(info.config.head);

		if (!trackref_from_imu && info.config.imu)
			trackref_from_imu = solve_vive_pose(info.config.imu);

		if (trackref_from_head && trackref_from_imu) {
			var pa = [ trackref_from_head[4], trackref_from_head[5], trackref_from_head[6] ];
			var qa = [ trackref_from_head[0], trackref_from_head[1], trackref_from_head[2], trackref_from_head[3] ];
			var ipose = invertPose(pa, qa);
			trackref.position.copy(ipose[0]);
			trackref.quaternion.copy(ipose[1]);
			trackref.verticesNeedUpdate = true;

			var pa = [ trackref_from_imu[4], trackref_from_imu[5], trackref_from_imu[6] ];
			var qa = [ trackref_from_imu[0], trackref_from_imu[1], trackref_from_imu[2], trackref_from_imu[3] ];
			var ipose = invertPose(pa, qa);
			imuref.position.copy(ipose[0]);
			imuref.quaternion.copy(ipose[1]);
			imuref.position.fromArray(pa);
			imuref.quaternion.fromArray(qa);
			imuref.verticesNeedUpdate = true;

			var axes = new THREE.AxesHelper(.075);
			imuref.add(axes);
		}

		for (var idx in info.config.lighthouse_config.modelPoints) {
			var p = info.config.lighthouse_config.modelPoints[idx];
			var pn = info.config.lighthouse_config.modelNormals[idx];
			var color = idx / info.config.lighthouse_config.modelPoints * 0xFFFFFF;
			if (idx === 0)
				color = 0x00ff00;
			var sensorMaterial = new THREE.MeshBasicMaterial({color : color});
			var newSensor = new THREE.Mesh(sensorGeometry, sensorMaterial);
			newSensor.position.set(p[0], p[1], p[2]);
			newSensor.tooltip = info.tracker + " " + idx;

			var normalGeom = new THREE.Geometry();
			normalGeom.vertices.push(newSensor.position,
									 new THREE.Vector3(p[0] + pn[0] * .02, p[1] + pn[1] * .02, p[2] + pn[2] * .02));
			var normal =
				new THREE.Line(normalGeom, new THREE.LineBasicMaterial({color : idx == 4 ? 0xFF0000 : 0x00FF00}));
			group.sensors[idx] = sensorMaterial;

			sensorref.add(normal);
			sensorref.add(newSensor);
		}

		group_rot.add(trackref);
		trackref.add(imuref);
		trackref.add(sensorref);
		group.trackref = trackref;
		group.sensorref = sensorref;
		group.imuref = imuref;
	} else {
		axesLength = 1.5;
	}

	{
		var geometry = new THREE.SphereGeometry(1, 16, 16);
		var material = new THREE.MeshBasicMaterial(
			{color : external ? 0x00FF00 : 0xFF0000, opacity : .15, transparent : true, side : THREE.DoubleSide});

		var mesh = new THREE.Mesh(geometry, material);
		mesh.position.set(0, 0, 0);

		mesh.scale.set(1e-4, 1e-4, 1e-4)

		let ellipsoid_name = info.tracker
		if (ellipsoid_name.endsWith("-raw-obs")) {
			ellipsoid_name = ellipsoid_name.replace("-raw-obs", "'")
		}
		mesh.visible = false;
		ellipsoids[ellipsoid_name] = mesh

		group.add(mesh);
	}

	var axes = new THREE.AxesHelper(axesLength);
	group_rot.add(axes);

	objs[info.tracker] = group;
	objs[info.tracker].group = group;
	objs[info.tracker].group_rot = group_rot;
	if (objs[info.tracker].sensorref)
		objs[info.tracker].sensorref.visible = false;

	group.oldPoseTime = 0;
	group.poseCnt = 0;

	objs[info.tracker].group.position.set(NaN);
	var velocityGeom = new THREE.Geometry();
	velocityGeom.vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));
	objs[info.tracker].velocity =
		new THREE.Line(velocityGeom, new THREE.LineBasicMaterial({color : 0xFFFF00, linewidth : 2}));

	var angVelocityGeom = new THREE.Geometry();
	angVelocityGeom.vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));
	objs[info.tracker].angVelocity =
		new THREE.Line(angVelocityGeom, new THREE.LineDashedMaterial({color : 0x00FFFF, scale : .1}));

	var accelGeom = new THREE.Geometry();
	accelGeom.vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));
	objs[info.tracker].accel =
		new THREE.Line(accelGeom, new THREE.LineBasicMaterial({color : 0xFF00FF, linewidth : 3}));

	group.add(objs[info.tracker].velocity);
	group.add(objs[info.tracker].angVelocity);
	group.add(objs[info.tracker].accel);

	if (external) {
		external_group.add(group);
	} else {
		scene.add(group);
	}
}

var displayTrails = false;
var trails = {};
var MAX_LINE_POINTS = 100000;
var trail_colors = [ 0x173f5f, 0xed553b, 0xa8dadc, 0x0, 0xffffff, 0x11FF11, 0x8888ff, 0xff8888 ];
var trail_idx = 0;
function get_trails(obj, external) {
	if (trails[obj.tracker] == null) {
		var geometry = new THREE.Geometry();
		var material = new THREE.LineBasicMaterial({color : trail_colors[trail_idx++ % trail_colors.length]});
		// var material = new THREE.PointsMaterial({color : trail_colors[trail_idx++ % trail_colors.length], size:
		// .001});

		for (i = 0; i < MAX_LINE_POINTS; i++) {
			geometry.vertices.push(new THREE.Vector3(obj.position[0], obj.position[1], obj.position[2]));
		}
		geometry.dynamic = true;

		// trails[obj.tracker] = new THREE.Points(geometry, material);
		trails[obj.tracker] = new THREE.Line(geometry, material);

		if (external)
			external_group.add(trails[obj.tracker]);
		else
			scene.add(trails[obj.tracker]);
	}

	return trails[obj.tracker]
}

function update_trails() {
	var names = Object.keys(trails);
	for (var i = 0; i < names.length; i++) {
		var name = names[i];
		trails[name].visible = this.checked;
	}
}
$(function() { $("#trails").change(update_trails); });

function update_external() {
	var val = $("#external_displace")[0].checked ? 1.5 : 0;
	external_group.position.set(external2world[0], external2world[1] + val, external2world[2]);
	external_group.quaternion.set(external2world[4], external2world[5], external2world[6], external2world[3]);
}
$(function() { $("#external_displace").change(update_external); });

function update_show_model() { showModel = this.checked; }
$(function() { $("#model").change(update_show_model); });

function update_show_imu() { showIMU = this.checked; }
$(function() { $("#imu").change(update_show_imu); });

function update_fpv() { useFPV = this.checked; }
$(function() { $("#fpv").change(update_fpv); });

function set_object_position(obj, name = null) {
	var objr = objs[name ?? obj.tracker];

	objr.group.position.set(obj.position[0], obj.position[1], obj.position[2]);
	objr.group_rot.quaternion.set(obj.quat[1], obj.quat[2], obj.quat[3], obj.quat[0]);
	objr.group.verticesNeedUpdate = true;
	objr.group_rot.verticesNeedUpdate = true;

	if ("HMD" === obj.tracker || "T20" == obj.tracker || "SM0" == obj.tracker) {
		var up = new THREE.Vector3(0, 1, 0);
		var out = new THREE.Vector3(0, 0, -1);

		fpv_camera.up = up.applyQuaternion(objr.group_rot.quaternion);
		var lookAt = out.applyQuaternion(objr.group_rot.quaternion);
		lookAt.add(objr.position);

		fpv_camera.position.set(obj.position[0], obj.position[1], obj.position[2]);
		fpv_camera.lookAt(lookAt);
	}
}

function createMat4(A) {
	let m = new THREE.Matrix4()
	for (var i = 0; i < 3; i++)
	for (var j = 0; j < 3; j++) {
		m.elements[i * 4 + j] = Array.isArray(A[0]) ? A[j][i] : (i == j ? A[i] : 0);
	}
	return m;
}

function update_ellipsoid(name, u, s, v, A) {
	if (ellipsoids[name]) {
		var ellipsoid = ellipsoids[name];
		// console.log(ellipsoids[name])
		const upscale = name.endsWith("'") ? 2 : 100;
		for (var i = 0; i < 3; i++) {
			s[i] = Math.max(Math.sqrt(s[i]), 1e-5) * upscale
		}
		const um = createMat4(u)
		const sm = createMat4(s)
		const vm = createMat4(v)
		vm.transpose()

		// ellipsoid.scale.set(s[0] * upscale, s[1]  * upscale, s[2] * upscale)

		ellipsoid.matrix.copy(um)
		ellipsoid.matrix.multiply(sm)
		ellipsoid.matrix.multiply(vm)
		ellipsoid.visible = true;
		// ellipsoid.rotation.setFromRotationMatrix(m)
		ellipsoid.verticesNeedUpdate = true;
		ellipsoid.matrixAutoUpdate = false;
		ellipsoid.matrixWorldNeedsUpdate = true
		// ellipsoid.geometry.applyMatrix( new THREE.Matrix4().makeScale( s[0], s[1], s[2] ) );
	}
}
var lighthouse_ups = {};
function lhup(v) {
	console.log(v)
	lighthouse_ups[v[2]] = v;
	var group = lighthouses[v[2]];
	if (group) {
		var fv = v.slice(3).map(parseFloat);
		group.imu.vertices[1].set(fv[0] / 128., fv[1] / 128., fv[2] / 128.);
		group.imu.verticesNeedUpdate = true;
	}
}
var covar_canvas = {}, covar_names = {};
function update_fullcov(v) {
	var name = v[1];
	if (covar_canvas[name] == null) {
		const canvas = document.createElement("canvas");
		canvas.className = "myClass";
		canvas.id = "myId";
		canvas.style.cssText =
			"position: absolute;z-index: 10;width: 105px;height: 105px;bottom:50px;image-rendering: pixelated;"
		canvas.style.right = (5 + Object.keys(covar_canvas).length * 110) + "px";
		canvas.width = canvas.height = 21;

		const div = document.createElement("div");
		div.innerText = name;
		document.body.appendChild(div);
		div.style.cssText =
			"position: absolute;z-index: 11;width: 105px;bottom: 50px;image-rendering: pixelated;right: 225px;color: white;";
		div.style.right = (5 + Object.keys(covar_canvas).length * 110) + "px";
		covar_names[name] = div;
		document.body.appendChild(canvas);
		covar_canvas[name] = canvas;
	}

	var fv = v.slice(3).map(parseFloat);
	let fmax = Math.max(...fv);
	const fmin = Math.min(...fv);
	fmax = Math.max(fmax, -fmin);
	const imageData =
		Uint8ClampedArray.from(fv.map(x => [...turbo(Math.min(1, Math.abs(fmax != 0 ? x / fmax : 0))), 255]).flat());
	var canvas = covar_canvas[name];

	var ctx = canvas.getContext("2d");
	ctx.imageSmoothingEnabled = false;
	// ctx.scale(5,5);
	const l = Math.floor(Math.sqrt(fv.length))
	ctx.putImageData(new ImageData(imageData, l, l), 0, 0);
	covar_names[name].innerText = name + " " + fmax;

	const A = [ [ fv[0], fv[1], fv[2] ], [ fv[l], fv[1 + l], fv[2 + l] ], [ fv[2 * l], fv[1 + l * 2], fv[2 + l * 2] ] ];
	const svd_results = SVDJS.SVD(A, true, true)
	update_ellipsoid(name, svd_results.u, svd_results.q, svd_results.v, A)
}

function display_matrix(name, rows, cols, fv) {
	if (covar_canvas[name] == null) {
		const canvas = document.createElement("canvas");
		canvas.className = "myClass";
		canvas.id = "myId";
		canvas.style.cssText =
			"position: absolute;z-index: 10;width: 105px;height: 105px;bottom:50px;image-rendering: pixelated;"
		canvas.style.right = (5 + Object.keys(covar_canvas).length * 110) + "px";
		canvas.width = canvas.height = 21;

		const div = document.createElement("div");
		div.innerText = name;
		document.body.appendChild(div);
		div.style.cssText =
			"position: absolute;z-index: 11;width: 105px;bottom: 50px;image-rendering: pixelated;right: 225px;color: white;";
		div.style.right = (5 + Object.keys(covar_canvas).length * 110) + "px";
		covar_names[name] = div;
		document.body.appendChild(canvas);
		covar_canvas[name] = canvas;
	}

	let fmax = Math.max(...fv);
	const fmin = Math.min(...fv);
	fmax = Math.max(fmax, -fmin);
	const scaledData = fv.map(x => Math.min(1, Math.abs(fmax != 0 ? x / fmax : 0)));
	const imageData =
		Uint8ClampedArray.from(scaledData.map(x => [...turbo(x), 255]).flat());
	var canvas = covar_canvas[name];

	var ctx = canvas.getContext("2d");
	ctx.imageSmoothingEnabled = false;
	// ctx.scale(5,5);
	canvas.width = cols;
	canvas.height = rows;
	ctx.putImageData(new ImageData(imageData, cols, rows), 0, 0);
	covar_names[name].innerText = name + " " + fmax;
}

function data_matrix(v) {
	let name = v[1] + "." + v[3];
	let rows = parseInt(v[4]), cols = parseInt(v[5]);
	display_matrix(name, rows, cols, v.slice(6).map(parseFloat));
}

function update_fullstate(v) {
	var obj = {
		tracker : v[1],
		position : [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
		quat : [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ],
		velocity : [
			parseFloat(v[10]), parseFloat(v[11]), parseFloat(v[12]), parseFloat(v[13]), parseFloat(v[14]),
			parseFloat(v[15])
		],
		accel : [ parseFloat(v[16]), parseFloat(v[17]), parseFloat(v[18]) ],
		accel_scale : parseFloat(v[19]),
		imu_correction : [ parseFloat(v[20]), parseFloat(v[21]), parseFloat(v[22]), parseFloat(v[23]) ],
		acc_bias : [ parseFloat(v[24]), parseFloat(v[25]), parseFloat(v[26]) ],
		gyro_bias : [ parseFloat(v[27]), parseFloat(v[28]), parseFloat(v[29]) ],
	};

	if (objs[obj.tracker] == null || objs[obj.tracker].velocity == null) {
		return;
	}

	objs[obj.tracker].accel.geometry.vertices[1].set(obj.accel[0], obj.accel[1], obj.accel[2]);
	objs[obj.tracker].accel.geometry.verticesNeedUpdate = true;
}

function update_velocity(v) {
	var obj = {
		tracker : v[1],
		position : [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
		euler : [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]) ]
	};

	if (objs[obj.tracker] == null || objs[obj.tracker].velocity == null) {
		return;
	}

	objs[obj.tracker].velocity.geometry.vertices[1].set(obj.position[0], obj.position[1], obj.position[2]);
	objs[obj.tracker].velocity.geometry.verticesNeedUpdate = true;

	objs[obj.tracker].angVelocity.geometry.vertices[1].set(obj.euler[0], obj.euler[1], obj.euler[2]);
	objs[obj.tracker].angVelocity.geometry.verticesNeedUpdate = true;
}

function update_object(v, allow_unsetup, external) {
	allow_unsetup = true;
	var obj = {
		tracker : v[1],
		position : [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
		quat : [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ]
	};
	var time = parseFloat(v[0]);

	var objr = objs[obj.tracker];

	var now = new Date().getTime();

	if (allow_unsetup && objr == null) {
		create_tracked_object({tracker : obj.tracker}, external);
		objr = objs[obj.tracker];
	}

	if (objr) {
		if (objr.oldPoseTime + 5000 < now) {
			objr.oldPoseTime = now;
			console.log(obj.tracker, (objr.poseCnt / 5) + "hz");
			objr.poseCnt = 0;
		}
		objr.poseCnt++;

		set_object_position(obj);

		if (objr.sensorref)
			objr.sensorref.visible = showModel;

		if (objr.oldPose == null) {
			objr.oldPose = [ 0, 0, 0 ];
		}

		var d = 0;
		for (var i = 0; i < 3; i++) {
			d += Math.pow(obj.position[i] - objr.oldPose[i], 2.);
		}

		var trails = get_trails(obj, external);
		if (trails && Math.sqrt(d) > .01) {
			trails.geometry.vertices.push(trails.geometry.vertices.shift()); // shift the array
			trails.geometry.vertices[MAX_LINE_POINTS - 1] =
				new THREE.Vector3(obj.position[0], obj.position[1], obj.position[2]);
			trails.geometry.verticesNeedUpdate = true;
			objr.oldPose = obj.position;
			record_position(obj.tracker, time, obj);
		}
	}
}

var position_history = {};
var max_time = 0;
function record_position(name, time, position) {
	if (isNaN(time))
		return;

	max_time = Math.max(max_time, time);
	$("#time")[0].max = max_time
	if (position_history[name] === undefined)
	position_history[name] = [];
	position_history[name].push({time : time, position : position});
}

$(function() {
	tooltip = $("#tooltip");
	$("#time").on('change', function(event, ui) {
		var time = $("#time").val()

		var names = Object.keys(position_history);
		for (var i = 0; i < names.length; i++) {
			var name = names[i];

			var j = 0;
			for (j = 0; j < position_history[name].length - 1 && position_history[name][j].time < time; j++)
				;

			var obj = position_history[name][j].position;
			set_object_position(obj, name);
		}
	});
});

var scrollPending = false;
function scrollConsoleToTop() {
	if (scrollPending === false) {
		setTimeout(function() {
			var consoleDiv = $("#console");
			consoleDiv[0].scrollTop = consoleDiv[0].scrollHeight;
			scrollPending = false;
		}, 1);
		scrollPending = true;
	}
}
var polys = {};
function add_axis(v) {
	var fv = v.map(parseFloat);

	var name = v[2];

	scene.remove(polys[name]);

	if (fv[3] <= 1e-5)
		return;

	var group = new THREE.Group();
	var helper = new THREE.AxesHelper(fv[3]);
	group.add(helper);
	group.position.set(fv[4], fv[5], fv[6]);
	group.quaternion.fromArray([ fv[8], fv[9], fv[10], fv[7] ]);
	group.tooltip = name;
	polys[name] = group;

	scene.add(group);
}
function add_sphere(v) {
	var fv = v.map(parseFloat);

	var name = v[2];

	scene.remove(polys[name]);

	if (fv[3] <= 1e-5)
		return;

	var geometry = new THREE.SphereGeometry(fv[3], 4, 4);
	var material =
		new THREE.MeshBasicMaterial({color : fv[4], opacity : .5, transparent : true, side : THREE.DoubleSide});

	var mesh = new THREE.Mesh(geometry, material);
	mesh.position.set(fv[5], fv[6], fv[7]);
	mesh.tooltip = name;
	polys[name] = mesh;

	scene.add(mesh);
}

function get_rp_angles(tracker, lighthouse, sensor_id) {
	rp_angles[tracker] = rp_angles[tracker] || {};
	rp_angles[tracker][lighthouse] = rp_angles[tracker][lighthouse] || {};
	rp_angles[tracker][lighthouse][sensor_id] = rp_angles[tracker][lighthouse][sensor_id] || {};
	return rp_angles[tracker][lighthouse][sensor_id]
}

function add_poly(v) {
	var fv = v.map(parseFloat);

	var geometry = new THREE.Geometry();
	var name = v[2];

	scene.remove(polys[name]);

	for (var i = 3; i < v.length; i += 3) {
		geometry.vertices.push(new THREE.Vector3(fv[i], fv[i + 1], fv[i + 2]))
	}

	var material =
		new THREE.MeshBasicMaterial({color : 0x00ff00, opacity : .5, transparent : true, side : THREE.DoubleSide});
	if (geometry.vertices.length === 4) {
		geometry.faces.push(new THREE.Face3(0, 1, 2));
		geometry.faces.push(new THREE.Face3(2, 3, 0));

		var mesh = new THREE.Mesh(geometry, material);
		mesh.tooltip = name;
		polys[name] = mesh;

		scene.add(mesh);
	} else if (geometry.vertices.length === 2) {
		var line = new THREE.Line(geometry, material);
		polys[name] = line;
		scene.add(line);
	}
}
var external2world = [0, 0, 0, 0, 0, 0, 1];

var survive_log_handlers = {
	"EXTERNAL_TO_WORLD": function(v) {
		external2world = v.slice(2).map(parseFloat);
		update_external();
	},
	"LH_POSE" : function(v) {
		var obj = {
			lighthouse : parseInt(v[1]),
			position : [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
			quat : [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ]
		};

		add_lighthouse(obj.lighthouse, obj.position, obj.quat);
	},
	"POLY" : add_poly,
	"SPHERE" : add_sphere,
	"AXIS" : add_axis,
	"POSE" : update_object,
	"VELOCITY" : update_velocity,
	"FULL_STATE" : update_fullstate,
	"DATA_MATRIX" : data_matrix,
	"FULL_COVARIANCE" : update_fullcov,
	"LH_UP" : lhup,
	"EXTERNAL_VELOCITY" : function(v) { update_velocity(v, true, true); },
	"EXTERNAL_POSE" : function(v) { update_object(v, true, true); },
	"DISCONNECT" : function(v) { delete_tracked_object(v[1]); },
	"CONFIG" : function(v, tracker) {
		var configStr = v.slice(3).join(' ');
		var config = JSON.parse(configStr);
		var obj = {config : config, tracker : v[1]};

		create_tracked_object(obj);
	},
	'W' : function() {},
	'C' : function() {},
	'Y' : function() {},
	'L' : function() {},
	'R' : function() {},
	'BUTTON' : function() {},
	'i' : function() {},
	'OPTION' : function(v) {
		var opt = {name : v[2], type : v[3], value : v[4]};

		switch (opt.name) {
		case 'report-in-imu':
			report_in_imu = parseInt(opt.value);
			break;
		}
	},
	'B' : function(v, tracker) {
		// #define SWEEP_ANGLE_PRINTF_ARGS dev, channel, sensor_id, timecode, plane, angle
		var obj = {
			tracker : v[1],
			channel : parseInt(v[3]),
			sensor_id : parseInt(v[4]),
			timecode : parseInt(v[5]),
			plane : parseInt(v[6]),
			angle : parseFloat(v[7])
		};

		if (bsd_map[obj.channel] === undefined) {
			bsd_map[obj.channel] = Object.keys(bsd_map).length;
		}

		var lighthouse = bsd_map[obj.channel];
		angles[obj.tracker] = angles[obj.tracker] || {};
		angles[obj.tracker][lighthouse] = angles[obj.tracker][lighthouse] || {};
		angles[obj.tracker][lighthouse][obj.sensor_id] = angles[obj.tracker][lighthouse][obj.sensor_id] || {};

		var axis = obj.plane;

		angles[obj.tracker][lighthouse][obj.sensor_id][axis] = [ obj.angle, obj.timecode, -1 ];
		timecode[obj.tracker] = obj.timecode;
	},
	'A' : function(v, tracker) {
		var obj = {
			tracker : v[1],
			sensor_id : parseInt(v[3]),
			acode : parseInt(v[4]),
			timecode : parseInt(v[5]),
			length : parseFloat(v[6]),
			angle : parseFloat(v[7]),
			lighthouse : parseInt(v[8])
		};

		angles[obj.tracker] = angles[obj.tracker] || {};
		angles[obj.tracker][obj.lighthouse] = angles[obj.tracker][obj.lighthouse] || {};
		angles[obj.tracker][obj.lighthouse][obj.sensor_id] = angles[obj.tracker][obj.lighthouse][obj.sensor_id] || {};

		angles[obj.tracker][obj.lighthouse][obj.sensor_id][obj.acode & 1] = [ obj.angle, obj.timecode, obj.length ];
		timecode[obj.tracker] = obj.timecode;
	},
	"RA" : function add_reproject_angle(v, tracker) {
		var obj = {
			tracker : v[1],
			sensor_id : parseInt(v[3]),
			axis : parseInt(v[4]),
			angle : parseFloat(v[5]),
			lighthouse : parseInt(v[6]),
		};

		get_rp_angles(obj.tracker, obj.lighthouse, obj.sensor_id)[obj.axis] = obj.angle;
	},
	'LOG' : function(v) {
		var msg = v.slice(3).join(' ');

		var consoleDiv = $("#console");
		consoleDiv.append(msg + "</br>");

		scrollConsoleToTop();

		console.log(msg);
	},
	"I" : function(v, tracker) {
		var obj = {
			mask : parseInt(v[3]),
			timecode : parseInt(v[4]),
			accelgyro : [
				parseFloat(v[5]), parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]),
				parseFloat(v[10])
			],
			tracker : v[1]
		};
		timecode[obj.tracker] = obj.timecode;

		if (objs[obj.tracker]) {
			if (!downAxes[obj.tracker] && objs[obj.tracker]) {
				downAxes[obj.tracker] = new THREE.Geometry();
				downAxes[obj.tracker].vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));

				var line = new THREE.Line(downAxes[obj.tracker], new THREE.LineBasicMaterial({color : 0xffffff}));
				downAxes[obj.tracker].line = line;
				downAxes[obj.tracker].line.visible = false;
				objs[obj.tracker].imuref.add(line);

				downAxes[obj.tracker].gyro_geom = new THREE.Geometry();
				downAxes[obj.tracker].gyro_geom.vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));

				var line =
					new THREE.Line(downAxes[obj.tracker].gyro_geom, new THREE.LineBasicMaterial({color : 0xff00ff}));
				downAxes[obj.tracker].gyro_geom.line = line;
				downAxes[obj.tracker].gyro_geom.line.visible = false;
				objs[obj.tracker].imuref.add(line);
			}

			if (downAxes[obj.tracker].line) {
				downAxes[obj.tracker].gyro_geom.line.visible = downAxes[obj.tracker].line.visible = showIMU;
			}
			if (objs[obj.tracker].position) {
				var q = obj.accelgyro;

				var gyroInWorld = new THREE.Vector3(0, 0, 0);
				gyroInWorld.fromArray(q, 3);
				downAxes[obj.tracker].gyro_geom.vertices[1] = gyroInWorld;
				downAxes[obj.tracker].gyro_geom.verticesNeedUpdate = true;

				var inWorld = new THREE.Vector3(0, 0, 0);
				inWorld.fromArray(q);
				downAxes[obj.tracker].vertices[1] = inWorld;
				downAxes[obj.tracker].verticesNeedUpdate = true;
			}
		}
	}
};

function add_survive_log_handler(name, entry) { survive_log_handlers[name] = entry; }
function process_survive_handlers(msg) {
	var s = msg.replace('\t', ' ').split(' ').filter(function(x) { return x; });
	var handled = false;
	if (survive_log_handlers[s[2]]) {
		survive_log_handlers[s[2]](s);
		handled = true;
	}
	if (survive_log_handlers[s[1]]) {
		survive_log_handlers[s[1]](s);
		handled = true;
	}

	if (!handled) {
		console.log(msg);
	}

	return {};
}

var survive_ws;

// Dial up the websocket
$(function() {
	setTimeout(function() {
		var url = new URL(window.location.href);
		var remote = url.searchParams.get("remote");

		if (remote && remote.length) {
			survive_ws = new WebSocket("ws://" + remote + "/ws");
		} else if (window.location.protocol === "file:") {
			survive_ws = new WebSocket("ws://localhost:8080/ws");
		} else {
			survive_ws = new WebSocket(((window.location.protocol === "https:") ? "wss://" : "ws://") +
									   window.location.host + "/ws");
		}

		survive_ws.onmessage = function(evt) {
			var msg = evt.data;
			process_survive_handlers(msg);
		};
	}, 60); // Hacky, but this gives the server time to restart on CTRL+R
});

// Init and start the render loop
$(function() {
	// initialization
	init();

	// animation loop / game loop
	animate();
})

///////////////
// FUNCTIONS //
///////////////

function init() {
	///////////
	// SCENE //
	///////////
	scene = new THREE.Scene();
	scene.background = new THREE.Color(0x888888);
	var SCREEN_WIDTH = window.innerWidth, SCREEN_HEIGHT = window.innerHeight;
	// camera attributes
	var VIEW_ANGLE = 45, ASPECT = SCREEN_WIDTH / SCREEN_HEIGHT, NEAR = 0.01, FAR = 200;

	// set up camera
	camera = new THREE.PerspectiveCamera(VIEW_ANGLE, ASPECT, NEAR, FAR);
	camera.up = new THREE.Vector3(0, 0, 1);

	fpv_camera = new THREE.PerspectiveCamera(VIEW_ANGLE * 2, ASPECT, .1, FAR);
	scene.add(fpv_camera);

	// add the camera to the scene
	scene.add(camera);
	camera.position.set(5, 2, 5.00);
	camera.lookAt(scene.position);

	window.addEventListener('mousemove', onMouseMove, false);
	// Function called when download progresses
	var onProgress = function(xhr) { console.log((xhr.loaded / xhr.total * 100) + '% loaded'); };

	// Function called when download errors
	var onError = function(error) { console.log('An error happened' + error); };

	for (var z = 0; z < 5; z++) {
		for (var i = -4; i < 5; i++) {
			for (var j = 0; j < 5; j++) {
				var size = .1;
				var geometry = new THREE.BoxGeometry(size, size, size);

				var cube = new THREE.Mesh(geometry, material);
				var x, y, zz = z, color;
				switch (j) {
				case 0:
					x = i;
					y = 5;
					color = 0xff;
					break;
				case 1:
					x = i;
					y = -5;
					color = 0xff00;
					break;
				case 2:
					x = 5;
					y = i;
					color = 0xff0000;
					break;
				case 3:
					x = -5;
					y = i;
					color = 0xffffff;
					break;
				case 4:
					x = 2 * z - 5;
					y = i;
					zz = 5;
					color = 0xffff00;
					break;
				}
				var material = new THREE.MeshStandardMaterial({color : color});
				cube.position.set(x, y, zz);
				scene.add(cube);
			}
		}
	}

	//////////////
	// RENDERER //
	//////////////

	renderer = new THREE.WebGLRenderer({antialias : true});
	renderer.setSize(SCREEN_WIDTH, SCREEN_HEIGHT);

	// attach div element to variable to contain the renderer
	var container = document.getElementById('ThreeJS');

	// attach renderer to the container div
	container.appendChild(renderer.domElement);

	// move mouse and: left   click to rotate,
	//                 middle click to zoom,
	//                 right  click to pan
	var controls = new THREE.OrbitControls(camera, renderer.domElement);

	// create a light
	var light = new THREE.PointLight(0xffffff);
	light.position.set(0, 0, 5);
	scene.add(light);

	var floorMaterial =
		new THREE.MeshBasicMaterial({color : 0x888888, opacity : 1., transparent : true, side : THREE.FrontSide});
	var floorGeometry = new THREE.PlaneGeometry(100, 100);
	floor = new THREE.Mesh(floorGeometry, floorMaterial);
	floor.position.z = -1;

	external_group.position.set(0, 1.5, 0);
	scene.add(external_group);

	// scene.add(floor);

	/**
	 * Will be called when load completes.
	 * The argument will be the loaded texture.
	 */
	var onLoad =
		function(texture) {
		var s = 50;
		var im_w = 4096;
		var im_h = 2048;
		var circ = 2 * Math.PI * s;
		var h = circ * im_h / im_w;
		var skyBoxGeometry = new THREE.CylinderGeometry(s, s, h, 32);
		texture.wrapS = THREE.RepeatWrapping;
		texture.wrapT = THREE.RepeatWrapping;

		var height = 10;
		var width = 10;

		texture.repeat.set(5, 5);

		var skyBoxMaterial = new THREE.MeshBasicMaterial(
			{map : texture, color : 0x888888, side : THREE.BackSide, transparent : true, opacity : .3});
		var skyBox = new THREE.Mesh(skyBoxGeometry, skyBoxMaterial);
		skyBox.rotateX(Math.PI / 2);
		// skyBox.translateY(h/2.-2.);
		scene.add(skyBox);
	}

	var loader = new THREE.TextureLoader();
	loader.load('https://i.imgur.com/zUrlBcp.jpg', // 'https://i.imgur.com/7lmxb0b.jpg',
				onLoad, onProgress, onError);

	var axes = new THREE.AxesHelper(5);
	scene.add(axes);

	update_trails.call($("#trails")[0]);
}

var last_time = null;
function animate(time) {
	requestAnimationFrame(animate);
	recolorTrackers(timecode);
	render();
	redrawCanvas(timecode);
	frames++;
	if (last_time != null)
		fps = fps * .5 + 1000 / (time - last_time + 1);
	// console.log(
	last_time = time;
}

function measure_fps() { $("#fps").text((Math.round(fps)) + " fps"); }
setInterval(measure_fps, 1000);

var showPlanes = false;
var ang = 0;
function render() {
	var renderCamera = useFPV ? fpv_camera : camera;
	// update the picking ray with the camera and mouse position
	raycaster.setFromCamera(mouse, renderCamera);

	renderer.render(scene, renderCamera);
}
