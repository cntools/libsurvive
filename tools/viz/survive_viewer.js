var objs = {};
var visible_tolerance = 1608200 * 2;
var downAxes = {};
var angles = {};
var ctx;
var canvas;
var oldDrawTime = 0;
var timecode = {};
var oldPoseTime = 0, poseCnt = 0;
var oldPose = [ 0, 0, 0 ];
var scene, camera, renderer, floor, fpv_camera;
var fov_scale = 1;

var report_in_imu = false;

var raycaster = new THREE.Raycaster();

var mouse = new THREE.Vector2();

function onMouseMove(event) {
	// calculate mouse position in normalized device coordinates
	// (-1 to +1) for both components

	mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
	mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

	var foundObj = false;
	var intersects = raycaster.intersectObjects(scene.children, true);
	for (var i = 0; i < intersects.length; i++) {
		if (intersects[i].object.tooltip) {
			$("#tooltip").text(intersects[i].object.tooltip);
			$("#tooltip").css({top : event.clientY, left : event.clientX, position : 'absolute', display : 'block'});
			foundObj = true;
			break;
		}
	}

	if (!foundObj) {
		$("#tooltip").css({display : 'none'});
	}
}

$(function() { $("#toggleBtn").click(function() { $("#cam").toggle(); }); });

var lhColors = [
	0xecba82, 0x4f3920, 0x8c7070, 0xf4eded, 0x4e6e5d, 0x3bc14a, 0x251351, 0xc97b84, 0xa85751, 0x251351, 0xc7eae4,
	0xa7e8bd, 0xffd972, 0xaba361, 0x732c2c, 0x773344c
];

var lighthouses = {};
function add_lighthouse(idx, p, q) {
	if (lighthouses[idx]) {
		var group = lighthouses[idx];
		group.position.fromArray(p);
		group.quaternion.fromArray([ q[1], q[2], q[3], q[0] ]);
		group.verticesNeedUpdate = true;
		return;
	}

	var group = new THREE.Group();
	lighthouses[idx] = group;

	var lh = new THREE.AxesHelper(1);

	group.position.fromArray(p);
	group.quaternion.fromArray([ q[1], q[2], q[3], q[0] ]);

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

	lhBoxMaterial = new THREE.MeshBasicMaterial({color : lhColors[idx]});
	var lhBox = new THREE.Mesh(lhBoxGeom, lhBoxMaterial);
	lhBox.tooltip = "Lighthouse " + (idx)
	group.add(lhBox);

	cone.translateZ(-height / 2);
	cone.rotateZ(Math.PI / 4);
	cone.rotateX(Math.PI / 2);

	group.add(cone);
	group.add(lh);

	var material1 =
		new THREE.MeshBasicMaterial({color : 0x00FF00, opacity : .2, transparent : true, side : THREE.DoubleSide});

	var material2 =
		new THREE.MeshBasicMaterial({color : 0xFF0000, opacity : .2, transparent : true, side : THREE.DoubleSide});

	var geometry = new THREE.PlaneGeometry(3, 3);

	var mesh = new THREE.PlaneHelper(null, 3, 0x00FF00);
	group.plane1 = mesh;
	var mesh2 = new THREE.PlaneHelper(null, 3, 0xFF0000);
	group.plane2 = mesh2;

	mesh.visible = false;
	mesh2.visible = false;

	group.add(mesh);
	group.add(mesh2);

	scene.add(group);
}

function run_planes() {}

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
	ctx.beginPath();
	for (var x = -fov_degrees; x < fov_degrees; x += 10) {
		var length = Math.abs(x) == 60 ? canvas.width : 10;
		ctx.moveTo(rad_to_x(x / 180 * Math.PI), 0);
		ctx.lineTo(rad_to_x(x / 180 * Math.PI), length);

		ctx.moveTo(0, rad_to_x(x / 180 * Math.PI));
		ctx.lineTo(length, rad_to_x(x / 180 * Math.PI));

		ctx.moveTo(rad_to_x(x / 180 * Math.PI), canvas.width);
		ctx.lineTo(rad_to_x(x / 180 * Math.PI), canvas.width - length);

		ctx.moveTo(canvas.width, rad_to_x(x / 180 * Math.PI));
		ctx.lineTo(canvas.width - length, rad_to_x(x / 180 * Math.PI));
	}

	ctx.stroke();

	for (var key in angles) {
		for (var lh = 0; lh < 16; lh++) {
			var bvalue = 0; // get_bvalue(key);

			ctx.strokeStyle = "#" + lhColors[lh].toString(16);

			if (angles[key][lh])

				for (var id in angles[key][lh]) {
					var ang = angles[key][lh][id];

					if (ang[0] === undefined || ang[1] === undefined ||
						survive_timecode_difference(when[key], ang[1][1]) > visible_tolerance ||
						survive_timecode_difference(when[key], ang[0][1]) > visible_tolerance)
						continue;

					var half_fov = 1.0472 * 2.;
					var x = rad_to_x(ang[0][0]);
					var y = rad_to_y(ang[1][0]);

					ctx.fillStyle = "white";
					ctx.font = "14px Arial";
					// ctx.fillText(id, x, y);

					ctx.beginPath();
					ctx.arc(x, y, 1, 0, 2 * Math.PI);
					ctx.stroke();
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

function create_tracked_object(info) {
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

	var axes = new THREE.AxesHelper(axesLength);
	group_rot.add(axes);

	objs[info.tracker] = group;
	objs[info.tracker].group = group;
	objs[info.tracker].group_rot = group_rot;
	if (objs[info.tracker].sensorref)
		objs[info.tracker].sensorref.visible = false;

	objs[info.tracker].group.position.set(NaN);
	var velocityGeom = new THREE.Geometry();
	velocityGeom.vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));
	objs[info.tracker].velocity =
		new THREE.Line(velocityGeom, new THREE.LineBasicMaterial({color : 0xFFFF00, linewidth : 2}));

	var angVelocityGeom = new THREE.Geometry();
	angVelocityGeom.vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));
	objs[info.tracker].angVelocity =
		new THREE.Line(angVelocityGeom, new THREE.LineDashedMaterial({color : 0x00FFFF, scale : .1}));

	group.add(objs[info.tracker].velocity);
	group.add(objs[info.tracker].angVelocity);

	scene.add(group);
}

var displayTrails = false;
var trails = {};
var MAX_LINE_POINTS = 100000;
var trail_colors = [ 0x0, 0xffffff, 0x11FF11, 0x8888ff ];
var trail_idx = 0;
function get_trails(obj) {
	if (trails[obj.tracker] == null) {
		var geometry = new THREE.Geometry();
		var material = new THREE.LineBasicMaterial({color : trail_colors[trail_idx++ % trail_colors.length]});

		for (i = 0; i < MAX_LINE_POINTS; i++) {
			geometry.vertices.push(new THREE.Vector3(obj.position[0], obj.position[1], obj.position[2]));
		}
		geometry.dynamic = true;

		trails[obj.tracker] = new THREE.Line(geometry, material);

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
function update_object(v, allow_unsetup) {
	allow_unsetup = true;
	var obj = {
		tracker : v[1],
		position : [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
		quat : [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ]
	};
	var time = parseFloat(v[0]);

	if (allow_unsetup && objs[obj.tracker] == null) {
		create_tracked_object({tracker : obj.tracker});
	}

	if (objs[obj.tracker]) {
		var now = new Date().getTime();
		if (oldPoseTime + 5000 < now) {
			oldPoseTime = now;
			console.log((poseCnt / 5) + "hz");
			poseCnt = 0;
		}
		poseCnt++;

		objs[obj.tracker].group.position.set(obj.position[0], obj.position[1], obj.position[2]);
		objs[obj.tracker].group_rot.quaternion.set(obj.quat[1], obj.quat[2], obj.quat[3], obj.quat[0]);
		objs[obj.tracker].group.verticesNeedUpdate = true;
		objs[obj.tracker].group_rot.verticesNeedUpdate = true;

		if (objs[obj.tracker].sensorref)
			objs[obj.tracker].sensorref.visible = $("#model")[0].checked;

		if (objs[obj.tracker].oldPose == null) {
			objs[obj.tracker].oldPose = [ 0, 0, 0 ];
		}

		var d = 0;
		for (var i = 0; i < 3; i++) {
			d += Math.pow(obj.position[i] - objs[obj.tracker].oldPose[i], 2.);
		}

		var trails = get_trails(obj);
		if (trails && Math.sqrt(d) > .01) {
			trails.geometry.vertices.push(trails.geometry.vertices.shift()); // shift the array
			trails.geometry.vertices[MAX_LINE_POINTS - 1] =
				new THREE.Vector3(obj.position[0], obj.position[1], obj.position[2]);
			trails.geometry.verticesNeedUpdate = true;
			objs[obj.tracker].oldPose = obj.position;
			record_position(obj.tracker, time, obj);
		}

		if ("HMD" === obj.tracker || "T20" == obj.tracker) {
			var up = new THREE.Vector3(0, 1, 0);
			var out = new THREE.Vector3(0, 0, -1);

			fpv_camera.up = up.applyQuaternion(objs[obj.tracker].group_rot.quaternion);
			var lookAt = out.applyQuaternion(objs[obj.tracker].group_rot.quaternion);
			lookAt.add(objs[obj.tracker].position);

			fpv_camera.position.set(obj.position[0], obj.position[1], obj.position[2]);
			fpv_camera.lookAt(lookAt);
		}
	}
}

var position_history = {};
var max_time = 0;
function record_position(name, time, position) {
	max_time = Math.max(max_time, time);
	if (position_history[name] == undefined)
		position_history[name] = [];
	position_history[name].push({time : time, position : position});
}

$(function() {
	$("#time").on('change mousemove', function(event, ui) {
		var time = $("#time").val() * max_time / 100.;

		var names = Object.keys(position_history);
		for (var i = 0; i < names.length; i++) {
			var name = names[i];

			var j = 0;
			for (j = 0; j < position_history[name].length - 1 && position_history[name][j].time < time; j++)
				;

			var obj = position_history[name][j].position;
			objs[name].position.set(obj.position[0], obj.position[1], obj.position[2]);
			objs[name].quaternion.set(obj.quat[1], obj.quat[2], obj.quat[3], obj.quat[0]);
			objs[name].verticesNeedUpdate = true;
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

var survive_log_handlers = {
	"LH_POSE" : function(v) {
		var obj = {
			lighthouse : parseInt(v[1]),
			position : [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
			quat : [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ]
		};

		add_lighthouse(obj.lighthouse, obj.position, obj.quat);
	},
	"POSE" : update_object,
	"VELOCITY" : update_velocity,
	"EXTERNAL_VELOCITY" : function(v) { update_velocity(v, true); },
	"EXTERNAL_POSE" : function(v) { update_object(v, true); },
	"CONFIG" : function(v, tracker) {
		var configStr = v.slice(3).join(' ');
		var config = JSON.parse(configStr);
		var obj = {config : config, tracker : v[1]};

		create_tracked_object(obj);
	},
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
			lighthouse : parseInt(v[3]),
			sensor_id : parseInt(v[4]),
			timecode : parseInt(v[5]),
			plane : parseInt(v[6]),
			angle : parseFloat(v[7])
		};

		angles[obj.tracker] = angles[obj.tracker] || {};
		angles[obj.tracker][obj.lighthouse] = angles[obj.tracker][obj.lighthouse] || {};
		angles[obj.tracker][obj.lighthouse][obj.sensor_id] = angles[obj.tracker][obj.lighthouse][obj.sensor_id] || {};

		var axis = obj.plane;

		angles[obj.tracker][obj.lighthouse][obj.sensor_id][axis] = [ obj.angle, obj.timecode ];
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

		angles[obj.tracker][obj.lighthouse][obj.sensor_id][obj.acode & 1] = [ obj.angle, obj.timecode ];
		timecode[obj.tracker] = obj.timecode;
	},
	'LOG' : function(v) {
		var msg = v.slice(3).join(' ');

		var consoleDiv = $("#console");
		consoleDiv.append(msg + "</br>");

		scrollConsoleToTop();
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
				downAxes[obj.tracker].gyro_geom.line.visible = downAxes[obj.tracker].line.visible =
					$("#imu")[0].checked;
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
	var s = msg.split(' ').filter(function(x) { return x; });

	if (survive_log_handlers[s[2]]) {
		survive_log_handlers[s[2]](s);
	}
	if (survive_log_handlers[s[1]]) {
		survive_log_handlers[s[1]](s);
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

	fpv_camera = new THREE.PerspectiveCamera(VIEW_ANGLE, ASPECT, .1, FAR);
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

function animate() {
	requestAnimationFrame(animate);
	recolorTrackers(timecode);
	render();
	redrawCanvas(timecode);
}
var showPlanes = false;
var ang = 0;
function render() {
	Object.keys(lighthouses).forEach(function(idx) {
		var lh = lighthouses[idx];

		{
			var a = ang - Math.PI * 2 / 3;
			var plane = new THREE.Plane(new THREE.Vector3(Math.cos(a), 0.57735026919, -Math.sin(a)));
			// lh.plane.translate(plane.coplanarPoint());
			// lh.plane1.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), plane.normal);
			lh.plane1.plane = plane;

			lh.plane1.visible = showPlanes && (ang > Math.PI * 2 / 3 - 1.5 && ang < Math.PI * 2 / 3 + 1.5);
		}

		{
			var a = ang - Math.PI * 4 / 3;
			var plane = new THREE.Plane(new THREE.Vector3(Math.cos(a), -0.57735026919, -Math.sin(a)));
			// lh.plane.translate(plane.coplanarPoint());
			// lh.plane2.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), plane.normal);
			lh.plane2.plane = plane;

			lh.plane2.visible = showPlanes && (ang > Math.PI * 4 / 3 - 1.5 && ang < Math.PI * 4 / 3 + 1.5);
		}
	});
	ang += 0.01;
	ang = ang % (2 * Math.PI);

	var use_fpv = $("#fpv").length > 0 && $("#fpv")[0].checked;
	var renderCamera = use_fpv ? fpv_camera : camera;
	// update the picking ray with the camera and mouse position
	raycaster.setFromCamera(mouse, renderCamera);

	renderer.render(scene, renderCamera);
}
