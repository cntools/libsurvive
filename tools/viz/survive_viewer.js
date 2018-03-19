var objs = {};
var visible_tolerance = 48000000 / 15;
var downAxes = {};
var angles = {};
var ctx;
var canvas;
var oldDrawTime = 0;

$(function() { $("#toggleBtn").click(function() { $("#cam").toggle(); }); });

function add_lighthouse(idx, p, q) {
	var group = new THREE.Group();

	var lh = new THREE.AxesHelper(1);

	group.position.fromArray(p);
	group.quaternion.fromArray([ q[1], q[2], q[3], q[0] ]);

	var height = 3;
	var geometry = new THREE.ConeGeometry(Math.sin(1.0472) * height, height, 4, 1, true);
	var material = new THREE.MeshBasicMaterial({
		wireframe : true,
		vertexColor : true,
		color : 0x111111,
		opacity : 0.09,
		transparent : true,
		blending : THREE.AdditiveBlending,
		side : THREE.BothSides
	});
	var cone = new THREE.Mesh(geometry, material);

	var lhBoxGeom = new THREE.CubeGeometry(.1, .1, .1);
	var lhBoxMaterial = new THREE.MeshLambertMaterial({color : 0x111111, side : THREE.FrontSide});
	var lhBox = new THREE.Mesh(lhBoxGeom, lhBoxMaterial);
	group.add(lhBox);

	cone.translateZ(-height / 2)
	cone.rotateZ(Math.PI / 4)
	cone.rotateX(Math.PI / 2)
	// cone.position.z

	group.add(cone);

	group.add(lh);
	scene.add(group);
	// DrawCoordinateSystem(p[0], p[1], p[2], q[0], q[1], q[2], q[3]);
	}

function recolorTrackers(when) {
	for (var key in angles) {
		var colors = [];

		for (var lh = 0; lh < 2; lh++) {
			var bvalue = {"WW0" : "FF", "TR0" : "00"};
			ctx.strokeStyle = (lh === 0 ? "#FF00" : "#00FF") + bvalue[key];

			if (angles[key][lh])

				for (var id in angles[key][lh]) {
					var ang = angles[key][lh][id];

					colors[id] = colors[id] || 0;

					if (ang[0] === undefined || ang[1] === undefined)
						continue;

					for (var acode = 0; acode < 2; acode++) {
						if (ang[acode][1] < when[key] - visible_tolerance)
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

	var fov_degrees = 150;
	var fov_radians = fov_degrees / 180 * Math.PI;

	function rad_to_x(ang) {
		var half_fov = fov_radians / 2;
		return ang / half_fov * canvas.width / 2 + canvas.width / 2;
		}
	var rad_to_y = rad_to_x;

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
		for (var lh = 0; lh < 2; lh++) {
			var bvalue = {"WW0" : "FF", "TR0" : "00"};
			ctx.strokeStyle = (lh === 0 ? "#FF00" : "#00FF") + bvalue[key];

			if (angles[key][lh])

				for (var id in angles[key][lh]) {
					var ang = angles[key][lh][id];

					if (ang[0] === undefined || ang[1] === undefined || ang[1][1] < when[key] - visible_tolerance ||
						ang[0][1] < when[key] - visible_tolerance)
						continue;

					var half_fov = 1.0472 * 2;
					var x = ang[0][0] / half_fov * canvas.width / 2 + canvas.width / 2;
					var y = -ang[1][0] / half_fov * canvas.height / 2 + canvas.height / 2;

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

function create_object(info) {
	var sensorGeometry = new THREE.SphereGeometry(.01, 32, 16);
	var group = new THREE.Group();
	group.sensors = [];
	if (info.config && info.config.lighthouse_config) {
		for (var idx in info.config.lighthouse_config.modelPoints) {
			var p = info.config.lighthouse_config.modelPoints[idx];
			var color = 0xFFFFFF; // / info.points.length * idx;
			if (idx === 10)
				color = 0x00ff00;
			if (idx === 12)
				color = 0x0000ff;
			var sensorMaterial = new THREE.MeshBasicMaterial({color : color});
			var newSensor = new THREE.Mesh(sensorGeometry, sensorMaterial);
			newSensor.position.set(p[0], p[1], p[2]);

			group.sensors[idx] = sensorMaterial;
			group.add(newSensor);
		}
		}
	var axes = new THREE.AxesHelper(1);
	group.add(axes);

	objs[info.tracker] = group;
	scene.add(group);
	}

var timecode = {};

function parseLine(msg) {
	var s = msg.split(' ');

	var command_mappings = {
		"LH_POSE" : function(v) {
			return {
				type : "lighthouse_pose",
				lighthouse : parseInt(v[1]),
				position : [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
				quat : [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ]
			};
		},
		"POSE" : function(v) {
			return {
				type: "pose", position: [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
					quat: [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ]
			}
		},
		"CONFIG" : function(v) {
			var configStr = s.slice(3).join(' ');
			var config = JSON.parse(configStr);

			return { type: "htc_config", config: config }

		},
		'A' : function(v) {
			return {
				type: 'angle', sensor_id: parseInt(v[3]), acode: parseInt(v[4]), timecode: parseInt(v[5]),
					length: parseFloat(v[6]), angle: parseFloat(v[7]), lighthouse: parseInt(v[8])
			}
		},
		'LOG' : function(v) {
			return { type: "info", msg: s.slice(3).join(' ') }
		},
		"I" : function(v) {
			return {
				type : "imu",
				mask : parseInt(v[3]),
				timecode : parseInt(v[4]),
				accelgyro : [
					parseFloat(v[5]), parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]),
					parseFloat(v[10])
				]

			};
		}
	};
	if (command_mappings[s[2]]) {
		var rtn = command_mappings[s[2]](s);
		rtn.time = parseFloat(s[0]);
		rtn.tracker = s[1];
		return rtn;
		}
	return {};
}

$(function() {
	setTimeout(function() {
		var ws;

		if (window.location.protocol === "file:") {
			ws = new WebSocket("ws://localhost:8080/ws");
		} else {
			ws = new WebSocket(((window.location.protocol === "https:") ? "wss://" : "ws://") + window.location.host +
							   "/ws");
		}

		ws.onopen = function(evt) {
			// ws.send("!");
		};
		ws.onmessage = function(evt) {
			var msg = evt.data;
			var obj;
			if (msg[0] == "{")
				obj = JSON.parse(msg);
			else
				obj = parseLine(msg);

			// console.log(obj);
			if (obj.type === "pose") {
				if (objs[obj.tracker]) {
					objs[obj.tracker].position.set(obj.position[0], obj.position[1], obj.position[2]);
					objs[obj.tracker].quaternion.set(obj.quat[1], obj.quat[2], obj.quat[3], obj.quat[0]);
				}
			} else if (obj.type === "info") {
				var consoleDiv = $("#console");
				consoleDiv.append(obj.msg + "</br>");
				consoleDiv[0].scrollTop = consoleDiv[0].scrollHeight;
			} else if (obj.type === "lighthouse_pose") {
				add_lighthouse(obj.lighthouse, obj.position, obj.quat);
			} else if (obj.type === "htc_config") {
				create_object(obj);
			} else if (obj.type === "imu") {
				if (objs[obj.tracker]) {
					if (!downAxes[obj.tracker] && objs[obj.tracker]) {
						downAxes[obj.tracker] = new THREE.Geometry();
						downAxes[obj.tracker].vertices.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0));

						var line =
							new THREE.Line(downAxes[obj.tracker], new THREE.LineBasicMaterial({color : 0xffffff}));
						scene.add(line);
						}

					if (objs[obj.tracker].position) {
						var q = obj.accelgyro;

						downAxes[obj.tracker].vertices[0] = objs[obj.tracker].position;
						downAxes[obj.tracker].vertices[1].fromArray(q);
						downAxes[obj.tracker].vertices[1].add(objs[obj.tracker].position);
						downAxes[obj.tracker].verticesNeedUpdate = true;
					}
				}

			} else if (obj.type === "angle") {
				angles[obj.tracker] = angles[obj.tracker] || {};
				angles[obj.tracker][obj.lighthouse] = angles[obj.tracker][obj.lighthouse] || {};
				angles[obj.tracker][obj.lighthouse][obj.sensor_id] =
					angles[obj.tracker][obj.lighthouse][obj.sensor_id] || {};

				angles[obj.tracker][obj.lighthouse][obj.sensor_id][obj.acode & 1] = [ obj.angle, obj.timecode ];
				timecode[obj.tracker] = obj.timecode;
			}

			// ws.send("!");
		};
	}, 60); // Hacky, but this gives the server time to restart on CTRL+R
});

// standard global variables
var container, scene, camera, renderer, controls;

// custom global variables
$(function() {
	// initialization
	init();

	// animation loop / game loop
	animate();
})

///////////////
// FUNCTIONS //
///////////////

function
init() {
	///////////
	// SCENE //
	///////////
	scene = new THREE.Scene();

	var SCREEN_WIDTH = window.innerWidth, SCREEN_HEIGHT = window.innerHeight;
	// camera attributes
	var VIEW_ANGLE = 45, ASPECT = SCREEN_WIDTH / SCREEN_HEIGHT, NEAR = 0.01, FAR = 200;

	// set up camera
	camera = new THREE.PerspectiveCamera(VIEW_ANGLE, ASPECT, NEAR, FAR);
	camera.up = new THREE.Vector3(0, 0, 1);

	// add the camera to the scene
	scene.add(camera);
	camera.position.set(5, 2, 5.00);
	camera.lookAt(scene.position);

	//////////////
	// RENDERER //
	//////////////

	renderer = new THREE.WebGLRenderer({antialias : true});
	renderer.setSize(SCREEN_WIDTH, SCREEN_HEIGHT);

	// attach div element to variable to contain the renderer
	container = document.getElementById('ThreeJS');

	// attach renderer to the container div
	container.appendChild(renderer.domElement);

	// move mouse and: left   click to rotate,
	//                 middle click to zoom,
	//                 right  click to pan
	controls = new THREE.OrbitControls(camera, renderer.domElement);

	// create a light
	var light = new THREE.PointLight(0xffffff);
	light.position.set(0, 5, 0);
	scene.add(light);

	var floorMaterial =
		new THREE.MeshBasicMaterial({color : 0x000000, opacity : 0.15, transparent : true, side : THREE.FrontSide});
	var floorGeometry = new THREE.PlaneGeometry(10, 10);
	var floor = new THREE.Mesh(floorGeometry, floorMaterial);
	floor.position.z = -1;

	scene.add(floor);

	var skyBoxGeometry = new THREE.CubeGeometry(50, 50, 50);
	var skyBoxMaterial = new THREE.MeshBasicMaterial({color : 0x888888, side : THREE.BackSide});
	var skyBox = new THREE.Mesh(skyBoxGeometry, skyBoxMaterial);
	scene.add(skyBox);

	var axes = new THREE.AxesHelper(5);
	scene.add(axes);
}

function animate() {
	requestAnimationFrame(animate);
	recolorTrackers(timecode);
	render();
	redrawCanvas(timecode);
	}

function render() { renderer.render(scene, camera); }
