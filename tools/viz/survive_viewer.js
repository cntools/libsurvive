var sphere, axes;

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
var downAxes = {};
var angles = {};
var ctx;
var canvas;
var oldDrawTime = 0;
var lastWhen = {};

$(function() { $("#toggleBtn").click(function() { $("#cam").toggle(); }); });

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

					if (ang[0] === undefined || ang[1] === undefined || ang[1][1] < when[key] - 48000000 ||
						ang[0][1] < when[key] - 48000000)
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

var objs = {};
var sensorGeometry = new THREE.SphereGeometry(.01, 32, 16);
// use a "lambert" material rather than "basic" for realistic lighting.
//   (don't forget to add (at least one) light!)

function create_object(info) {
	var group = new THREE.Group();
	group.colors = [];
	if (info.config && info.config.lighthouse_config) {
		for (var idx in info.config.lighthouse_config.modelPoints) {
			var p = info.config.lighthouse_config.modelPoints[idx];
			var color = 0xFFFFFF; // / info.points.length * idx;
			if (idx === 10)
				color = 0x00ff00;
			if (idx === 12)
				color = 0x0000ff;
			var sensorMaterial = new THREE.MeshLambertMaterial({color : color});
			var newSensor = new THREE.Mesh(sensorGeometry, sensorMaterial);
			newSensor.position.set(p[0], p[1], p[2]);

			group.colors[idx] = sensorMaterial;
			group.add(newSensor);
		}
		}
	var axes = new THREE.AxesHelper(1);
	group.add(axes);

	objs[info.tracker] = group;
	scene.add(group);
	}

var timecode = {};
$(function() {

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
					type: "pose", tracker: v[1], position: [ parseFloat(v[3]), parseFloat(v[4]), parseFloat(v[5]) ],
						quat: [ parseFloat(v[6]), parseFloat(v[7]), parseFloat(v[8]), parseFloat(v[9]) ]
				}
			},
			"CONFIG" : function(v) {
				var configStr = s.slice(3).join(' ');
				var config = JSON.parse(configStr);

				return { type: "htc_config", config: config }

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
		} else if (obj.type === "lighthouse_pose") {
			add_lighthouse(obj.lighthouse, obj.position, obj.quat);
		} else if (obj.type === "htc_config") {
			create_object(obj);
		} else if (obj.type === "imu") {
			if (objs[obj.tracker]) {
				if (!downAxes[obj.tracker]) {
					downAxes[obj.tracker] = new THREE.Geometry();
					downAxes[obj.tracker].vertices.push(
						new THREE.Vector3(0, 0, 0),
						new THREE.Vector3(obj.accelgyro[0], obj.accelgyro[1], obj.accelgyro[2]));

					var line = new THREE.Line(downAxes[obj.tracker], new THREE.LineBasicMaterial({color : 0xffffff}));
					objs[obj.tracker].add(line);
				} else {
					var q = obj.accelgyro;
					downAxes[obj.tracker].vertices[1].fromArray(q);
					downAxes[obj.tracker].verticesNeedUpdate = true;
				}
			}

		} else if (obj.type === "angle") {
			angles[obj.tracker] = angles[obj.tracker] || {};
			angles[obj.tracker][obj.lighthouse] = angles[obj.tracker][obj.lighthouse] || {};
			angles[obj.tracker][obj.lighthouse][obj.sensor_id] =
				angles[obj.tracker][obj.lighthouse][obj.sensor_id] || {};

			angles[obj.tracker][obj.lighthouse][obj.sensor_id][obj.acode] = [ obj.angle, obj.timecode ];
			timecode[obj.tracker] = obj.timecode;
		}

		// ws.send("!");
	};
});

//////////
// MAIN //
//////////

// standard global variables
var container, scene, camera, renderer, controls, stats;
var clock = new THREE.Clock();

// custom global variables
var cube;
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

	////////////
	// CAMERA //
	////////////

	// set the view size in pixels (custom or according to window size)
	// var SCREEN_WIDTH = 400, SCREEN_HEIGHT = 300;
	var SCREEN_WIDTH = window.innerWidth, SCREEN_HEIGHT = window.innerHeight;
	// camera attributes
	var VIEW_ANGLE = 45, ASPECT = SCREEN_WIDTH / SCREEN_HEIGHT, NEAR = 0.01, FAR = 200;
	// set up camera
	camera = new THREE.PerspectiveCamera(VIEW_ANGLE, ASPECT, NEAR, FAR);
	camera.up = new THREE.Vector3(0, 0, 1);
	// add the camera to the scene
	scene.add(camera);
	// the camera defaults to position (0,0,0)
	// 	so pull it back (z = 400) and up (y = 100) and set the angle towards the
	// scene origin
	camera.position.set(5, 2, 5.00);
	camera.lookAt(scene.position);

	//////////////
	// RENDERER //
	//////////////

	renderer = new THREE.WebGLRenderer({antialias : true});

	renderer.setSize(SCREEN_WIDTH, SCREEN_HEIGHT);

	// attach div element to variable to contain the renderer
	container = document.getElementById('ThreeJS');
	// alternatively: to create the div at runtime, use:
	//   container = document.createElement( 'div' );
	//    document.body.appendChild( container );

	// attach renderer to the container div
	container.appendChild(renderer.domElement);

	////////////
	// EVENTS //
	////////////

	/*
	// automatically resize renderer
	THREEx.WindowResize(renderer, camera);
	// toggle full-screen on given key press
	THREEx.FullScreen.bindKey({ charCode : 'm'.charCodeAt(0) });
*/
	//////////////
	// CONTROLS //
	//////////////

	// move mouse and: left   click to rotate,
	//                 middle click to zoom,
	//                 right  click to pan
	controls = new THREE.OrbitControls(camera, renderer.domElement);

	///////////
	// LIGHT //
	///////////

	// create a light
	var light = new THREE.PointLight(0xffffff);
	light.position.set(0, 5, 0);
	scene.add(light);
	var ambientLight = new THREE.AmbientLight(0x111111);
	// scene.add(ambientLight);

	var floorTexture = new THREE.ImageUtils.loadTexture('images/checkerboard.jpg');
	floorTexture.wrapS = floorTexture.wrapT = THREE.RepeatWrapping;
	floorTexture.repeat.set(10, 10);
	// DoubleSide: render texture on both sides of mesh
	var floorMaterial =
		new THREE.MeshBasicMaterial({color : 0x000000, opacity : 0.15, transparent : true, side : THREE.FrontSide});
	var floorGeometry = new THREE.PlaneGeometry(10, 10);
	var floor = new THREE.Mesh(floorGeometry, floorMaterial);
	floor.position.z = -1;

	scene.add(floor);

	/////////
	// SKY //
	/////////

	// recommend either a skybox or fog effect (can't use both at the same time)
	// without one of these, the scene's background color is determined by
	// webpage background

	var skyBoxGeometry = new THREE.CubeGeometry(50, 50, 50);
	var skyBoxMaterial = new THREE.MeshBasicMaterial({color : 0x888888, side : THREE.BackSide});
	var skyBox = new THREE.Mesh(skyBoxGeometry, skyBoxMaterial);
	scene.add(skyBox);

	// fog must be added to scene before first render
	// scene.fog = new THREE.FogExp2(0xffffff, 0.025);
}

function animate() {
	requestAnimationFrame(animate);
	render();
	update();
	redrawCanvas(timecode);
	}

function update() {
	// delta = change in time since last call (in seconds)
	var delta = clock.getDelta();

	// controls.update();
	}

function render() { renderer.render(scene, camera); }
