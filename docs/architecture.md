# General Structure

At the lowest level, libsurvive has a number of driver plugins which provide the raw light, config and IMU data into 
the system. The default and most useful driver is the `Vive` driver which implements the USB protocol that is exposed in 
all Vive and commercial lighthouse products. 

Depending on which generation of lighthouse system is involved, different logistical steps turn the raw data into 
disambiguated angle data which relates a given lighthouse to a sensor on a device. This also includes the collection of OOTX
data from the lighthouses themselves which contain configuration and calibration data for the lighthouses. It also 
involves filtering of stray light that is either wildly divergent from prior data to that lighthouse/sensor/axis tuple as well
as reflection filtering by removing outliers according to the standard deviation for all observed sensor data for a given
lighthouse/axis. 

The filtered data is then passed into an extended kalman filter as well as a plugin-based 'poser'. On initial startup, 
lighthouse positions are not known and the poser is responsible for solving for a scene and providing the initial 
lighthouse positions. After that point, the poser uses the lighthouse positions and provides its poses into the kalman
filter as a measurement. 

Once the kalman filters covariance matrix has a low enough trace, the internal state of the position and velocity for a 
given object are emitted into user-registerable callbacks. 

# Vive Driver

The vive driver is the main useful driver of the system and is what lets it interoperate with all consumer SteamVR 
devices. Other drivers are pluggable for testing and proprietary systems. 

The vive driver is heavily built around libusb and uses it as an abstraction layer to linux, windows and android systems.
It is responsible for polling the individual devices and deciphering the protocols for each. It also performs feature 
requests to retrieve the JSON-encoded configuration data which provides the metadata per device which is required for 
tracking. 

While early in the HTC vive ecosystem there were a variety of tracker chipsets used in tracked objects, most devices now
are a consistent vendor / product ID and protocol, and so new objects should be trackable without explicit driver code 
for each device type. 

All controller / input events are also captured and exposed into user-definable event handlers. This includes digital
button presses, as well as analog joystick / touchpad input and things like battery state and finger proximity. 

# Math infrastructure

Almost no portion of the system is describable in linear terms. The reprojection model for both generations of 
lighthouse systems relies heavily on trigonomic functions, especially when including the full lighthouse calibration 
model. This means that most of the math is reliant on non-linear least squares methods to provide system estimates.

All the non-linear least squares methods used benefit greatly from having analytical jacobian available to provide 
gradient information to the various algorithms. To avoid the bugs and time sink involved in calculating these by hand, 
there is a robust set of functions and tools under `tools/generate_math_functions` which uses symengine to generate C 
code from both python implementations of the necessary function as well as their jacobians over whatever input parameters
are required. 

Additionaly there is sanity-check code which compares those results to numerically calculated jacobians to makes sure 
the results are valid. 

# Iterative Extended Kalman Filter

The kalman filter is a largely standard extended kalman filter implementation. It models the problem in a largely 
configurable way depending on the desired CPU / precision tradeoffs but the default configuration is to represent the
state space as the following:

- Pose (Vec3 + Quaternion)
- Velocity Estimate (Vec3 + Axis-angle)
- Acceleration
- Acceleration Scale
- IMU rotational correction

Altogether this represents a 21 dimensional state space per tracked object.

There is also a kalman state implementation for lighthouse positions, but after experimentation it is disabled by default
since the tradeoffs involved are not worth the CPU requirement imposed. 

The measurement models for tracked objects is the following:

- IMU (Gyro + Accelerometer data)
- Light data (Processed in batches for a given lighthouse/axis for computational efficiency reasons)
- Poser output (6DoF + covariance matrix)

The light data integration and the poser output are somewhat redundant but the light data integration is capable of 
integrating data which would be underdetermined if it were not reliant on the existing kalman state, which makes it 
largely equivalent to light-based odometry. In practice, the system can be run with few or no poser outputs past the 
initial scene solve, however the output tends to be less robust. 

The one departure from a standard extended kalman filter is that the light data measurement model incorporates an iterative
aspect to it which prevents it from ever integrating data which makes the overall kalman criteria a worse fit at the cost
of CPU usage. It does this by checking the light-error model on what the system would be under the 'optimal' kalman gain, 
and if it goes up due to the linearization assumptions, it simply performs a line search scaling back the gain until 
a gain criteria is reached. It also performs a limited number of gauss newton steps until any of a number of termination 
criteria are reached. Experimentally, the CPU cost of the iterations isn't that noticeable since when the model state
largely agrees with the new data, it's a largely linear landscape and in the cases it is not, the extra CPU usage is 
preferable to destablizing the filter state. 

Both IMU and poser output are also non linear measurement updates, but empirically the iterative approach doesn't benefit
them and so is not used.

# MPFit / Barycentric SVD Poser

The poser provides constant updates to the kalman filter which are minimally biased towards the current kalman state. 
This is useful as an initial bootstrap solution but also provides a somewhat more precise solution for most situations. 

The main position solver uses the C library `MPFit` which is a generic library for solving non-linear least squares via
Levenberg-Marquardt optimization. The main measurement input into this library is the raw sensor data for a given object,
although when the object is stationary it also uses the accelerometer to establish an up direction, which is useful when
there is low light coverage. This optimization is done in axis angle space to minimize the non-free parameter count and
the solver is only allowed to run when the system is sufficiently overdetermined. 

The velocity estimate is also provided, although not yet optimized for. This allows us to incorporate a larger amount of
sensor data since the acceleration over the relatively small (10-20ms) time window should be relatively small while the
velocity (particularly the angular velocity) could be significant. 

In general, the determining equations could have multiple minima, so the initial estimate is provided by the most recent
solution. If the most recent solution doesn't exist or is stale, a 'seed' poser is used which has no need for an initial
estimate. The default seed poser is the barycentric SVD solver, which tends to be much less resilient to noise and expensive
to run, but runs only at startup or when tracking is lost. This solver is essentially the same mathematical framework as
the Efficient Perspective-n-Point(EPnP) algorithm, but adapated to use either the standara camera / lighthouse 1 plane
equations, or the non-standard 'X' plane equations from the lighthouse 2 system. 

