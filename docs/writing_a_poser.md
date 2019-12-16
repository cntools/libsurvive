# How to write a poser

Posers in libsurvive serve two related purposes:

1) If the lighthouse position isn't known, solve for them.
2) If the lighthouse position is known, solve for the position of objects in the scene. 

To do this, a poser is given a variety of different signals about the current state of the input. 

# Registering a Poser

In an effort to decouple posers from the underlying mechanics, posers are registered through a link-time 
constructor. Make a function in the following form:

```
#include "survive.h"

int PoserMyPoser(SurviveObject *so, PoserFnData *pd) {
	switch (pd->pt) {
	case POSERDATA_LIGHT: {
		PoserDataLight *lightData = (PoserDataLight *)pd;
		return -1;
	}
	case POSERDATA_FULL_SCENE: {
	    PoserDataFullScene pdfs = (PoserDataFullScene *)(pd);
		return -1;
	}
	case POSERDATA_IMU: {
		PoserDataIMU *imuData = (PoserDataIMU *)pd;
        return -1;
	}
	return -1;
}

REGISTER_LINKTIME(PoserMyPoser);

```

Note that the prefix of the function MUST be "Poser" for it to work. 

Now compile it into the shared object, and it will be one of the poser options available. 

## Input to a poser

`PoserFnData` is a tagged union that contains the new input to the poser. See above for how to get the concrete
types for each type of event. 

If you don't handle a given input, return -1. If you do, return a 0. If you have a more specific
error number, you can return that as well. 

## Output from a poser

To report an objects location for any one of the events above, call the function

`PoserData_poser_raw_pose_func`. See the documentation in poser.h for more info.

To report an initial solution to the lighthouse solution, call the function `PoserData_lighthouse_pose_func`. Again, 
see documention in poser.h

## Calibration data

Note that by default, no angle data is presented in a calibrated fashion. Depending on your use case, you
may want to treat the data by grouping X, Y pairs and trying to undistort them. You can do this with the library
function `survive_apply_bsd_calibration` from `survive_reproject.h`. Please understand though that this is, at best, an 
approximation and can be far off if the angle values are too spread out in time. 

