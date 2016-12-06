# libsurvive

**WARNING PROJECT NOT YET IN EXPERIMENTAL PHASE**

First livestream: https://www.youtube.com/watch?v=sv_AVI9kHN4

Second livestream: https://www.youtube.com/watch?v=gFyEbGQ88s4

Notes from second livestream trying to reverse engineer the watchman protocol: https://gist.github.com/cnlohr/581c433f36f4249f8bbc9c2b6450ef0e



## Introduction
High-performance HTC Vive Library

I say "high-performance" really this project is based tightly off of OSVR-Vive-Libre, but, specifically is an attempt to:

1. Minimize external libraries.  Actual reason for starting this: Downloading all of the libraries needed for OSVR-Vive-Libre maxed out my data plan.
2. Put it under an open-source instead of a force-source license.  (GPL to MIT/X11)
3. Write it in C.
4. Avoid extra layers where convenient.
5. (long shot) Make the vive vivable for use with Intel Integrated Graphics systems.


Will I succeed?  Probably not.

Definitely going to try!


## External dependencies

* libUSB
* pthread
* (planned, may not be needed) lapack

If I ever get to video output... OpenGL.

## Architecture

There is an internal representation and an external representation.  These lines may get blurred.  Internal representation lives in .h files in the ```src/``` folder. External lives in ```include/``` folder.  

It is written in some fairly stout "layers" which are basically just function calls:

|  Layer | Description | Status |
| ------- | ------------- | -------- |
| survive_usb.c | Data is taken in at "survive_usb.c" from libusb. | Done |
| survive_data.c | Raw HID messages are processed into logical "light" "analog" and "imu" messages. | Mostly done, Missing light data from controllers, and lighthouse data. |
| survive_process.c | Process the high-level data into solutions for | Not yet started.  Will be done by ultramn |

I may or may not read data from the Vive regarding configuration.  If I do, it would be added to the survive_usb.c





