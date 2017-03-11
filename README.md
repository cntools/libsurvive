# libsurvive

**WARNING PROJECT NOT YET IN EXPERIMENTAL PHASE**

Discord: https://discordapp.com/invite/7QbCAGS

## Livestream collection
| Note                                   | Youtube URL                                 | Run time |
| -------------------------------------- | ------------------------------------------- | -------- |
| First livestream                       | https://www.youtube.com/watch?v=sv_AVI9kHN4 | 5:01:25  |
| Second livestream                      | https://www.youtube.com/watch?v=gFyEbGQ88s4 | 4:03:26  |
| Summary of first and second livestream | https://www.youtube.com/watch?v=oHJkpNakswM | 23:00   |
| Third livestream                       | https://www.youtube.com/watch?v=RExji5EtSzE | 4:11:16 |
| Fourth livestream                      | https://www.youtube.com/watch?v=fces1O7kWGY | 4:50:33 |
| Fifth livestream                       | https://www.youtube.com/watch?v=hHt3twW5_fI | 3:13:38 |
| Sixth livestream                       | https://www.youtube.com/watch?v=JsfkNRFkFM4 | 3:44:49 |
| Seventh livestream                     | https://www.youtube.com/watch?v=EKSHvO3QSWY | 1:17:21 |

Notes from second livestream trying to reverse engineer the watchman protocol: https://gist.github.com/cnlohr/581c433f36f4249f8bbc9c2b6450ef0e

Please see the issues for what help needs to be done now!

## Extra resources

HackADay article and video with Dr. Yates on how they made the Vive a thing. http://hackaday.com/2016/12/21/alan-yates-why-valves-lighthouse-cant-work/

## Getting things working

There are two things you should consider doing to your system before running libsurvive.

(1) Install the udev rules:  ```cp usefulfiles/81-vive.rules to /etc/udev/rules.d/``` and reboot.
(2) If you are running on an NVIDIA Card, you will need to AllowHMD to true.  Add the following line to your /etc/X11/xorg.conf device section:  ```Option "AllowHMD" "yes"```

## Introduction
High-performance HTC Vive Library

I say "high-performance" really this project is based tightly off of OSVR-Vive-Libre, but, specifically is an attempt to:

1. Minimize external libraries.  Actual reason for starting this: Downloading all of the libraries needed for OSVR-Vive-Libre maxed out my data plan.
2. Put it under an open-source instead of a force-source license.  (GPL to MIT/X11)
3. Write it in C.
4. Avoid extra layers where convenient.
5. (long shot) Make the vive vivable for use with Intel Integrated Graphics systems.


Will ~~I~~ we succeed?  Probably not.

Definitely going to try!


## External dependencies

* libUSB
* pthread
* libX11 (where applicable)
* zlib (may use puff.c if needed)

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





