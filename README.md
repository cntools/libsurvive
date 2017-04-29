# libsurvive

![Logo](https://cloud.githubusercontent.com/assets/2748168/24084003/9095c98a-0cb8-11e7-88a3-575f9f4c7bb4.png)

An Open-Source tool for working with lighthouse-based tracking data, including support for the HTC Vive, which is still in the experimental phase.

Most of the development is discussed on Discord.  Join the chat and discussion here: https://discordapp.com/invite/7QbCAGS

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


## Nomenclature

WRT = With Respect To
PoV / POV = Point of View (typically WRT to a LH, sometimes (though rarely) a sensor)
LH = Lighthouse = Base Station = A device that produces a 1.8 MHz modulated sync pulse in IR and then sweeps the scene with laser planes.
Sync Pulse = A pulse of modulated IR data sent from a ligthhouse, typically by the floodlight aspect of a lighthouse.
Sweep Pulse = The evenlope created by a laser sweeping over a light sensor.
OOTX = Omnidirectional Optical Transmitter = Data encoded in the sync pulses of the LHs.
HMD = Headset = Main sensor receiver with a visual display for a human.
WM = Watchman = Controller = The HTC Vive controller.
TR = Tracker = Official HTC Tracker.
LightcapElement = A single pulse of light, including a timestamp, source sensor and length of pulse.
Disambiguator = System that accepts lightcap elements and pulls out OOTX data and relative sweep times of sweep pulses.
Poser = Device to convert series of angles from a LH's PoV

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
5. (long shot) Make the vive viable for use with Intel Integrated Graphics systems. [It works with HD4000 using DisplayPort. See "Intel Integrated Graphics" section below.]

Will ~~I~~ we succeed?  Probably not.  ~~Definitely going to try!~~ Though it's looking like we might.


## External dependencies

* libUSB (Linux) or hidapi (Win, OSX; included in redist)
* pthread
* libX11 (Linux) or Native (win32) or OpenGL (OSX)
* zlib (Linux) or puff.c (win32, included in redist)

## Architecture

<TABLE><TR><TH>Description</TH><TH>Diagram</TH>
</TR><TR>
<TD WIDTH=50%>

### Layout

In the src/ folder you'll find most of the internal code that is part of libsurvive.  The redist/ folder contains code that libsurvive uses that was copied from other projects.  Libsurvive links to other libraries, but very few.  You'll find that most of the functionality lies within libsurvive or in the redist folder.  For the user-facing headers you can find them in the include/ folder.

### Logical Data Flow

There are device drivers, such as survive_vive.c which connect to physical devices, via libUSB, hidapi or another method and collect raw IMU and lightcap data.  Lightcap data is specically a sensor ID, light pulse length (in ticks) and the time of the light pulse (in ticks).  The driver also must provide locations of the sensors to populate the SurviveObject structures of whatever sensors that driver is responsible for.

Once this data is collected, the light pulses are disambiguated (see survive_data.c) into OOTX sync pulses (id -1, -2 depending on lighthouse) as well as sweep pulses which provide the time of a sweep pulse passing the sensor.  This is passed off to "lightproc."  The default behavior for lightproc can be found in survive.c.  The user may override this, however, if they have interest in the raw pulse information for whatever reason.  The default behavior for lightproc determines the time delta between the sync pulse and the sweep pulse time and derives angle.  The derivation for the angle is simply calculated by the time difference between the sync pulse and the sweep pulse.   It then calls "angleproc"  (not implemented yet: Using OOTX data from lighthouses to correct and tweak angles)

Angleproc may also be overridden by the user for similar purposes to for "angleproc" which passes its information off to a calibrator (if running) as well as to whatever posers are enabled.  The posers will take this data and determine position from it.

</TD>
<TD WIDTH=50%><img src=https://raw.githubusercontent.com/cnlohr/libsurvive/master/useful_files/FunctionalSystem.png width=400></TD>
</TR>
</TABLE>

## Lists of components

Component Type | Component | Description | Authors
--- | --- | --- | ---
Poser | [poser_charlesslow.c](src/poser_charlesslow.c) | A very slow, but exhaustive poser system. Calibration only. | [@cnlohr](https://github.com/cnlohr)
Poser | [poser_daveortho.c](src/poser_daveortho.c) | A very fast system using orthograpic view and affine transformations. Calibration only (for now) | [@ultramn](https://github.com/ultramn)
Poser | [poser_dummy.c](src/poser_dummy.c) | Template for posers | [@cnlohr](https://github.com/cnlohr)
Poser | [poser_octavioradii.c](src/poser_octavioradii.c) | A potentially very fast poser that works by finding the best fit of the distances from the lighthouse to each sensor that matches the known distances between sensors, given the known angles of a lighthouse sweep.  Incomplete- distances appear to be found correctly, but more work needed to turn this into a pose. | [@mwturvey](https://github.com/mwturvey) and [@octavio2895](https://github.com/octavio2895)
Poser | [poser_turveytori.c](src/poser_turveytori.c) | A moderately fast, fairly high precision poser that works by determine the angle at the lighthouse between many sets of two sensors.  Using the inscirbed angle theorom, each set defines a torus of possible locations of the lighthouse.  Multiple sets define multiple tori, and this poser finds most likely location of the lighthouse using least-squares distance.   Best suited for calibration, but is can be used for real-time tracking on a powerful system.  | [@mwturvey](https://github.com/mwturvey)
Disambiguator | [survive_data.c](src/survive_data.c) (currently #ifdefed out) | The old disambiguator - very fast, but slightly buggy. | [@cnlohr](https://github.com/cnlohr)
Disambiguator | [survive_data.c](src/survive_data.c) (current disambiguator) | More complicated but much more robust disambiguator | [@mwturvey](https://github.com/mwturvey)
Dismabiguator | superceded disambiguator | A more sophisticated disambiguator, development abandoned.  Removed from tree. |  [@jpicht](https://github.com/jpicht)
Driver | [survive_vive.c](src/survive_vive.c) | Driver for HTC Vive HMD, Watchmen (wired+wireless) and Tracker | [@cnlohr](https://github.com/cnlohr) and [@mwturvey](https://github.com/mwturvey)
OOTX Decoder | [ootx_decoder.c](src/ootx_decoder.c) | The system that takes the pulse-codes from the sync pulses from the lighthouses and get [OOTX Data](https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md) | [@axlecrusher](https://github.com/axlecrusher)

## Component Pluggability Matrix

Component Type | Pluggability method
--- | ---
Driver | Dynamically loadable runtime, can co-exist with other drivers.
Poser | Selectable by configuration at runtime
Disambiguator | Selectable by #define
OOTX Decoder | Not Pluggable



## Intel Integrated Graphics

The limiting factor for Vive viability on a given computer is the maximum available pixel clock frequency, and frequency limitations of the HDMI port, and HDMI and DisplayPort video cables. DisplayPort can support higher frequencies than HDMI, on Ivy Bridge HD4000 graphics. In fact, the Vive works with HD4000 graphics using DisplayPort, with native EDID resolution (2160x1200@90Hz).

To support the Vive on HDMI, you either need a newer version of HDMI, or you need to define a custom resolution that respects pixel clock and video port limits, and is also accepted and displayed by the Vive. So far, we have not had success using custom resolutions on linux or on Windows. Windows imposes additional limitations in the form of restriction of WHQL certified drivers forbidden from using custom display resolutions (only allowing those defined by EDID in the monitor). Intel has released uncertified beta drivers for Haswell and newer processors, which should be able to support custom resolutions for the Vive (untested at this time).



## Addendum and notes

Thanks to Mr. Fault for our logo!
Special thanks to @nairol for an extreme amount of detail in reverse engineering the existing HTC Vive system on his https://github.com/nairol/LighthouseRedox project.
