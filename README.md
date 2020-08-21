# libsurvive [![Build Status](https://travis-ci.com/cntools/libsurvive.svg?branch=master)](https://travis-ci.com/cntools/libsurvive)


![Logo](https://cloud.githubusercontent.com/assets/2748168/24084003/9095c98a-0cb8-11e7-88a3-575f9f4c7bb4.png)


Libsurvive is a set of tools and libraries that enable 6 dof tracking on 
[lighthouse and vive](https://en.wikipedia.org/wiki/HTC_Vive) based systems that is completely open source and can run 
on any device. It currently supports both SteamVR 1.0 and SteamVR 2.0 generation of devices and should support any tracked
object commercially available. 

Since the focus is on tracking; it does not independently run the HMD. For an open souce stack that does that see [monado](https://monado.freedesktop.org/)

Most of the development is discussed on Discord.  [Join the chat and discussion in our discord!](https://discordapp.com/invite/7QbCAGS)

# Quick start

## Debian
```
git clone git@github.com:cntools/libsurvive.git
cd libsurvive
sudo cp ./useful_files/81-vive.rules to /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
sudo apt-get install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev websocketd
sudo make
```

Plug in a headset / tracker / controller / etc and run:
```
./bin/survive-websocketd & xdg-open ./tools/viz/index.html
```

This should calibrate and display your setup. 

[![Watch video](https://img.youtube.com/vi/l4doRSXM0tU/0.jpg)](https://www.youtube.com/watch?v=l4doRSXM0tU)

## Windows

For windows you must open the CMakeLists file in something like [CMake GUI](https://cmake.org/runningcmake/) to build
from source using one of the visual studio generators. The build uses NuGet to get the [necessary development dependencies](https://www.nuget.org/packages/lapacke/).

[Websocketd](http://websocketd.com/) should work the same with with the visualization tool.

# Current Status

The tracking and device enumeration work fairly well at this point; but there isn't an extremely large testing base and 
the tools aren't as polished as the comparable ones bundled in SteamVR. Work is ongoing to quantify how accurate the 
tracking is and to improve the user experience. 

# Roadmap

A very loose collection of things that are on the short term agenda:

- Dynamic correction of calibration. This would detect and silently recalibrate lighthouses that might have moved. The 
end goal is to have a system where the user is never aware of any calibration requirements. 
- An android binary / port
- Hard numbers on the accuracy and precision of libsurvive as a tracking system. If anyone wants to contribute time testing
on a CNC please get in our [discord](https://discordapp.com/invite/7QbCAGS)
- Better handling of data starvation -- if the USB or radio connection stutters too long, the tracking will occasionally
glitch out for a brief period. 
- Use something like usbmon for windows

# Getting Started

If you followed the quick start guide, you'll notice that the first thing you must do (for linux) is install udev rules:

```
sudo cp ./useful_files/81-vive.rules to /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

This allows one to use those devices without having root. No such steps are necessary in windows. 

After that, when you run `survive-cli` or `survive-websocketd` it should recognize any plugged in vive devices and start
calibrating and tracking those devices. 

**Important: Close SteamVR for best results. Depending on the system, libsurvive
will either cause SteamVR to lose connection to the device or will compete for bandwidth with it**

## Calibration

Calibration is the process which establishes where the lighthouses are set up in relation to the tracked objects. The
first time you run libsurvive, it can take up to ten seconds to communicate with the lighthouses and figure out where 
they are. During this time you should not move the devices.

Calibration should work with any device, but the HMD gives the best results due to it's size and number of sensors. If
the HMD and other controllers are present at startup, it will automatically prioritize using the data from the HMD.

Once you do this one time, it is saved in `config.json` in your working directory. If you delete this file, it will simply
recalibrate. 

If you have a large space, and you can not centrally locate a single device to 'see' all lighthouses, you can calibrate
a few lighthouses and move the tracked object into the field of view of the uncalibrated lighthouse while keeping it in
view of at least one of the calibrated ones. Set it down so it doesn't move; and the remaining lighthouse(s) should now
calibrate. 

**Important: Resetting calibration automatically when a lighthouse is moved is being planned on but is not currently 
available. If one of the calibrated lighthouses are moved, you either have to redo calibration by deleting the config.json
file or passing `--force-calibrate` into any of the libsurvive tools.**

Version 1 systems will have an output that looks roughly like:

```
Info: Stage 2 good - continuing. 32 1 0
Info: Stage 2 good - continuing. 32 1 1
Info: Stage 2 good - continuing. 32 1 2
Info: Stage 2 good - continuing. 32 1 3
Info: Stage 2 good - continuing. 32 1 4
Info: Stage 2 moving to stage 3. 32 1 5
Lighthouse Pose: [0][ 0.28407975, 0.93606335,-0.37406892] [ 0.05594964,-0.33792987, 0.93887696, 0.03439615]
Info: Stage 4 succeeded.
```
[For reference, here is an older recording of how a properly running calibration looks like](https://haagch.frickel.club/Peek%202018-02-21%2023-23.webm).

Version 2 systems will have output that looks something like:
```
Info: Attempting to solve for 0 with 30 meas
Info: Attempting to solve for 1 with 21 meas
Info: Solved for 0 with error of 0.005446/0.000000
Info: Solved for 1 with error of 0.005446/0.000000
Info: Using LH 0 (d99e7eac) as reference lighthouse
Info: Position found for LH 0(d99e7eac)
Info: Position found for LH 1(fe0398ef)
```

## Visualization

The main visualization tool is a THREE.js page which is fed data through (websocketd)[http://websocketd.com/]. To use
this tool, run `survive-websocketd [options]` and open a web browser to `./tools/viz/index.html` from the root of the
cloned repo. 

![Visuzliation Screenshot](https://raw.githubusercontent.com/cnlohr/libsurvive/master/useful_files/viz_screenshot.png)

## libsurvive Tools

- `survive-cli` - This is the main command line interface to the library; really just a very thin wrapper around the library.
- `survive-websocketd` - A script which runs `survive-cli` through `websocketd` with all the appropriate flags set.
- `sensors-readout` - Display raw sensor information in a ncurses display

## Using libsurvive in your own application

Libsurvive offers a low level API as well as a higher level application API.

Example code for the application interface can be found in [api_example.c](https://github.com/cntools/libsurvive/blob/master/api_example.c).
 
Have a look at the other libsurvive tools a the top level of the repository for example usage of the lower level API.

### Python Bindings

To build the python bindings, run `python setup.py install` in the repo root. This should install the `pysurvive` package.

An example which streams poses out as they come in:

```
import pysurvive
import sys

actx = pysurvive.SimpleContext(sys.argv)

for obj in actx.Objects():
    print(obj.Name())

while actx.Running():
    updated = actx.NextUpdated()
    if updated:
        print(updated.Name(), updated.Pose())
```

There are more examples in `./bindings/python`.

### C# Bindings

There are currently bindings in `./bindings/cs` for C#/.net although they are somewhat out of date and still need to be 
tied into the CI system. If anyone has interest in these bindings please hop into the [discord](https://discordapp.com/invite/7QbCAGS). 

## Data recording

There are a lot of things that can go wrong to give bad tracking or calibration results, and given the wide variety of 
devices, configurations and use cases, it's extremely helpful when trying to resolve bugs if there is a data recording 
of the bug. This also can get added to our CI system to have automatic testing against that use case. 

There are two mechanisms available for recording data: `--record` and `--usbmon-record`. 

`--record` is available in every installation, and less setup to use so for pure tracking issues this is usually the 
approach to take. However, if the issue is in parsing and understanding the low level data packes from the tracked device,
the `--usbmon-record` option might be more helpful. 

### Normal recording

When running any of the libsurvive tools, pass in `--record <filename>.rec.gz`. This will create a data log of everything
the system sees as it runs in that file. This records a large amount of data, so if it is allowed to run for a long time
that file might get very large. 

To playback that file, run:

`./survive-cli --playback <filename>.rec.gz`

### Raw USB recording

Occasionally, when dealing with new hardware or certain types of bugs that cause an issue in the USB layer, it is necessary to have a raw capture of the USB data seen / sent. The USBMON driver lets you do this.

Currently this driver is only available on linux and you must have libpcap installed -- `sudo apt install libpcap-dev`. You also need the `usbmon` kernel module installed; but many linux flavors come with that built in. 

To start usbmon and prepare it for use for all users, run:

```
sudo modprobe usbmon
sudo setfacl -m u:$USER:r /dev/usbmon* # In sensitive environments, you can run survive-cli with sudo instead.
```

To capture usb data, run:

```
./survive-cli --usbmon-record <filename>.pcap.gz --htcvive <additional options>` 
```

You can run that playback with:
```
./survive-cli --usbmon-playback <filename>.pcap.gz [--playback-factor x] <additional options>` 
```

If you are sending this file for analysis, note that you need the accompanying `*.usbdevs` file with it to be useful. If you follow the `*.pcap.gz` convention, run something like

```
zip logs.zip *.pcap* config.json
```

and post the `logs.zip` to an issue or to discord. 

This driver specifically only captures devices on a white list of VR equipment; but if you don't want to publish raw USB
data to the internet, ask in discord for who to send it to in a private message. 

## Common command line flags

Libsurvive is very configurable, and contains a lot of command line options depending on the drivers and options given
at build time. 

It's recommended that you install the bash completion for libsurvive if you are working with the command line options:

`sudo cp survive_autocomplete.sh /etc/bash_completion.d/`.

The most useful command line option for debugging is `--v` -- this sets the reporting level. This verbosity level roughly follows this guideline:

- `--v 10` - Statistics and information displayed either at the start or end of the application.
- `--v 100` - Information displayed at many common points while tracking
- `--v 150` - Information displayed for almost every pose output
- `--v 250` - Information displayed for almost all light data event in the system
- `--v 1000` - Everything. 

Running at higher verbosity (>100) will make the visualization tool sluggish. 

`--force-calibrate`: This reruns calibration but reuses OOTX; which makes it much faster to run. 
`--playback-factor`: When playing back a recording, this will speed up the playback (0 is run everything as fast as possible) or slow it down (2 takes twice as much time)

# Drivers

These are the different drivers for providing information into libsurvive. All of them are encapsulated in the `src` directory
with a `driver_` prefix. Each driver can be specified with a `--<driver-name>` flag. You can disable a default driver (eg, `htcvive`) with
`--no-<driver-name>`.

- `htcvive` - This is the main driver which provides data via USB connection to vive hardware.
- `simulator` - This simulates a device floating around while providing realistic light / IMU data into libsurvive. Useful
for testing different features.
- `playback` - The playback driver is what enables record/playback functionality. It replays a file into the various data points.
- `usbmon` - USBmon can be ran concurrent with steamvr to allow both systems to use the tracked object data. 
- `openvr` - This driver exposes external poses and velocities and can be ran with `usbmon` to compare the two systems.

# FAQ

Other projects using libsurvive

 * The [Monado OpenXR runtime](https://monado.freedesktop.org/) uses libsurvive as one of its HMD and controller drivers.
   * OpenXR applications that run on Monado include the [OpenXR plugin for Godot 3.x](https://github.com/GodotVR/godot_openxr)
 * There is a very unofficial (and not upstreamable) [OpenHMD/libsurvive fork](https://github.com/ChristophHaag/OpenHMD/commits/libsurvive2) that adds a libsurvive driver.
   * This OpenHMD/libsurvive fork can be plugged into [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) or used natively with the [OpenHMD plugin for Godot 3.x](https://github.com/BastiaanOlij/godot_openhmd).

## Addendum and notes

Thanks to Mr. Faul for our logo!
Special thanks to @nairol for an extreme amount of detail in reverse engineering the existing HTC Vive system on his https://github.com/nairol/LighthouseRedox project.
