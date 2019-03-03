# CDT Husky Challenge
## Introduction

This repo hosts all of the code for the CDT husky challenge. It is composed of the following repositories:

* `SLAM`: This repository contains a basic implementation of SLAM
* `sensors`: Provides various helper functions for reading in and visualising data
received from the huskies into MATLAB. For more information see `examples/sensor_example.m`
* `control`: Provides functions for controlling the husky using MATLAB. For more information see
`example/move_robot_example.m`.

And that is pretty much it! **Before using this repository make sure you read the rest of this README...**

## Initial Setup
### Assumptions
From this point on it is assumed that you have the following
dependencies installed:

* A C++ compiler (eg XCode)
* Git
* A departmental install of MATLAB (> R2016b)

### Installing Repository Dependenecies
 In order to use this repository the following dependencies will need to be installed:

* moos: The [MOOS](http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php/Main/HomePage) library provides communication tools that will be important for sending information between apps.
* mex-moos: Provides Matlab wrappers to send and recieve messages over moos from Matlab

In oder to install both of these packages we will use cmake. If you have not used cmake before then run in your command line shell:

```bash
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

To install Moos:
```bash
mkdir -p ~/code/cdt/moos/build
cd ~/code/cdt/moos
git clone https://github.com/themoos/core-moos.git src
cd build
cmake ../src
make -j
```

Now install Mexmoos as:
```bash
mkdir -p ~/code/cdt/mexmoos/build
cd ~/code/cdt/mexmoos
git clone https://github.com/robw4/mexmoos.git src
cd build
cmake ../src
make -j
```

### Add mexmoos to your matlab path
The final step is to add the following paths
to the `startup-cdt.m` script found in this folder.

```buildoutcfg
% --> CHANGE THESE PATHS TO YOUR USER
addpath('/Users/robweston/code/mexmoos/src')
addpath('/Users/robweston/code/mexmoos/build')
```
To check that everything has installed correctly run your `startup_cdt.m` manually followed by `examples/simple_moos_example.m`. In order to see the hlep documentation assoicated with mexmoos type `mexmoos` into the Matlab command prompt which should return the mexmoos function help documentation.

To validate that mexmoos is correctly installed and for more information run `examples\simple_moos_example.m`. To run this without connection to a Husky, run `~/code/cdt/moos/build/bin/MOOSDB` in a terminal and ensure `host = 'localhost';` is active in the example script. This is as far as you need to go before CDT week begins and you get a Husky to befriend.

## Getting started with the robots: Using Mission control
Now that you're with your husky, you can start to interface with the software and sensors. Mission control is used to run applications on the husky from a web based app. You will use this to set up the husky to publish different sensor observations over Moos for you to manipulate in Matlab before sending back speed commands to the husky. In order to launch mission control go open a browser and enter `"192.168.0.14:8080"` to open `MissionControl`. 

Mission control provides various tasks to use the robot. To access these tasks click on the control icon (gear). The tasks that you are interested in can now be found at:

* `sensors -> Bumblebee2_NoLogs`: Run the camera driver to publish images to Moos
* `sensors -> LMS1xxPublishOnly`: Run the Laser driver to publish laser scans to Moos 
* `general -> Reboot_Machine`: Reboot the husky machine
* `general -> Shutdown`: Shutdown the husky

Run the camera and laser drivers using the command above in order to publish images and laser scans to Moos. Both these tasks should now be green, but you'll probably find this isn't the case if you're running directly after turning the husky on. If you're having trouble with the sensor drivers, reboot the Low Level Computer (LLC) from Mission Control (under the "General" section) and try again. We can check what is being published to Moos by running:

* `Moos -> mtm`

The output should look something like:

```buildoutcfg
                   BUMBLEBEE2_IMAGES    17:48:53.586         /home/mrg/code/mrgcorelibs/build/base-cpp/bin/pgr_
DB_CLIENTS           17:48:53.768         MOOSDB_#1            0.5 Hz S  iLMS1xx-14320092,mtm-1...
DB_QOS               17:48:51.767         MOOSDB_#1            0.5 Hz S  /home/mrg/code/mrgcore...
DB_RWSUMMARY         17:48:51.767         MOOSDB_#1            0.5 Hz S  iLMS1xx-14320092=&LMS1...
DB_TIME              17:48:53.768         MOOSDB_#1            0.5 Hz D  1519753733.8
DB_UPTIME            17:48:53.768         MOOSDB_#1            0.5 Hz D  12050.8
DB_VARSUMMARY        17:48:51.767         MOOSDB_#1            0.5 Hz S  BB2-14200208_pose_pb 0...
                   LMS1xx_14320092_laser2d 17:48:53.749         iLMS1xx-14320092     49.9Hz B  *binary* 9.573 KB
MTM-158_STATUS       17:48:53.767         mtm-158              0.5 Hz S  AppErrorFlag=false,Upt...
                   husky_odometry_pose  16:47:26.760         huskysrs             33210.2Hz B  *binary* 202  B
                   husky_state          16:47:26.760         huskysrs             33186.5Hz B  *binary* 111  B
```

## Making the huskies move
**When making the huskies move ensure that you follow these instructions exactly**
First you'll need to connect a gamepad to the husky. To do this follow these instructions **carefully**:

1. **Ensure that the red safety button is pressed in** on the husky.
2. Now press the logitech button on the front of the gamepad until the gamepad lights up green (not flashing) **and** send some speed commands to the husky by twiddling the joystick. **Whilst seemingly random this step including twiddling the joystick is crucial in order to guarantee safe operation of the husky**
3. Now disable the husky saftey switch by releasing the red safety button
4. The husky can now be controlled by using the left (angle) and right (linear) joystick controls

Note, the huskies will only move when they are connected to battery power and not to the mains, unless the battery bypass connector is plugged in.

## Interfacing the Huskies with Matlab
The repository provides several functions to allow you to interface with the Huskies from Matalb. Several examples can be found in the `examples` repository:

* `sensors_example.m`: Shows how to use functions for loading Bumblebee images and laser scans into Matlab for **each** husky as well as undistorting the bumblebee images. Please make sure to reboot the LLC from Mission Control (under the "General" section) before trying to run the sensor drivers (in Mission Control) and display data (in Matlab). The drivers tend to fail if you just switch the husky power on without rebooting the LLC as the sensors take time to initialise.
* `move_robot_example.m`: Provides an example of how to make the huskies move using Matlab. In order to make the husky move, run this script, then when you see "Send command" in your Matlab command window, **hold down the deadman trigger** (RB top-right) and **press Y**. The husky should now execute whatever control command you are sending from the while loop. As soon as the deadman switch is released, the robot will stop moving. For diagnostic information, go to the "Robot" task group and monitor the output for the Small Robots Stack task.
* `simple_moos_example`: Shows how to use Moos to send messages to and from the husky.
