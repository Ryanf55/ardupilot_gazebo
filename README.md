# ArduPilot Gazebo Plugin

[![ubuntu-build](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ubuntu-build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ubuntu-build.yml)
[![ccplint](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccplint.yml/badge.svg)](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccplint.yml)
[![cppcheck](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccpcheck.yml/badge.svg)](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccpcheck.yml)

This is the official ArduPilot plugin for [Gazebo](https://gazebosim.org/home).
It replaces the previous
[`ardupilot_gazebo`](https://github.com/khancyr/ardupilot_gazebo)
plugin and provides support for the recent releases of the Gazebo simulator
[(Gazebo Garden)](https://gazebosim.org/docs/garden/install) and [(Gazebo Harmonic)](https://gazebosim.org/docs/harmonic/install).

It also adds the following features:

- More flexible data exchange between SITL and Gazebo using JSON.
- Additional sensors supported.
- True simulation lockstepping. It is now possible to use GDB to stop
  the Gazebo time for debugging.
- Improved 3D rendering using the `ogre2` rendering engine.

The project comprises a Gazebo plugin to connect to ArduPilot SITL
(Software In The Loop) and some example models and worlds.

## Prerequisites

Gazebo Garden or Harmonic is supported on Ubuntu 22.04 (Jammy).
If you are running Ubuntu as a virtual machine you will need at least
Ubuntu 20.04 in order to have the OpenGL support required for the
`ogre2` render engine. Gazebo and ArduPilot SITL will also run on macOS
(Big Sur, Monterey and Venturua; Intel and M1 devices).

Follow the instructions for a binary install of
[Gazebo Garden](https://gazebosim.org/docs/garden/install) or [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
and verify that Gazebo is running correctly.

Set up an [ArduPilot development environment](https://ardupilot.org/dev/index.html).
In the following it is assumed that you are able to run ArduPilot SITL using
the [MAVProxy GCS](https://ardupilot.org/mavproxy/index.html).

## Installation

Install additional dependencies:

### Ubuntu

Gazebo Garden:

```bash
sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
```

Or Gazebo Harmonic:

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
```

### macOS

```bash
brew update
brew install rapidjson
```

Ensure the `GZ_VERSION` environment variable is set to either
`garden` or `harmonic`.

Clone the repo and build:

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

## Configure

Set the Gazebo environment variables in your `.bashrc` or `.zshrc` or in 
the terminal used to run Gazebo.

#### Terminal

Assuming that you have cloned the repository to `$HOME/ardupilot_gazebo`:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

#### .bashrc or .zshrc

Assuming that you have cloned the repository to `$HOME/ardupilot_gazebo`:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Reload your terminal with `source ~/.bashrc` (or `source ~/.zshrc` on macOS).

## Usage

### 1. Iris quad-copter

#### Run Gazebo

```bash
gz sim -v4 -r iris_runway.sdf
```

The `-v4` parameter is not mandatory, it shows additional information and is
useful for troubleshooting.

#### Run ArduPilot SITL

To run an ArduPilot simulation with Gazebo, the frame should have `gazebo-`
in it and have `JSON` as model. Other commandline parameters are the same
as usual on SITL.

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

#### Arm and takeoff

```bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

### 2. Zephyr delta wing  

The Zephyr delta wing is positioned on the runway for vertical take-off. 

#### Run Gazebo

```bash
gz sim -v4 -r zephyr_runway.sdf
```

#### Run ArduPilot SITL

```bash
sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console
```

#### Arm, takeoff and circle

```bash
MANUAL> mode fbwa
FBWA> arm throttle
FBWA> rc 3 1800
FBWA> mode circle
```

#### Increase the simulation speed

The `zephyr_runway.sdf` world has a `<physics>` element configured to run
faster than real time: 

```xml
<physics name="1ms" type="ignore">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>-1.0</real_time_factor>
</physics>
```

To see the effect of the speed-up set the param `SIM_SPEEDUP` to a value
greater than one:

```bash
MANUAL> param set SIM_SPEEDUP 10
```

## Models

In addition to the Iris and Zephyr models included here, a selection
of models configured use the ArduPilot Gazebo plugin is available in
[ArduPilot/SITL_Models](https://github.com/ArduPilot/SITL_Models). 
Click on the images to see further details.

<table>
<tr>
<td title="Alti Transition">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/AltiTransition.md">
<img src="https://user-images.githubusercontent.com/24916364/150612555-958a64d4-c434-4f90-94bd-678e6b6011ec.png" width="100%" style="display: block;">
</a>
</td>
<td title="SkyCat TVBS">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/SkyCatTVBS.md">
<img src="https://user-images.githubusercontent.com/24916364/145025150-4e7e48e1-3e83-4c83-be7b-b944db1d9152.png" width="100%" style="display: block;">
</a>
</td>
<td title="Skywalker X8">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/SkywalkerX8.md">
<img src="https://user-images.githubusercontent.com/24916364/142733947-1a39e963-0aea-4b1b-a57b-85455b2278fe.png" width="100%" style="display: block;">
</a>
</td>
</tr>
<tr>
<td title="Quadruped">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/Quadruped.md">
<img src="https://user-images.githubusercontent.com/24916364/144449710-5bab34b4-dabf-410f-b276-d290ddbb54b2.gif" width="100%" style="display: block;">
</a>
</td>
<td title="WildThumper">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/WildThumper.md">
<img src="https://user-images.githubusercontent.com/24916364/144286154-231ac9b3-e54b-489f-b35e-bc2adb4b1aa0.png" width="100%" style="display: block;">
</a>
</td>
<td title="Rover Playpen">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/RoverPlayPen.md">
<img src="https://user-images.githubusercontent.com/24916364/144513412-1b0661f1-fdf8-4aed-a745-e8bb73ffca91.jpg" width="100%" style="display: block;">
</a>
</td>
</tr>

</td>
</tr>
<tr>
<td title="Swan-K1">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/Swan-K1.md">
<img src="https://user-images.githubusercontent.com/24916364/210408630-01e5f56d-57ba-430e-b04d-62cb8d232527.png" width="100%" style="display: block;">
</a>
</td>
<td title="Sawppy Rover">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/Sawppy.md">
<img src="https://user-images.githubusercontent.com/24916364/210653579-e635ffc2-2962-4221-83a8-9622915a4121.png" width="100%" style="display: block;">
</a>
</td>
<td title="Hexapod Copter">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/HexapodCopter.md">
<img src="https://user-images.githubusercontent.com/24916364/225340320-9aa31fe2-4602-4036-ba6b-491f72097c01.jpg" width="100%" style="display: block;">
</a>
</td>
</tr>

</table>

## Troubleshooting

For issues concerning installing and running Gazebo on your platform please
consult the Gazebo documentation for [troubleshooting frequent issues](https://gazebosim.org/docs/harmonic/troubleshooting#ubuntu).

## Architecture

### References

* [Ardupilot SITL JSON Backend](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md)


### What are the main components

* ArduPilot SITL - Runs the autopilot code with SITL
* Gazebo - Runs the FDM (flight dynamics model)

```mermaid

```

### How

* ArduPilot SITL sends a binary data out which is the servo outputs
```
    uint16 magic = 18458
    uint16 frame_rate
    uint32 frame_count
    uint16 pwm[16]
```
* Gazebo receives this, uses controller XML to define what to do with the PWM.
  * The physics engine gets these values, updates the state, and advances, and then reads the state.
* Gazebo replies with the JSON state
  * At minimum, it includes what the EKF needs
    * pose
    * rates
    * IMU's
* Lockstep means 1 servo outputs from SITL triggers 1 JSON FDM state
  * Either end can stop and wait for the next data

* The JSON packet also supports some simple data:
  * Range sensors
  * Airspeed

* The SITL model has a SIM_JSON backend that periodically sends out to port. 
* Once a server replies (gazebo plugin), it replies SITL back an address.

* The FDM runs in Gazebo, The "controller" is SITL.

The problem with the JSON packet is that it all needs to be sent at 1000Hz, but not all sensors provide that.
It doesn't scale well as a model. The core JSON FDM is ok. For other stuff like GPS, Lidar, should likely use a different channel (message based). The messages decoded in SIM_JSON are what's currently possible.
We don't plan to extend this.

Gazebo has a NavSat sensor, this is currently not sent to ArduPilot. Because ArduPilot only supplies servo outputs,
these simulation parameters are not mirrored across to Gazebo.


ArduPilot SITL runs Linux ArduPilot binary that has setup() and loop().
This is almost the same binary as on a FC.

Historically, the ArduPilot gazebo plugin did its own controller. More recently, the commands get forwarded over gazewbo transport to run joint velocity controllers. Servo commands go from ardupilot gazebo plugin, put on gz bus, and picked up by more standard plugins. With a model set up in this way, it's more similar to PX4.

PX4's equivalent puts data directly on gazebo transport, but it prevents lockstep. 
There is some detail on how gazebo physics work in gazebo (TODO add links).

The ArduPilot plugin exists to provide lockstep these days, but this needs included in the model SDF.
Thus, PX4 models can only depend on gazebo packages, which makes them more shareable. 
This can be converged.

Any sensor data from gazebo goes through JSON today, anything else needs another mechanism. 

There is an example of sending avoidance data from Lidar over MAVROS API (TODO add rover URL).
This is an open item of work - it should be topic-based.
One simple way is use one socket for each channel, but this doesn't scale well.
Multiple vehicles need sane socket ID's.

Or, listen to gazebo transport directly in SITL, which is ZMQ and protobuf. Sim on hardware currently works, so
we have to decide.

Use cases:
* Simulate GPS in physics engine and use it in SITL
* Simulate Camera
* Proximity Sensor

In many cases, it's a companion computer that processes the simulated data, such as a companion data.
Key question: What are we expecting the autopilot to process.


ArduPilot SITL by itself bolts on a physics engine and controller in the same package. 
Usually, FC on hardware just runs "controller". 
HIL testing has phsyics engine embedded in sim_vehicle. 
SITL doesn't have collisions or other vehicles. 

Running AP with Gazebo is different than running SITL alone.
Realflight and JSBsim have good dynamics.

### Extending to other autopilots

Gazebo is given commands to move actuators, gazebo handles all physics. Render engine broadcasts info and displays it.
Gazebo can run the FDM on the vehicle and all other entities, or it could simulate parts of the world.

Assumptions:
* You have an FDM already running for the vehicle
* You want use gazebo to show the path of the vehicle, but we want to add other things
* Gazebo could simulate external physics



### Lockstep

### FDM
