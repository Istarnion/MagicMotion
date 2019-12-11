RGB-D Launchpad
===============

This code base is meant as a starting point for writing applications that use RGB-D cameras to generate and process point cloud data.
It supports multiple simultanious cameras and has features for simple 3D space user interaction.

The software is based on OpenNI 2, so with the right drivers, it should run on the various Orbbec Astra cameras, the Itel D4** series, the old Kinect, and others. It is only tested with Orbbec Astra and Orbbec Astra Mini.

## Setup
Before building, you need to get OpenNI. You can download it ![here](https://structure.io/openni). Follow the instructions to set it up, and place the folder in the root of this project. Then _copy_ the Drivers/OpenNI2 folder and place it as well in the root of this project. Make sure the drivers you want present, and you should be ready to build.

## Building
run `make`. There are few external dependencies and it uses a unity build system.

## Usage
In the future there will be a more user friendly way of doing this, but for now edit `main.cpp` to start the wanted scene, compile and run.

The current scenes are:
- `scene_viewer` For viewing point clouds and aligning them.
- `scene_video` For viewing the video streams directly. Can show RGB, grayscale depth, and colorized depth.
- `scene_interaction` For testing acceleration structures and user interaction. This is currently under development and not yet functional.

New experiments and uses should get their own scene. If an experiment yields an interesting result, that part of the scene should be factored out to `utils.h` or its own decoupled module so that other older or future scenes can easily access it.

## Acknowledgement
Much thanks to @ocornut for writing Dear ImGui, and @nothings for all the stb libraries.

