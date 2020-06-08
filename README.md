Magic Motion
============

Magic Motion is a library for making room scale interactive and immersive experiences. It builds point clouds from one or several RGB-D sensors and does background subtraction in real time.

The architechture for background subtraction is built as part of my masters thesis at NTNU, and is fully decoupled from the RGB-D sensor interface. It is very flexible and well suited for experimenting on point cloud segmentation problems.

The `launchpad` folder contains source for an application intended for testing and inspection.

## Coming soon
I will upload a labeled data set for training and testing.

## Acknowledgement
Much thanks to @ocornut for writing Dear ImGui, and @nothings for all the stb libraries.

## How to build
The only dependency is `SDL2`. You should be able to find it most linux package managers. On Ubuntu, you can install it by `apt install libsdl2-dev`.

Run `make` to build both `MagicMotion` and `magicmotion_test` (the launcpad project).
To make use of OpenNI cameras, set `HAS_OPENNI=true` in both `Makefile` and `linux/Makefile`, and set the correct sensor interface in `linux/Makefile`.
The classifier performance can be improved by using OpenCV to do background subtraction on each camera frame before they are transformed into point clouds. To enable this, set `HAS_OPENCV=true` in `linux/Makefile`, and make sure it is set for `classifier_2D` in `src/magicmotion.cpp`.

## How to use
In order to use the recording sensor interface, intended for use when you need reproducible data or don't have access to compatible RGB-D cameras, a file called `recording_video.vid` must exist in the root folder of the project. See `dataset.zip` for one such file.

In the viewer scene, you can fly around using the keyboard, using a FPS controller scheme. There are several options for seeing the raw video frames, and aligning the point clouds.

In the inspector scene, you can load a cloud recording and step through it frame by frame. Using the so-called "boxinator" you can manually alter the background subtraction. Any changes are automatically saved back to the file.
