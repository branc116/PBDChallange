# PBD challenge

[Challenge accepted](https://matthias-research.github.io/pages/challenges/challenges.html)

## I've decided to improve speed of PBD simulation

I've done so by writing small pbd library in c++ and compiling it for web assembly (WASM). Results are "dosta dobri." Implementation in javascript is running on my pc with ~10fps. WASM implementation is running on my pc with ~60fps.


## You can test for yourself:

* [Js implementation](https://matthias-research.github.io/pages/challenges/bodyChainPBD.html)
* [Wasm implementation](https://ricko.us.to/PBDChallange)

## How to build

### Prerequisite

* Linux (I don't think it will work on Windows/Mac (prove me wrong ;]))
* cmake
* make
* git
* [emsdk](https://github.com/emscripten-core/emsdk)
* gml (OpenGL math library)

### Build && Run

* ```git clone https://github.com/branc116/PBDChallange```
* ```cd PBDChallange```
* ```mkdir build```
* ```cd build```
* ```cmake ../src``` 
* ```cmake-gui```
* Edit variable ADITIONAL such that it's value points to the folder where your **glm folder** is located.
* Press configure.
* Exit cmake-guid
* ```make```
* ```cd .. && serve```

**Serve** is some tool that hosts your current directory on a local http server. I'm using ```npm install -g serve```, but one can use what one wants.

If you don't want to build it yourself, you can just download [prebuilt version of wasm and js files](https://github.com/branc116/PBDChallange/releases/tag/1.0.0)

### Looking at a simulation

When you are hosting a server, you can go to localhost:5000 and you will see the PBD simulation of 100 boxes connected by joints. You can interact with them by dragging them (It's fun.)

It's is possible to tweek some parameters of the simulation in index.html file.


## File descriptions

* index.html - scene description created by Matthias with slight modification for making it consume wasam.
* src/Body.cpp - Implementation of body as described in [Original paper](https://matthias-research.github.io/pages/publications/PBDBodies.pdf)
* src/CMakeLists.txt - cmake make file to make c++ make Wasm files.
* src/Help.cpp - help me i need my head checked functions to help.
* src/Joint.cpp - Implementation of joint as described in [Original paper](https://matthias-research.github.io/pages/publications/PBDBodies.pdf).
* src/Pose.cpp - abstraction over position and rotation.
* src/Simulator.cpp - entry point to the simulator. One can add bodies and joint in to the simulator and take them out. One can run few steps of the simulator.
* src/Hello.hpp - declatraton of all the classes.
* src/Hello.idl - some file that defines the surfice of the library that is exposed to the javascript world. This file is used by WEBidl tool to generate glue javascript code.
* src/onload.js - some javascript (one must have some javascript file in ones repo.)








