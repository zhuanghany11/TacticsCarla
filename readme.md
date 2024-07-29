# This repo aims explore co-simulation between carla and tactics2d

The system use docker (ubuntu20.04, python 3.8) to build environment which contains two parts:
- a carla client at version 9.12 (Only cara client PythonAPI, no server)
- a tactics2d simulator at version 0.17

There are two targets:
1. realize carla-based agent in tactics2d. Spawn a vehicle by tactics2d, but the vehicle is controlled by carla, tactics2d is only to visualize and life-cycle management.
2. realize 2D BEV visualization of carla world using tactics2d. Xodr parser in tactics2d is used to visualize road while dynamic objects are visualized in tactics2d by fetching real-time object information from carla server.


# Something useful for understanding carla-client docker
https://github.com/unicorns/carla-client-docker

