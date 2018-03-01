# libmotioncapture
Interface Abstraction for Motion Capture System APIs such as VICON, OptiTrack, PhaseSpace, or Qualisys.

## Setup

For legal reasons we are not allowed to include the VICON DataStream SDK in this repository, which is used to capture data from a VICON motion capture system. Please download the SDK (version 1.7.1) from http://www.vicon.com and place the following files in `externalDependencies/vicon_sdk` (that is, from the Linux64-boost-1.58.0 folder):

```
├── include
│   └── vicon_sdk
│       └── DataStreamClient.h
└── lib64
    ├── libboost_system-mt.so.1.58.0
    ├── libboost_thread-mt.so.1.58.0
    └── libViconDataStreamSDK_CPP.so

```
