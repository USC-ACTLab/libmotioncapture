# libmotioncapture
Interface Abstraction for Motion Capture System APIs such as VICON, OptiTrack, PhaseSpace, or Qualisys.

## Compile options
By default, `libmotioncapture` supports the following hardware:

- VICON - SDK git submodule
- Qualisys - SDK git submodule
- OptiTrack - binary parsing over network (no dependency)

Support for following hardware requires additional action by the user:

- VRPN - set up `libvrpn` or `vrpn-client-ros` on the machine.
- Phasespace - manually obtain SDK and copy to `externalDependencies/phasespace_sdk/`.

After setup, enable the appropriate compile flags in `CMakeLists.txt`.
