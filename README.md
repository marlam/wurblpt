# wurblpt

Wurblpt is an experimental path tracer with the following features:
- Camera modes for surround images (360° and 180°) and stereoscopic 3D
- Camera intrinsics and lens distortion compatible to OpenCV
- Support for RGB sensors and Time-of-Flight distance sensors
- Animation of everything in the scene (geometry, textures, cameras, ...)
- Ground Truth generation including geometry, materials, optical flow
- Light travel distances for each path for precise simulation of
  Time-of-Flight sensors
- Support for rendering on HPC clusters (pure OpenMP or mixed MPI/OpenMP)
- Support for measured materials from the RGL material database
- Import of OBJ/MTL scenes
- Export to OBJ/MTL including snapshots of animated scenes and procedural
  textures


# Requirements

Only [libtgd](https://marlam.de/tgd/) is required, nothing else. Furthermore,
for use in wurblpt, libtgd does not need any external libraries.


# Usage

First build and install libwurblpt, then build some of the included example
applications, depending on your interest.

Write a simple application (often less than 100 lines) that uses libwurblpt.
See the available examples.

You should activate all compiler optimizations, and enable OpenMP.

For use on HPC clusters:
- When rendering videos (i.e. series of frames), use array jobs to let each
  invocation of your application render one frame.
- When rendering at high resolution and/or high quality, you can distribute
  rendering of a single frame to multiple nodes. For this purpose, simply build
  your application with OpenMPI, and define `WURBLPT_WITH_MPI` when compiling.
  See e.g. the wurblpt-stagelights example for how to use the MPICoordinator
  class (3 lines of code). The frame will automatically be subdivided into
  blocks which will be rendered by the MPI ranks with dynamic load balancing.
  Launch one MPI rank per node and let OpenMP use all the cores in the node.
  Make sure that MPI binding does not get in the way of OpenMP, e.g. by using
  `mpirun --bind-to none`.
