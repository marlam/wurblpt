# WurblPT

WurblPT is an experimental path tracer with the following features:
- Camera modes for surround images (360° and 180°) and stereoscopic 3D
- Camera intrinsics and lens distortion compatible to OpenCV
- Support for RGB sensors and Time-of-Flight distance sensors
- Animation of everything in the scene (geometry, textures, cameras, ...)
- Ground Truth generation including geometry, materials, optical flow
- Light travel distances for each path for precise simulation of
  Time-of-Flight sensors
- Support for rendering on HPC clusters (pure OpenMP or mixed MPI/OpenMP)
- Support for measured materials from the [RGL material database](https://rgl.epfl.ch/materials)
- Import of OBJ/MTL scenes
- Export to OBJ/MTL including snapshots of animated scenes and procedural
  textures


# Requirements

Only [libtgd](https://marlam.de/tgd/) is required, nothing else. Furthermore,
for use in WurblPT, libtgd does not need any external libraries.


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

# Surround Video and 3D Video

Use the AV1 codec if the target systems support hardware accelerated playback,
otherwise fall back to H265. In both cases, use the mp4 container format.
Examples for high quality encoding:
```
ffmpeg -i input-%04d-360.png -vf format=yuv420p -c:v libaom-av1 -crf 30 -g 125 output-360.mp4
ffmpeg -i input-%04d-360.png -vf format=yuv420p -c:v libx265 -preset veryslow -crf 20 output-360.mp4
```

For 360° video, set the appropriate metadata
[defined by Google](https://github.com/google/spatial-media/blob/master/docs/spherical-video-rfc.md)
and understood by [VLC](https://www.videolan.org/vlc/).
Note that [Bino](https://bino3d.org) does not yet support this metadata because
of QtMultimedia limitations; that's why it uses file name conventions.
```
exiftool \
	-XMP-GSpherical:Spherical="true" \
	-XMP-GSpherical:Stitched="true" \
	-XMP-GSpherical:StitchingSoftware="WurblPT" \
	-XMP-GSpherical:ProjectionType="equirectangular" \
	output-360.mp4
```
In the case of stereoscopic 360 video, add `-XMP-GSpherical:StereoMode="top-bottom"`.

WurblPT includes tools that can
- extract 180° video from 360° video (both 3D and 2D),
- extract the left view from a 3D input to create a 2D output (works fine for conventional video,
  but for surround video this is not what you might expect because of the
  [moving eye centers when rendering 3D surround](https://developers.google.com/static/vr/jump/rendering-ods-content.pdf)),
  and
- render 2D views from 360° video (the same way that [Bino](https://bino3d.org) or
  [VLC](https://www.videolan.org/vlc/) do).

If you want to create all combinations with minimal computational costs, do the following:
- Render 360° 3D, and extract 180° 3D from it.
- Render 360° 2D, and extract 180° 2D from it.
  Do not extract one view from the 360° 3D to get 2D because this is not strictly the same.
- To get highest quality conventional video, render conventional 3D, and extract the left view to get conventional 2D.
- Alternatively, render conventional 3D from 360° and conventional 2D from 360° 2D, both
  with half the resolution of the 360° video (more does not make sense since details vanish).
