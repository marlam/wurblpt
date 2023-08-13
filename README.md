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


# Video Encoding and Metadata

Use the AV1 codec with the mp4 container format. Example for high quality encoding:
```
ffmpeg -i input-%04d-360.png -vf format=yuv420p -c:v libaom-av1 -crf 30 -g 125 output-360.mp4
```

For 360° video, set the appropriate metadata
[defined by Google](https://github.com/google/spatial-media/blob/master/docs/spherical-video-rfc.md)
and understood by [VLC](https://www.videolan.org/vlc/).
```
exiftool \
	-XMP-GSpherical:Spherical="true" \
	-XMP-GSpherical:Stitched="true" \
	-XMP-GSpherical:StitchingSoftware="WurblPT" \
	-XMP-GSpherical:ProjectionType="equirectangular" \
	output-360.mp4
```
In the case of stereoscopic 360° video, add `-XMP-GSpherical:StereoMode="top-bottom"`.

Note that [Bino](https://bino3d.org) does not yet support this metadata because
of QtMultimedia limitations; that's why it uses [file name conventions](https://bino3d.org/bino-manual.html#file-name-conventions).

With WurblPT, append the following marker to the file name just before the extension so that Bino detects the correct format:
- Conventional 2D: no marker; example: `image.png`
- Conventional 3D: marker `-tb`; example: `image-tb.png`
- 180° 2D: marker `-180`; example: `image-180.png`
- 180° 3D: marker `-180-tb`; example: `image-180-tb.png`
- 360° 2D: marker `-360`; example: `image-360.png`
- 360° 3D: marker `-360-tb`; example: `image-360-tb.png`


# Creating and Converting Conventional and Surround Output

WurblPT includes tools that can
- extract 180° output from 360° input, both 3D and 2D (`wurblpt-360-to-180`)
- extract the left view from a 3D input to create a 2D output (`wurblpt-stereo-to-mono`)
- render 2D views from 360° input (`wurblpt-360-to-conventional`)

If you want to create all six forms (conventional, 180°, 360°, each in 2D and 3D) with minimal computational costs, do the following:
- Render 360° 3D, and extract 180° 3D from it with `wurblpt-360-to-180`.
  This is exactly what you would get when rendering 180° 3D directly.
- Render 360° 2D, and extract 180° 2D from it with `wurblpt-360-to-180`.
  This is exactly what you would get when rendering 180° 2D directly.
- Render conventional 3D, and extract conventional 2D from it with `wurblpt-stereo-to-mono`.
  This is almost exactly what you would get when rendering conventional 2D
  directly, but the 2D camera will be positioned at the left eye and not at the
  center between the eyes, which is a difference of half the stereoscopic eye
  distance, typically 3.5cm, which should not be noticable for most scenes.

Notes:
- You can extract one view from 360° 3D to get 360° 2D with `wurblpt-stereo-to-mono`,
  but the result is typically not what you want due to
  [moving eye centers when rendering 360° 3D](https://developers.google.com/static/vr/jump/rendering-ods-content.pdf).
- If you have 360° (either 2D or 3D), you can get conventional 2D or 3D cheaply
  with `wurblpt-360-to-conventional`. This renders the conventional view in the same
  way a surround video player such as [Bino](https://bino3d.org) or
  [VLC](https://www.videolan.org/vlc/) does.
  However the output should not have more than half the resolution of the
  360° input; more does not make sense since details vanish.
