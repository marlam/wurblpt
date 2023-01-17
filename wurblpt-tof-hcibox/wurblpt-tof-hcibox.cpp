/*
 * Copyright (C) 2019, 2020, 2021, 2022
 * Computer Graphics Group, University of Siegen (written by Martin Lambers)
 * Copyright (C) 2022, 2023
 * Martin Lambers <marlam@marlam.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* This simulates a Time-of-Flight distance looking into the HCI Box from
 * https://hci.iwr.uni-heidelberg.de/benchmarks/The_HCIBOX_Depth_Evaluation_Dataset
 * You need to download the box in OBJ format (and also the the material MTL file). */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s </path/to/sceneDescription.obj>\n", argv[0]);
        return 1;
    }

    Scene scene;
    importIntoScene(scene, argv[1], Transformation(), ImportBitInvertedTf);

    SensorTofAmcw sensor(320, 200);
    sensor.pauseTime = 36000.0f;
    float shotNoiseFactor = 1.0f;

    Optics optics(Projection(radians(38.4f), sensor.aspectRatio()));

    Transformation cameraTransformation;
    cameraTransformation.translation = vec3(4.4f, 3.2f, -0.54f);
    cameraTransformation.rotation = toQuat(radians(96.133f), vec3(-0.332739f, 0.884304f, 0.327553f));
    cameraTransformation.translation += cameraTransformation.rotation * vec3(-0.15f, -0.05f, -4.0f);
    Camera camera(optics, cameraTransformation);
    vec3 camLookFrom = cameraTransformation.lookFrom();
    vec3 camLookAt = cameraTransformation.lookAt();
    vec3 camUp = cameraTransformation.up();
    fprintf(stderr, "lookFrom = %g, %g, %g\n", camLookFrom.x(), camLookFrom.y(), camLookFrom.z());
    fprintf(stderr, "lookAt = %g, %g, %g\n", camLookAt.x(), camLookAt.y(), camLookAt.z());
    fprintf(stderr, "up = %g, %g, %g\n", camUp.x(), camUp.y(), camUp.z());

    Material* lightFrontSide = scene.take(new LightTof(40.0f / (4.0f * pi), radians(120.0f)));
    Material* lightBackSide = scene.take(new MaterialLambertian(vec4(0.0f)));
    Material* lightMaterial = scene.take(new MaterialTwoSided(lightFrontSide, lightBackSide));
    Transformation lightTransformation(
            vec3(0.0f),
            toQuat(radians(180.0f), vec3(1.0f, 0.0f, 0.0f)),
            vec3(.10f, 0.05f, 1.0f));
    lightTransformation = cameraTransformation * lightTransformation;
    scene.take(new MeshInstance(scene.take(generateQuad(lightTransformation)), lightMaterial), HotSpot);

    scene.updateBVH(0.0f, sensor.frameDuration());
    unsigned int samples_sqrt = 10;
    TGD::Array<float> tofPhaseImages[sensor.phaseImageCount];
    for (unsigned int phaseImage = 0; phaseImage < sensor.phaseImageCount; phaseImage++) {
        float t0 = phaseImage * sensor.phaseImageDuration();
        float t1 = t0 + sensor.exposureTime / 1e6f;
        sensor.setPhaseIndex(phaseImage);
        mcpt(sensor, camera, scene, samples_sqrt, t0, t1);
        tofPhaseImages[phaseImage] = sensor.phase(shotNoiseFactor);
    }
    TGD::save(tofPhaseImages[0], "tof-phase0.tgd");
    TGD::save(tofPhaseImages[1], "tof-phase1.tgd");
    TGD::save(tofPhaseImages[2], "tof-phase2.tgd");
    TGD::save(tofPhaseImages[3], "tof-phase3.tgd");
    TGD::Array<float> tofResult = sensor.result(tofPhaseImages);
    TGD::save(extractComponent(tofResult, 0), "tof-distance.tgd");
    TGD::save(extractComponent(tofResult, 1), "tof-amplitude.tgd");
    TGD::save(extractComponent(tofResult, 2), "tof-intensity.tgd");
    TGD::Array<float> tofCoords = cameraSpaceToPMDSpace(cameraSpaceDistanceToCoords(tofResult, optics.projection, optics.distortion));
    TGD::save(tofCoords, "tof-coordinates.tgd");

    float gtTime = 0.0f;
    GroundTruth gt = getGroundTruth(sensor, camera, scene, gtTime);
    TGD::save(gt.cameraSpaceDistances, "groundtruth-distance.tgd");
    TGD::save(gt.cameraSpacePositions, "groundtruth-coordinates.tgd");

    return 0;
}
