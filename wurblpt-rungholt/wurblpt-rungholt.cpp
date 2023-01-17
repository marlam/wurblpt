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

/* This renders the Rungholt scene from https://casual-effects.com/data/ */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

void createScene(const std::string& rungholtFileName, Scene& scene)
{
    Transformation transformation(
            vec3(0.0f, 0.0f, 0.0f), // translation
            quat::null(), // rotation
            vec3(0.05f)); // scaling
    importIntoScene(scene, rungholtFileName, transformation, ImportBitInvertedTf);

    scene.take(new MeshInstance(scene.take(generateQuad(
                Transformation(vec3(0.0f, 50.0f, 0.0f),
                    toQuat(radians(90.0f), vec3(1.0f, 0.0f, 0.0f)),
                    vec3(50.0f)))),
                scene.take(new LightDiffuse(vec3(6.0f)))),
            HotSpot);
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <path/to/rungholt>\n", argv[0]);
        return 0;
    }

    bool preview = true;

    unsigned int width        = 15360;
    unsigned int height       = 4320;
    unsigned int samples_sqrt = 30;
    if (preview) {
        width /= 4;
        height /= 4;
        samples_sqrt = 8;
    }

    Scene scene;
    createScene(argv[1], scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(35.0f), sensor.aspectRatio()));

    Transformation cameraTransformation(vec3(25.0f, 6.0f, -1.0f), toQuat(radians(vec3(-20.0f, 90.0f, 0.0f))));
    Camera camera(optics, cameraTransformation);

    scene.updateBVH();
    mcpt(sensor, camera, scene, samples_sqrt);
    const TGD::Array<float>& hdrImg = sensor.result();
    TGD::Array<uint8_t> ldrImg = toSRGB(hdrImg);

    //TGD::save(hdrImg, "image.exr");
    TGD::save(ldrImg, "image.png");

#if 0
    GroundTruth gt = getGroundTruth(sensor, camera, scene);
    TGD::save(gt.worldSpacePositions, "gt-world-space-positions.gta");
    TGD::save(gt.worldSpaceNormals, "gt-world-space-normals.gta");
    TGD::save(gt.cameraSpacePositions, "gt-camera-space-positions.gta");
    TGD::save(gt.cameraSpaceDepths, "gt-camera-space-depths.gta");
    TGD::save(gt.cameraSpaceDistances, "gt-camera-space-distances.gta");
#endif

    return 0;
}
