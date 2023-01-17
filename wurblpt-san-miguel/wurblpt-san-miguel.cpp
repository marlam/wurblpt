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

/* This renders the San Miguel scene from https://casual-effects.com/data/ */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

void createScene(const std::string& fileName, Scene& scene)
{
    Transformation transformation;
    importIntoScene(scene, fileName, Transformation(),
            ImportBitDisableLightSources | ImportBitTwoSidedMaterials | ImportBitInvertedTf);
    //Texture* tex = scene.take(createTextureImage("delta_2_2k.hdr"));
    Texture* tex = scene.take(new TextureConstant(vec4(1.0f)));
    scene.take(new EnvironmentMapEquiRect(tex));
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <path/to/san-miguel.obj>\n", argv[0]);
        return 0;
    }

    MPICoordinator mpiCoordinator;

    bool preview = true;

    unsigned int width        = (preview ? 480 : 1920);
    unsigned int height       = (preview ? 270 : 1080);
    unsigned int samples_sqrt = (preview ?  20  :  45);

    Scene scene;
    createScene(argv[1], scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(45.0f), sensor.aspectRatio()));

    Transformation cameraTransformation(
            vec3(6.33516f, 0.600968f + 1.61f, -2.13264f), // translation
            toQuat(radians(-99.3793f), vec3(0.0f, 1.0f, 0.0f))); // rotation
    Camera camera(optics, cameraTransformation);

    Parameters parameters;
    //parameters.maxPathComponents = 5; // setting this breaks realistic glass

    scene.updateBVH();
    mcpt(mpiCoordinator, sensor, camera, scene, samples_sqrt, 0.0f, 0.0f, parameters);

    if (mpiCoordinator.mainProcess()) {
        TGD::save(sensor.result(), "image.exr");
#if 0
        GroundTruth gt = getGroundTruth(sensor, camera, scene);
        TGD::save(gt.worldSpacePositions, "groundtruth-world-pos.tgd");
        TGD::save(gt.worldSpaceNormals, "groundtruth-world-nrm.tgd");
        TGD::save(gt.cameraSpacePositions, "groundtruth-cam-pos.tgd");
#endif
    }

    return 0;
}
