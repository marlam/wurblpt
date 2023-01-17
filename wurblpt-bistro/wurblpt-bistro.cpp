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

/* This renders the Amazon Lumberyard Bistro scene in its OBJ form
 * from https://casual-effects.com/data/ */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>

using namespace WurblPT;


void createScene(const std::string& bistroPathName, bool inside, Scene& scene)
{
    Transformation transformation(
            vec3(0.0f, 0.0f, 0.0f), // translation
            toQuat(radians(0.0f), vec3(0.0f, 1.0f, 0.0f)), // rotation
            vec3(0.01f)); // scaling
    if (inside) {
        importIntoScene(scene, bistroPathName + "/Interior/interior.obj", transformation, ImportBitDisableHotSpots);
    } else {
        //Texture* tex = scene.take(new TextureConstant(vec4(1.0f)));
        //scene.take(new EnvironmentMapEquiRect(tex));
        importIntoScene(scene, bistroPathName + "/Exterior/exterior.obj", transformation, ImportBitDisableHotSpots);
    }
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <path/to/bistro>\n", argv[0]);
        return 0;
    }

    MPICoordinator mpiCoordinator;

    bool preview = true;
    bool inside = false;

    unsigned int width        = (preview ? 480 : 1920);
    unsigned int height       = (preview ? 270 : 1080);
    unsigned int samples_sqrt = (preview ? 20  :   40);

    Scene scene;
    createScene(argv[1], inside, scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(45.0f), sensor.aspectRatio()));

    Transformation cameraTransformationExterior(
            vec3(-13.8f, 1.8f, -3.3f), // translation
            toQuat(radians(-102.0f), vec3(0.0f, 1.0f, 0.0f))); // rotation
    Transformation cameraTransformationInterior(
            vec3(0.663f, 2.0f, -2.06f), // translation
            toQuat(radians(-89.35f), vec3(0.0f, 1.0f, 0.0f))); // rotation
    Camera camera(optics,
            inside ? cameraTransformationInterior : cameraTransformationExterior);

    scene.updateBVH();
    mcpt(mpiCoordinator, sensor, camera, scene, samples_sqrt);
    if (mpiCoordinator.mainProcess()) {
        TGD::save(sensor.result(), inside ? "image-interior.exr" : "image-exterior.exr");
#if 0
        GroundTruth gt = getGroundTruth(sensor, camera, scene);
        //TGD::save(gt.worldSpacePositions, "groundtruth-world-pos.tgd");
        //TGD::save(gt.worldSpaceNormals, "groundtruth-world-nrm.tgd");
        //TGD::save(gt.cameraSpacePositions, "groundtruth-cam-pos.tgd");
        TGD::save(gt.materials, "materials.tgd");
        FILE* f = fopen("material-names.txt", "w");
        for (size_t i = 0; i < scene.materialNames().size(); i++)
            fprintf(f, "%zu %s\n", i, scene.materialNames()[i].c_str());
        fclose(f);
#endif
    }

    return 0;
}
