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

/* This renders the Crytek Sponza in its OBJ form
 * from https://casual-effects.com/data/
 * The camera type is set via a command line argument:
 * 0: 2D
 * 1: 3D
 * 2: 180° 2D
 * 3: 180° 3D
 * 4: 360° 2D
 * 5: 360° 3D
 * no argument: preview quality 2D
 */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

void createScene(const std::string& sponzaFileName, const std::string& envmapFileName, Scene& scene)
{
    /* Imported scene */
    Transformation transformation(
            vec3(0.0f, 0.0f, 0.0f), // translation
            toQuat(radians(90.0f), vec3(0.0f, 1.0f, 0.0f)), // rotation
            vec3(0.01f)); // scaling
    importIntoScene(scene, sponzaFileName, transformation);

    /* Environment Map */
    Texture* tex = scene.take(createTextureImage(envmapFileName));
    scene.take(new EnvironmentMapEquiRect(tex));
#if 0
    Texture* lightTex = scene.take(new TextureConstant(vec4(6.0f)));
    scene.take(new EnvironmentMapEquiRect(lightTex));
#endif

    // Remarks about importance sampling for light sources:
    // - Enabling importance sampling of the environment map does not help since
    //   most points cannot see it, and those that do get enough light anyway.
    // - marking the directly lit ground triangles as HotSpots does not
    //   help either, probably because only a small part of the ground
    //   area is actually lit; most is in shadow...
    // This means we have importance sampling only for materials, which makes
    // the Sponza scene slow to render: you need lots of samples.
}

int main(int argc, char* argv[])
{
    if (argc != 3 && argc != 4) {
        fprintf(stderr, "Usage: %s <path/to/sponza> <path/to/envmap> [configuration]\n", argv[0]);
        return 0;
    }
    std::string sponzaFileName = argv[1];
    std::string envmapFileName = argv[2];
    int configuration = (argc == 4 ? atoi(argv[3]) : -1);

    unsigned int width;
    unsigned int height;
    Camera::SurroundMode surroundMode = Camera::Surround_Off;
    float stereoscopicDistance = 0.0f;
    unsigned int samples_sqrt = 100;
    std::string marker;
    Parameters parameters;
    switch (configuration) {
    case 0: // 2d
        width = 3840;
        height = 2160;
        break;
    case 1: // 3d
        width = 3840;
        height = 2160 * 2;
        stereoscopicDistance = 0.07f;
        marker = "-tb";
        break;
    case 2: // 180° 2d
        width = 2160;
        height = 2160;
        surroundMode = Camera::Surround_180;
        marker = "-180";
        break;
    case 3: // 180° 3d
        width = 2160;
        height = 2160 * 2;
        surroundMode = Camera::Surround_180;
        stereoscopicDistance = 0.07f;
        marker = "-180-tb";
        break;
    case 4: // 360° 2d
        width = 4320;
        height = 2160;
        surroundMode = Camera::Surround_360;
        marker = "-360";
        break;
    case 5: // 360° 3d
        width = 4320;
        height = 2160 * 2;
        surroundMode = Camera::Surround_360;
        stereoscopicDistance = 0.07f;
        marker = "-360-tb";
        break;
    default: // preview 2d
        width = 960;
        height = 540;
        samples_sqrt = 20;
        // This results in a biased rendering, but is much faster:
        parameters.maxPathComponents = 5; // 5 == up to 3 indirect bounces
        parameters.rrThreshold = 0.0f;
        // However, this does not work with a glass object in the
        // scene since rays passing through that need several more bounces!
        break;
    }

    Scene scene;
    createScene(sponzaFileName, envmapFileName, scene);

    SensorRGB sensor(width, height);

#if 1
    Optics optics(Projection(radians(70.0f), (stereoscopicDistance > 0.0f ? 2.0f : 1.0f) * sensor.aspectRatio()));
    vec3 lookfrom(0.0f, 1.7f, 0.0f);
    vec3 lookat(0.0f, 1.7f, -1.0f);
    Camera camera(surroundMode, stereoscopicDistance, optics, Transformation::fromLookAt(lookfrom, lookat));
#else
    // Plant closeup
    Optics optics(Projection(radians(30.0f), sensor.aspectRatio()));
    Transformation cameraTransformation(
            vec3(-0.93f, 1.7f, -1.82f),
            toQuat(radians(34.79f), vec3(-0.0877701f, -0.995764f, -0.0273793f)));
    Camera camera(optics, cameraTransformation);
#endif

    scene.updateBVH();
    mcpt(sensor, camera, scene, samples_sqrt, 0.0f, 0.0f, parameters);

    const TGD::Array<float>& hdrImg = sensor.result();
    TGD::Array<float> ldrImg = uniformRationalQuantization(hdrImg, maxLuminance(hdrImg), 16.0f);
    TGD::Array<uint8_t> srgbImg = toSRGB(ldrImg);

    TGD::save(hdrImg, "image" + marker + ".exr");
    TGD::save(srgbImg, "image" + marker + ".png");

    return 0;
}
