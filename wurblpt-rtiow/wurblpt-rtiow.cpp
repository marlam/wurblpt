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

/* This is the scene from "Ray Tracing In One Weekend" by Peter Shirley.
 * It can optionally be rendered as a stereoscopic 3D image and/or as
 * a 180° or 360° surround image. */

#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>

using namespace WurblPT;


void createRandomScene(Scene& scene)
{
    Prng prng(17);

    Material* material;

    material = scene.take(new MaterialLambertian(vec3(0.5f)));
    scene.take(new Sphere(vec3(0.0f, -1000.0f, 0.0f), 1000.0f, material));

    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            float chooseMaterial = prng.in01();
            vec3 center(a + 0.9f * prng.in01(), 0.2f, b + 0.9f * prng.in01());
            if (length(center - vec3(4.0f, 0.2f, 0.0f)) > 0.9f) {
                if (chooseMaterial < 0.8f) { // diffuse
                    material = scene.take(new MaterialLambertian(vec3(
                                    prng.in01() * prng.in01(),
                                    prng.in01() * prng.in01(),
                                    prng.in01() * prng.in01())));
                    scene.take(new Sphere(center, 0.2f, material));
                } else if (chooseMaterial < 0.95f) { // metal
                    material = scene.take(new MaterialGGX(
                                vec3(0.5f * (1.0f + prng.in01()),
                                     0.5f * (1.0f + prng.in01()),
                                     0.5f * (1.0f + prng.in01())),
                                vec2(0.001f + 0.499f * prng.in01())));
                    scene.take(new Sphere(center, 0.2f, material));
                } else { // glass
                    material = scene.take(new MaterialGlass(MaterialGlass::transparentColorToAbsorption(vec3(1.0f)), 1.5f));
                    scene.take(new Sphere(center, 0.2f, material));
                }
            }
        }
    }

    material = scene.take(new MaterialGlass(MaterialGlass::transparentColorToAbsorption(vec3(1.0f)), 1.5f));
    scene.take(new Sphere(vec3(0.0f, 1.0f, 0.0f), 1.0f, material));
    material = scene.take(new MaterialLambertian(vec3(0.4f, 0.2f, 0.1f)));
    scene.take(new Sphere(vec3(-4.0f, 1.0f, 0.0f), 1.0f, material));
    material = scene.take(new MaterialGGX(vec3(0.7f, 0.6f, 0.5f), vec2(0.001f)));
    scene.take(new Sphere(vec3(4.0f, 1.0f, 0.0f), 1.0f, material));

    material = scene.take(new LightDiffuse(vec3(1.0f)));
    material = scene.take(new MaterialTwoSided(nullptr, material));
    scene.take(new Sphere(vec3(0.0f), 2000.0f, material));
}

int main(void)
{
    unsigned int width = 800;
    unsigned int height = 600;
    unsigned int samples_sqrt = 7;
    Camera::SurroundMode surroundMode = Camera::Surround_360;
    bool stereoscopic = false;

    Scene scene;
    createRandomScene(scene);

    if (surroundMode == Camera::Surround_180) {
        width = height;
        width *= 2;
        height *= 2;
    } else if (surroundMode == Camera::Surround_360) {
        width = 2 * height;
        width *= 2;
        height *= 2;
    }
    float aspectRatio = float(width) / height;
    if (stereoscopic) {
        height *= 2;
    }
    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(30.0f), aspectRatio), LensDistortion(), LensDepthOfField(0.06f, 7.0f));

    vec3 lookfrom(10.0f, 2.0f, 3.0f);
    vec3 lookat(0.0f, 0.0f, -1.0f);
    Camera camera(surroundMode, stereoscopic ? 0.08f : 0.0f, optics, Transformation::fromLookAt(lookfrom, lookat));

    Parameters params;
    params.minHitDistance = 0.00035;

    scene.updateBVH();
    mcpt(sensor, camera, scene, samples_sqrt, 0.0f, 0.0f, params);
    TGD::Array<uint8_t> ldrImg = toSRGB(sensor.result());

    std::string filename = "image";
    if (surroundMode == Camera::Surround_180)
        filename += "-180";
    else if (surroundMode == Camera::Surround_360)
        filename += "-360";
    if (stereoscopic)
        filename += "-tb";
    else
        filename += "-2d";
    TGD::save(ldrImg, filename + ".png");
    return 0;
}
