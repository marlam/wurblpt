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

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

void createScene(Scene& scene)
{
    Material* white = scene.take(new MaterialLambertian(vec3(0.725f, 0.71f, 0.68f))); // "white"
    Material* red = scene.take(new MaterialLambertian(vec3(0.63f, 0.065f, 0.05f))); // "red"
    Material* green = scene.take(new MaterialLambertian(vec3(0.14f, 0.45f, 0.091f))); // "green"

    Material* light = scene.take(new MaterialTwoSided(
                scene.take(new LightDiffuse(vec3(4.0f))),
                scene.take(new MaterialLambertian(vec3(0.0f)))));

#if 0
    Material* metal = scene.take(new MaterialMetal(0.1f, vec3(1.0f)));
    Material* ggx = scene.take(new MaterialGGX(vec3(1.0f), 0.1f));
    Material* glass = scene.take(new MaterialGlass(1.5f));
#endif

    Material* tallBoxMaterial = white;

    // left wall
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-1.01f, 0.0f, 0.99f), vec3(-0.99f, 0.0f, -1.04f), vec3(-1.02f, 1.99f, -1.04f), vec3(-1.02f, 1.99f, 0.99f) },
                { vec3(0.9999874f, 0.005025057f, 0.0f), vec3(0.9998379f, 0.01507292f, 0.009850611f), vec3(0.9999874f, 0.005025057f, 0.0f), vec3(0.9999874f, 0.005025057f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                red));
    // right wall
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(1.0f, 0.0f, -1.04f), vec3(1.0f, 0.0f, 0.99f), vec3(1.0f, 1.99f, 0.99f), vec3(1.0f, 1.99f, -1.04f) },
                { vec3(-1.0f, 0.0f, 0.0f), vec3(-1.0f, 0.0f, 0.0f), vec3(-1.0f, 0.0f, 0.0f), vec3(-1.0f, 0.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                green));
    // floor
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-1.01f, 0.0f, 0.99f), vec3(1.0f, 0.0f, 0.99f), vec3(1.0f, 0.0f, -1.04f), vec3(-0.99f, 0.0f, -1.04f) },
                { vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
    // ceiling
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-1.02f, 1.99f, 0.99f), vec3(-1.02f, 1.99f, -1.04f), vec3(1.0f, 1.99f, -1.04f), vec3(1.0f, 1.99f, 0.99f) },
                { vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
    // back wall
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.99f, 0.0f, -1.04f), vec3(1.0f, 0.0f, -1.04f), vec3(1.0f, 1.99f, -1.04f), vec3(-1.02f, 1.99f, -1.04f) },
                { vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, 1.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));

#if 0
    // short box left
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.05f, 0.0f, 0.57f), vec3(-0.05f, 0.6f, 0.57f), vec3(0.13f, 0.6f, 0.0f), vec3(0.13f, 0.0f, 0.0f) },
                { vec3(-0.9535826f, 0.0f, -0.3011314f), vec3(-0.9535826f, 0.0f, -0.3011314f), vec3(-0.9535826f, 0.0f, -0.3011314f), vec3(-0.9535826f, 0.0f, -0.3011314f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
    // short box right
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(0.7f, 0.0f, 0.17f), vec3(0.7f, 0.6f, 0.17f), vec3(0.53f, 0.6f, 0.75f), vec3(0.53f, 0.0f, 0.75f) },
                { vec3(0.9596285f, 0.0f, 0.2812705f), vec3(0.9596285f, 0.0f, 0.2812705f), vec3(0.9596285f, 0.0f, 0.2812705f), vec3(0.9596285f, 0.0f, 0.2812705f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
    // short box floor
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(0.53f, 0.0f, 0.75f), vec3(0.7f, 0.0f, 0.17f), vec3(0.13f, 0.0f, 0.0f), vec3(-0.05f, 0.0f, 0.57f) },
                { vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
    // short box ceiling
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(0.53f, 0.6f, 0.75f), vec3(0.7f, 0.6f, 0.17f), vec3(0.13f, 0.6f, 0.0f), vec3(-0.05f, 0.6f, 0.57f) },
                { vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
    // short box back
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(0.13f, 0.0f, 0.0f), vec3(0.13f, 0.6f, 0.0f), vec3(0.7f, 0.6f, 0.17f), vec3(0.7f, 0.0f, 0.17f) },
                { vec3(0.2858051f, 0.0f, -0.9582878f), vec3(0.2858051f, 0.0f, -0.9582878f), vec3(0.2858051f, 0.0f, -0.9582878f), vec3(0.2858051f, 0.0f, -0.9582878f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
    // short box front
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(0.53f, 0.0f, 0.75f), vec3(0.53f, 0.6f, 0.75f), vec3(-0.05f, 0.6f, 0.57f), vec3(-0.05f, 0.0f, 0.57f) },
                { vec3(-0.2963993f, 0.0f, 0.9550642f), vec3(-0.2963993f, 0.0f, 0.9550642f), vec3(-0.2963993f, 0.0f, 0.9550642f), vec3(-0.2963993f, 0.0f, 0.9550642f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                white));
#else
    std::vector<const Hitable*> shortBoundary = scene.take(new Sphere(vec3(0.33f, 0.3f, 0.37f), 0.3f,
                scene.take(new MaterialGlass(MaterialGlass::transparentColorToAbsorption(vec3(1.0f, 1.0f, 1.0f)), 1.5f))));
    Material* shortPhaseFunc = scene.take(new MaterialPhaseFunctionIsotropic(vec3(0.2f, 0.4f, 0.9f)));
    scene.take(new Medium(shortBoundary, 8.0f, shortPhaseFunc));
#endif

#if 0
    // tall box left
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.53f, 0.0f, 0.09f), vec3(-0.53f, 1.2f, 0.09f), vec3(-0.71f, 1.2f, -0.49f), vec3(-0.71f, 0.0f, -0.49f) },
                { vec3(-0.9550642f, 0.0f, 0.2963992f), vec3(-0.9550642f, 0.0f, 0.2963992f), vec3(-0.9550642f, 0.0f, 0.2963992f), vec3(-0.9550642f, 0.0f, 0.2963992f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                tallBoxMaterial));
    // tall box right
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.14f, 0.0f, -0.67f), vec3(-0.14f, 1.2f, -0.67f), vec3(0.04f, 1.2f, -0.09f), vec3(0.04f, 0.0f, -0.09f) },
                { vec3(0.9550642f, 0.0f, -0.2963992f), vec3(0.9550642f, 0.0f, -0.2963992f), vec3(0.9550642f, 0.0f, -0.2963992f), vec3(0.9550642f, 0.0f, -0.2963992f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                tallBoxMaterial));
    // tall box floor
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.53f, 0.0f, 0.09f), vec3(0.04f, 0.0f, -0.09f), vec3(-0.14f, 0.0f, -0.67f), vec3(-0.71f, 0.0f, -0.49f) },
                { vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                tallBoxMaterial));
    // tall box ceiling
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.53f, 1.2f, 0.09f), vec3(0.04f, 1.2f, -0.09f), vec3(-0.14f, 1.2f, -0.67f), vec3(-0.71f, 1.2f, -0.49f) },
                { vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                tallBoxMaterial));
    // tall box back
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.71f, 0.0f, -0.49f), vec3(-0.71f, 1.2f, -0.49f), vec3(-0.14f, 1.2f, -0.67f), vec3(-0.14f, 0.0f, -0.67f) },
                { vec3(-0.3011314f, 0.0f, -0.9535826f), vec3(-0.3011314f, 0.0f, -0.9535826f), vec3(-0.3011314f, 0.0f, -0.9535826f), vec3(-0.3011314f, 0.0f, -0.9535826f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                tallBoxMaterial));
    // tall box front
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(0.04f, 0.0f, -0.09f), vec3(0.04f, 1.2f, -0.09f), vec3(-0.53f, 1.2f, 0.09f), vec3(-0.53f, 0.0f, 0.09f) },
                { vec3(0.3011314f, 0.0f, 0.9535826f), vec3(0.3011314f, 0.0f, 0.9535826f), vec3(0.3011314f, 0.0f, 0.9535826f), vec3(0.3011314f, 0.0f, 0.9535826f) },
                { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                tallBoxMaterial));
#else
    //Hitable* tallBoundary = scene.takeUnhitable( new HitableSphere<Static>(vec3(-0.345f, 0.8f, -0.29f), 0.4f, nullptr));
    Transformation tallBoundaryTransformation(vec3(-0.345f, 0.8f, -0.29f), toQuat(radians(160.0f), vec3(0.0f, 1.0f, 0.0f)), vec3(0.5f));
#if 1
    //std::vector<const Hitable*> tallBoundaryHitables = scene.takeUnhitable(new MeshInstance(scene.take(generateBunny(tallBoundaryTransformation)), nullptr));
    std::vector<const Hitable*> tallBoundaryHitables = scene.takeUnhitable(new MeshInstance(scene.take(generateIcosahedron(tallBoundaryTransformation)), nullptr));
    Material* tallPhaseFunc = scene.take(new MaterialPhaseFunctionIsotropic(vec3(0.8f, 0.8f, 1.0f)));
    scene.take(new Medium(tallBoundaryHitables, 250.0f, tallPhaseFunc));
#else
    scene.take(new MeshInstance(scene.take(new Mesh(generateTeapot(tallBoundaryTransformation, scene.take(new MaterialLambertian(vec3(0.8f, 0.8f, 1.0f))))));
#endif
#endif

    // light source
#if 1
    // original Cornell Box
    const float light_y = 1.98f;
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.24f, light_y, 0.16f), vec3(-0.24f, light_y, -0.22f), vec3(0.23f, light_y, -0.22f), vec3(0.23f, light_y, 0.16f) },
                { vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f) },
                { vec2(0.0f, 1.0f), vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                light),
        HotSpot);
#else
    // larger to reduce noise
    const float light_x_min = -0.4f;
    const float light_x_max = 0.4f;
    const float light_y = 1.98f;
    const float light_z_min = -0.4f;
    const float light_z_max = 0.4f;
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(light_x_min, light_y, light_z_max), vec3(light_x_min, light_y, light_z_min), vec3(light_x_max, light_y, light_z_min), vec3(light_x_max, light_y, light_z_max) },
                { vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f) },
                { vec2(0.0f, 1.0f), vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                light),
        HotSpot);
#endif
}

int main(void)
{
    bool preview = true;

    unsigned int width        = (preview ? 500 : 2048);
    unsigned int height       = (preview ? 500 : 2048);
    unsigned int samples_sqrt = (preview ? 25 : 100);

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(50.0f), sensor.aspectRatio()));

    vec3 lookfrom(0.0f, 1.0f, 3.2f);
    vec3 lookat(0.0f, 1.0f, -1.0f);
    vec3 up(0.0f, 1.0f, 0.0f);
    Camera camera(optics, Transformation::fromLookAt(lookfrom, lookat, up));

    scene.updateBVH();
    mcpt(sensor, camera, scene, samples_sqrt);

    TGD::save(sensor.result(), "image.exr");

    return 0;
}
