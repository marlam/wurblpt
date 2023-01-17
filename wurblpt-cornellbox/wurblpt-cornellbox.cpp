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

class ObjectAnimation : public AnimationKeyframes
{
public:
    ObjectAnimation() : AnimationKeyframes(
            0.0f, Transformation(vec3(0.0f, 0.3f, 0.0f)),
            1.0f, Transformation(vec3(0.0f, 0.2f, 0.0f)))
    {
    }
};

void createScene(Scene& scene,
        int tallBoxMaterialType,        // 0 = white, 1 = metal
        int shortObjectType,            // 0 = box, 1 = sphere, 2 = bunny
        int shortObjectMaterialType,    // 0 = white, 1 = rgl, 2 = glass
        bool shortObjectMoving
        )
{
    Material* white = scene.take(new MaterialLambertian(vec3(0.725f, 0.71f, 0.68f))); // "white"
    Material* red = scene.take(new MaterialLambertian(vec3(0.63f, 0.065f, 0.05f))); // "red"
    Material* green = scene.take(new MaterialLambertian(vec3(0.14f, 0.45f, 0.091f))); // "green"
#if 1
    Material* light = scene.take(new LightDiffuse(vec3(4.0f)));
#elif 0
    Material* light = scene.take(new MaterialTwoSided(
                scene.take(new LightDiffuse(vec3(4.0f), scene.take(createTextureImage("world.jpg")))),
                scene.take(new MaterialLambertian(vec3(0.0f)))));
#else
    Material* light = scene.take(new MaterialTwoSided(
                scene.take(new LightSpot(radians(30.0f), vec3(4.0f))),
                scene.take(new MaterialLambertian(vec3(0.0f)))));
#endif

    Material* metal = scene.take(new MaterialGGX(vec3(1.0f), vec2(0.04f)));
    Material* rgl = shortObjectMaterialType == 1 ? scene.take(new MaterialRGL("cm_toxic_green_rgb.bsdf")) : nullptr;
    Material* glass = scene.take(new MaterialGlass(vec3(0.2f), 1.5f));

    Material* tallBoxMaterial = (tallBoxMaterialType == 0 ? white : metal);
    Material* shortObjectMaterial = (shortObjectMaterialType == 0 ? white : shortObjectMaterialType == 1 ? rgl : glass);

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

#if 1
    int animIndex = -1;
    if (shortObjectMoving)
        animIndex = scene.take(new ObjectAnimation);
    if (shortObjectType == 2) {
        Transformation soTransformation(vec3(0.33f, 0.301f, 0.37f), quat::null(), vec3(0.3f));
        Mesh* soData = scene.take(generateIcosahedron(soTransformation));
        scene.take(new MeshInstance(soData, shortObjectMaterial, animIndex));
    } else if (shortObjectType == 1) {
# if 1
        scene.take(new Sphere(vec3(0.33f, 0.3f, 0.37f), 0.3f, shortObjectMaterial, animIndex));//, HotSpot);
# else
        Material* glass = scene.take(new MaterialGlass(vec3(0.0f), 1.5f));
        Transformation sphereTransformation(vec3(0.33f, 0.3f, 0.37f), quat::null(), vec3(0.3f));
        scene.take(new TriangleShape(generateSphere(sphereTransformation, glass)));
# endif
    } else {
        // short box left
        scene.take(new MeshInstance(scene.take(new Mesh(
                    { vec3(-0.05f, 0.0f, 0.57f), vec3(-0.05f, 0.6f, 0.57f), vec3(0.13f, 0.6f, 0.0f), vec3(0.13f, 0.0f, 0.0f) },
                    { vec3(-0.9535826f, 0.0f, -0.3011314f), vec3(-0.9535826f, 0.0f, -0.3011314f), vec3(-0.9535826f, 0.0f, -0.3011314f), vec3(-0.9535826f, 0.0f, -0.3011314f) },
                    { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                    { 0, 1, 2, 0, 2, 3 })),
                    shortObjectMaterial, animIndex));
        // short box right
        scene.take(new MeshInstance(scene.take(new Mesh(
                    { vec3(0.7f, 0.0f, 0.17f), vec3(0.7f, 0.6f, 0.17f), vec3(0.53f, 0.6f, 0.75f), vec3(0.53f, 0.0f, 0.75f) },
                    { vec3(0.9596285f, 0.0f, 0.2812705f), vec3(0.9596285f, 0.0f, 0.2812705f), vec3(0.9596285f, 0.0f, 0.2812705f), vec3(0.9596285f, 0.0f, 0.2812705f) },
                    { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                    { 0, 1, 2, 0, 2, 3 })),
                    shortObjectMaterial, animIndex));
        // short box floor
        scene.take(new MeshInstance(scene.take(new Mesh(
                    { vec3(0.53f, 0.0f, 0.75f), vec3(0.7f, 0.0f, 0.17f), vec3(0.13f, 0.0f, 0.0f), vec3(-0.05f, 0.0f, 0.57f) },
                    { vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f) },
                    { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                    { 0, 1, 2, 0, 2, 3 })),
                    shortObjectMaterial, animIndex));
        // short box ceiling
        scene.take(new MeshInstance(scene.take(new Mesh(
                    { vec3(0.53f, 0.6f, 0.75f), vec3(0.7f, 0.6f, 0.17f), vec3(0.13f, 0.6f, 0.0f), vec3(-0.05f, 0.6f, 0.57f) },
                    { vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f) },
                    { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                    { 0, 1, 2, 0, 2, 3 })),
                    shortObjectMaterial, animIndex));
        // short box back
        scene.take(new MeshInstance(scene.take(new Mesh(
                    { vec3(0.13f, 0.0f, 0.0f), vec3(0.13f, 0.6f, 0.0f), vec3(0.7f, 0.6f, 0.17f), vec3(0.7f, 0.0f, 0.17f) },
                    { vec3(0.2858051f, 0.0f, -0.9582878f), vec3(0.2858051f, 0.0f, -0.9582878f), vec3(0.2858051f, 0.0f, -0.9582878f), vec3(0.2858051f, 0.0f, -0.9582878f) },
                    { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                    { 0, 1, 2, 0, 2, 3 })),
                    shortObjectMaterial, animIndex));
        // short box front
        scene.take(new MeshInstance(scene.take(new Mesh(
                    { vec3(0.53f, 0.0f, 0.75f), vec3(0.53f, 0.6f, 0.75f), vec3(-0.05f, 0.6f, 0.57f), vec3(-0.05f, 0.0f, 0.57f) },
                    { vec3(-0.2963993f, 0.0f, 0.9550642f), vec3(-0.2963993f, 0.0f, 0.9550642f), vec3(-0.2963993f, 0.0f, 0.9550642f), vec3(-0.2963993f, 0.0f, 0.9550642f) },
                    { vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f), vec2(0.0f, 1.0f) },
                    { 0, 1, 2, 0, 2, 3 })),
                    shortObjectMaterial, animIndex));
    }
#endif

#if 1
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
#endif

    // light source
    const float light_y = 1.98f;
    scene.take(new MeshInstance(scene.take(new Mesh(
                { vec3(-0.24f, light_y, 0.16f), vec3(-0.24f, light_y, -0.22f), vec3(0.23f, light_y, -0.22f), vec3(0.23f, light_y, 0.16f) },
                { vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f), vec3(0.0f, -1.0f, 0.0f) },
                { vec2(0.0f, 1.0f), vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 })),
                light) , HotSpot);
}

int main(void)
{
    bool preview = false;
#if 0 /* Use this configuration to demonstrate caustics */
    int tallBoxMaterial = 1;
    int shortObjectType = 2;
    int shortObjectMaterial = 2;
    bool animated = false;
    int samples_sqrt = 300;
#endif
    int tallBoxMaterial = 0;
    int shortObjectType = 0;
    int shortObjectMaterial = 0;
    bool animated = false;
    int samples_sqrt = 20;

    unsigned int width        = 1024;
    unsigned int height       = 1024;
    if (preview)
        samples_sqrt /= 4;

    Scene scene;
    createScene(scene, tallBoxMaterial, shortObjectType, shortObjectMaterial, animated);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(50.0f), sensor.aspectRatio()));

    vec3 lookfrom(0.0f, 1.0f, 3.2f);
    vec3 lookat(0.0f, 1.0f, -1.0f);
    vec3 up(0.0f, 1.0f, 0.0f);
    Camera camera(optics, Transformation::fromLookAt(lookfrom, lookat, up));

    Parameters params;
    //params.maxPathComponents=2;
    //params.randomizeRayOverPixel=true;

    float t0 = 0.0f;
    float t1 = animated ? 1.0f : 0.0f;
    scene.updateBVH(t0, t1);
    mcpt(sensor, camera, scene, samples_sqrt, t0, t1, params);

    const TGD::Array<float>& hdrImg = sensor.result();
    TGD::Array<float> ldrImg = uniformRationalQuantization(hdrImg, maxLuminance(hdrImg), 50.0f);
    TGD::Array<uint8_t> srgbImg = toSRGB(ldrImg);
    TGD::save(hdrImg, "image.exr");
    TGD::save(srgbImg, "image.png");
    //scene.exportToObj("cb");

    return 0;
}
