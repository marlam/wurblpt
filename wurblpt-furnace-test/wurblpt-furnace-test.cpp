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

#include <cstdio>
#include <filesystem>

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>

using namespace WurblPT;


int main(void)
{
    unsigned int width = 512;
    unsigned int height = 512;
    unsigned int samples_sqrt = 10;

    Scene scene;

    /* Light source: */
    bool useEnvMap = true;
    if (useEnvMap) {
        /* Environment map: */
        Texture* tex = scene.take(new TextureConstant(vec4(1.0f)));
        scene.take(new EnvironmentMapEquiRect(tex));
    } else {
        /* Surrounding light sphere, with or without importance sampling: */
        bool useMIS = false;
        Material* light = scene.take(new LightDiffuse(vec3(1.0f)));
        Material* lightTwoSided = scene.take(new MaterialTwoSided(light, light));
        SceneComponent* lightSphere = new Sphere(vec3(0.0f), 9.9f, lightTwoSided);
        //SceneComponent* lightSphere = new MeshInstance(scene.take(generateSphere(
        //                Transformation(vec3(0.0f), quat::null(), vec3(9.9f)))), lightTwoSided);
        scene.take(lightSphere, useMIS ? HotSpot : ColdSpot);
    }

    /* Material sphere: */
    Material* mat = scene.take(new MaterialLambertian(vec3(0.42f)));
    //Material* mat = scene.take(new MaterialLambertian(vec3(1.0f)));
    //Material* mat = scene.take(new MaterialModPhong(vec3(1.0f), vec3(0.0f)));
    //Material* mat = scene.take(new MaterialModPhong(vec3(0.0f), vec3(1.0f)));
    //Material* mat = scene.take(new MaterialModPhong(vec3(0.5f), vec3(0.5f)));
    //Material* mat = scene.take(new MaterialRGL("weta_brushed_steel_satin_pink_rgb.bsdf"));
    SceneComponent* sphere = new Sphere(vec3(0.0f), 1.0f, mat);
    scene.take(sphere);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(40.0f), sensor.aspectRatio()));

    Camera camera(optics, Transformation(vec3(0.0f, 0.0f, 5.0f)));

    scene.updateBVH();
    Parameters params;
    params.randomizeRayOverPixel = false; // disable antialiasing at the object border
    mcpt(sensor, camera, scene, samples_sqrt, 0.0f, 0.0f, params);
    TGD::save(sensor.result(), "frame.exr");

    return 0;
}
