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

void createScene(Scene& scene, bool withHotSpots)
{
    Material* white = scene.take(new MaterialLambertian(vec4(0.8f)));
    Transformation wallLeftT, wallRightT, wallFrontT, wallBackT, wallTopT, wallBottomT;
    wallLeftT.translate(vec3(-2.6f, 0.0f, 0.0f));
    wallLeftT.scale(vec3(5.0f));
    wallLeftT.rotate(toQuat(radians(+90.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallRightT.translate(vec3(+2.6f, 0.0f, 0.0f));
    wallRightT.scale(vec3(5.0f));
    wallRightT.rotate(toQuat(radians(-90.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallFrontT.translate(vec3(0.0f, 0.0f, +5.0f));
    wallFrontT.scale(vec3(5.0f));
    wallFrontT.rotate(toQuat(radians(180.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallBackT.translate(vec3(0.0f, 0.0f, -4.6f));
    wallBackT.scale(vec3(5.0f));
    wallBackT.rotate(toQuat(radians(0.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallTopT.translate(vec3(0.0f, -2.499, 0.0f));
    wallTopT.scale(vec3(5.0f));
    wallTopT.rotate(toQuat(radians(+90.0f), vec3(1.0f, 0.0f, 0.0f)));
    wallBottomT.translate(vec3(0.0f, -5.0f, 0.0f));
    wallBottomT.scale(vec3(5.0f));
    wallBottomT.rotate(toQuat(radians(-90.0f), vec3(1.0f, 0.0f, 0.0f)));
    Mesh* wallLeft = scene.take(generateQuad(wallLeftT));
    Mesh* wallRight = scene.take(generateQuad(wallRightT));
    Mesh* wallFront = scene.take(generateQuad(wallFrontT));
    Mesh* wallBack = scene.take(generateQuad(wallBackT));
    Mesh* wallTop = scene.take(generateQuad(wallTopT));
    Mesh* wallBottom = scene.take(generateQuad(wallBottomT));
    scene.take(new MeshInstance(wallLeft, white));
    scene.take(new MeshInstance(wallRight, white));
    scene.take(new MeshInstance(wallFront, white));
    scene.take(new MeshInstance(wallBack, white));
    scene.take(new MeshInstance(wallTop, white));
    scene.take(new MeshInstance(wallBottom, white));

    Material* mat0 = scene.take(new MaterialGGX(vec3(1.0f), vec2(0.001f)));
    Material* mat1 = scene.take(new MaterialGGX(vec3(1.0f), vec2(0.008)));
    Material* mat2 = scene.take(new MaterialGGX(vec3(1.0f), vec2(0.03f)));
    Material* mat3 = scene.take(new MaterialGGX(vec3(1.0f), vec2(0.1f)));

    Transformation tra0(vec3(0.0f, -4.2f, -4.2f), toQuat(radians(-35.0f), vec3(1.0f, 0.0f, 0.0f)), vec3(2.0f, 0.3f, 1.0f));
    Transformation tra1(vec3(0.0f, -4.6f, -3.8f), toQuat(radians(-47.0f), vec3(1.0f, 0.0f, 0.0f)), vec3(2.0f, 0.3f, 1.0f));
    Transformation tra2(vec3(0.0f, -4.8f, -3.4f), toQuat(radians(-59.0f), vec3(1.0f, 0.0f, 0.0f)), vec3(2.0f, 0.3f, 1.0f));
    Transformation tra3(vec3(0.0f, -4.9f, -3.0f), toQuat(radians(-71.0f), vec3(1.0f, 0.0f, 0.0f)), vec3(2.0f, 0.3f, 1.0f));

    scene.take(new MeshInstance(scene.take(generateQuad(tra0)), mat0));
    scene.take(new MeshInstance(scene.take(generateQuad(tra1)), mat1));
    scene.take(new MeshInstance(scene.take(generateQuad(tra2)), mat2));
    scene.take(new MeshInstance(scene.take(generateQuad(tra3)), mat3));

    Transformation lt0(vec3(-1.5f, -3.5f, -4.0f), quat::null(), vec3(0.032f));
    Transformation lt1(vec3(-0.5f, -3.5f, -4.0f), quat::null(), vec3(0.08f));
    Transformation lt2(vec3(+0.5f, -3.5f, -4.0f), quat::null(), vec3(0.2f));
    Transformation lt3(vec3(+1.5f, -3.5f, -4.0f), quat::null(), vec3(0.5f));

    Material* light0 = scene.take(new LightDiffuse(vec3(4.0f)));
    Material* light1 = scene.take(new LightDiffuse(vec3(4.0f)));
    Material* light2 = scene.take(new LightDiffuse(vec3(4.0f)));
    Material* light3 = scene.take(new LightDiffuse(vec3(4.0f)));
    scene.take(new Sphere(light0, lt0), withHotSpots ? HotSpot : ColdSpot);
    scene.take(new Sphere(light1, lt1), withHotSpots ? HotSpot : ColdSpot);
    scene.take(new Sphere(light2, lt2), withHotSpots ? HotSpot : ColdSpot);
    scene.take(new Sphere(light3, lt3), withHotSpots ? HotSpot : ColdSpot);
}

int main(void)
{
    unsigned int width        = 960;
    unsigned int height       = 540;
    unsigned int samples_sqrt = 10;

    Scene sceneWithHotSpots;
    createScene(sceneWithHotSpots, true);
    sceneWithHotSpots.updateBVH();

    Scene sceneWithoutHotSpots;
    createScene(sceneWithoutHotSpots, false);
    sceneWithoutHotSpots.updateBVH();

    SensorRGB sensor(width, height);

    Projection projection(radians(50.0f), sensor.aspectRatio());
    Transformation camT(vec3(0.0f, -4.5f, -1.2f));
    Camera camera(Optics(projection), camT);

    // 1. Pure material sampling
    mcpt(sensor, camera, sceneWithoutHotSpots, samples_sqrt, 0.0f, 0.0f, Parameters());
    TGD::save(sensor.result(), "img-with-material-importance-sampling.tgd");

    // 2. Pure light sampling
#if 0
    // Requires manual modifications of MaterialLambertian; see comments there
    Parameters paramsForLightSampling;
    paramsForLightSampling.probabilityToHitHotSpots = 1.0f;
    mcpt(sensor, camera, sceneWithHotSpots, samples_sqrt, 0.0f, 0.0f, paramsForLightSampling);
    TGD::save(sensor.result(), "img-with-light-importance-sampling.tgd");
#endif

    // 3. Multiple Importance Sampling
    mcpt(sensor, camera, sceneWithHotSpots, samples_sqrt, 0.0f, 0.0f, Parameters());
    TGD::save(sensor.result(), "img-with-multiple-importance-sampling.tgd");

#if 0
    sceneWithoutHotSpots.exportToObj("mis");
    GroundTruth gt = getGroundTruth(sensor, camera, sceneWithoutHotSpots);
    TGD::save(gt.worldSpaceGeometryNormals, "normals.tgd");
    TGD::save(gt.worldSpacePositions, "positions.tgd");
#endif

    return 0;
}
