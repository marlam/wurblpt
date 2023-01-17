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

/* The textures are from
 * https://polyhaven.com/a/stone_wall
 * License CC0 */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

void createScene(Scene& scene)
{
    Texture* diffuseTex = scene.take(createTextureImage("stone_wall_diff_1k.jpg"));
    Texture* normalTex = scene.take(createTextureImage("stone_wall_nor_gl_1k.jpg", LinearizeSRGB_Off));

    Material* cubeMaterial = new MaterialLambertian(vec3(0.5f), diffuseTex);
    cubeMaterial->normalTex = normalTex;
    scene.take(cubeMaterial);

    Texture* sphereDiffuseTex = scene.take(new TextureTransformer(diffuseTex, vec2(6.0f, 3.0f)));
    Texture* sphereNormalTex = scene.take(new TextureTransformer(normalTex, vec2(6.0f, 3.0f)));

    Material* sphereMaterial = new MaterialLambertian(vec3(0.5f), sphereDiffuseTex);
    sphereMaterial->normalTex = sphereNormalTex;
    scene.take(sphereMaterial);

    Transformation cubeT(vec3(-1.1f, 0.2f, 0.0f), toQuat(radians(40.0f), normalize(vec3(1.0f, 1.0f, 0.0f))), vec3(0.75f));
    Transformation sphereT(vec3(+1.1f, 0.0f, 0.0f), toQuat(radians(90.0f), vec3(0.0f, 1.0f, 0.0f)));

    scene.take(new MeshInstance(scene.take(generateCube(cubeT)), cubeMaterial));
    scene.take(new Sphere(sphereMaterial, sphereT));

    Transformation lightT(vec3(-2.0f, 4.0f, 8.0f), quat::null(), vec3(2.0f));
    scene.take(new Sphere(scene.take(new LightDiffuse(vec3(15.0f))), lightT), HotSpot);
}

int main(void)
{
    bool preview = false;

    unsigned int width        = 800;
    unsigned int height       = 600;
    unsigned int samples_sqrt = (preview ? 10 : 20);

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(37.0f), sensor.aspectRatio()));
    Camera camera(optics, Transformation(vec3(0.0f, 0.0f, 5.0f)));

    scene.updateBVH();
    mcpt(sensor, camera, scene, samples_sqrt);
    const TGD::Array<float>& hdrImg = sensor.result();
    TGD::Array<uint8_t> ldrImg = toSRGB(hdrImg);

    //TGD::save(hdrImg, "image.exr");
    TGD::save(ldrImg, "image.png");

#if 0
    GroundTruth gt = getGroundTruth(sensor, camera, scene);
    TGD::save(gt.worldSpacePositions, "wspos.tgd");
    TGD::save(gt.worldSpaceGeometryNormals, "wsgnrm.tgd");
    TGD::save(gt.worldSpaceGeometryTangents, "wsgtan.tgd");
    TGD::save(gt.worldSpaceMaterialNormals, "wsmnrm.tgd");
    TGD::save(gt.worldSpaceMaterialTangents, "wsmtan.tgd");
    TGD::save(gt.cameraSpacePositions, "cspos.tgd");
    TGD::save(gt.cameraSpaceGeometryNormals, "csgnrm.tgd");
    TGD::save(gt.cameraSpaceGeometryTangents, "csgtan.tgd");
    TGD::save(gt.cameraSpaceMaterialNormals, "csmnrm.tgd");
    TGD::save(gt.cameraSpaceMaterialTangents, "csmtan.tgd");
#endif

    return 0;
}
