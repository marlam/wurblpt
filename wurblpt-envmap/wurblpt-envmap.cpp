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

/* The environment map royal_esplanade_2k.hdr is from
 * https://polyhaven.com/a/royal_esplanade
 * License CC0 */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>
#include <wurblpt/wurblpt.hpp>

using namespace WurblPT;


void createScene(Scene& scene)
{
#if 0
    //Material* material = new MaterialGGX(vec3(0.9f), vec2(0.06f, 0.01f)); // 0.001 for mirror-like
    //Material* material = new MaterialGGX(vec3(0.0f), vec2(0.001f));// 0.001 for mirror-like
    Texture* texChecker = scene.take(new TextureChecker(
                vec4(0.001f, 0.05f, 0.0f, 0.0f), vec4(0.05f, 0.001f, 0.0f, 0.0f), 16, 8));
    Material* material = new MaterialGGX(vec3(0.75f), nullptr, vec2(0.0f), texChecker);
#elif 1
    MaterialGlass* material = new MaterialGlass(MaterialGlass::transparentColorToAbsorption(vec3(1.0f, 1.0f, 1.0f)), 1.5f);
#else
    MaterialMirror* material = new MaterialMirror(vec4(1.0f));
#endif
    scene.take(material);

#if 0
    //Transformation objectT(vec3(0.0f), toQuat(radians(90.0f), vec3(1.0f, 0.0f, 0.0f)));
    //Transformation objectT(vec3(0.0f), toQuat(radians(-45.0f), vec3(0.0f, 1.0f, 0.0f)));
    Transformation objectT(vec3(0.0f), toQuat(radians(vec3(40.0f, 80.0f, 120.0f))));
#else
    Transformation objectT;//(vec3(0.0f), quat::null(), vec3(0.2f));
#endif
    scene.take(new Sphere(material, objectT));

    EnvironmentMap* envmap;
#if 0
    Texture* posx = scene.take(createTextureImage("cubemap-posx.png"));
    Texture* negx = scene.take(createTextureImage("cubemap-negx.png"));
    Texture* posy = scene.take(createTextureImage("cubemap-posy.png"));
    Texture* negy = scene.take(createTextureImage("cubemap-negy.png"));
    Texture* posz = scene.take(createTextureImage("cubemap-posz.png"));
    Texture* negz = scene.take(createTextureImage("cubemap-negz.png"));
    envmap = scene.take(new EnvironmentMapCube(posx, negx, posy, negy, posz, negz));
#else
    Texture* tex = scene.take(createTextureImage("royal_esplanade_2k.hdr"));
    envmap = scene.take(new EnvironmentMapEquiRect(tex));
#endif
    //envmap->initializeImportanceSampling(512);
}

int main(void)
{
    bool preview = false;

    unsigned int width        = 800;
    unsigned int height       = 600;
    unsigned int samples_sqrt = (preview ? 15 : 25);

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(50.0f), sensor.aspectRatio()));

    Camera camera(optics, Transformation(vec3(0.0f, 0.0f, 2.5f)));

    scene.updateBVH();

    mcpt(sensor, camera, scene, samples_sqrt);
    const TGD::Array<float>& hdrImg = sensor.result();
    TGD::Array<uint8_t> ldrImg = toSRGB(hdrImg);

    TGD::save(hdrImg, "image.exr");
    TGD::save(ldrImg, "image.png");

#if 0
    GroundTruth gt = getGroundTruth(sensor, camera, scene);
    TGD::save(gt.worldSpacePositions, "gt-world-space-positions.pfs");
    TGD::save(gt.worldSpaceNormals, "gt-world-space-normals.pfs");
    TGD::save(gt.worldSpaceTangents, "gt-world-space-tangents.pfs");
    TGD::save(gt.texCoords, "gt-texcoords.pfs");
    //TGD::save(gt.cameraSpacePositions, "gt-camera-space-positions.pfs");
    //TGD::save(gt.cameraSpaceDepths, "gt-camera-space-depths.pfs");
    //TGD::save(gt.cameraSpaceDistances, "gt-camera-space-distances.pfs");
#endif

    return 0;
}
