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

/* The normal map is from
 * https://polyhaven.com/a/aerial_beach_01
 * License CC0 */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>

using namespace WurblPT;


void createScene(Scene& scene, bool withHotSpots)
{
    HotSpotType hotSpotType = (withHotSpots ? HotSpot : ColdSpot);

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

    Material* mat0 = scene.take(new MaterialModPhong(vec3(0.5f), vec3(0.5f), 120.0f));
    Material* mat1 = scene.take(new MaterialPhaseFunctionIsotropic(vec3(1.0f)));
    Material* mat2 = scene.take(new MaterialGlass(MaterialGlass::transparentColorToAbsorption(vec3(1.0f)), 1.5f));
    Texture* normalTex = scene.take(createTextureImage("aerial_beach_01_nor_gl_2k.jpg", LinearizeSRGB_Off));
    Texture* sphereNormalTex = scene.take(new TextureTransformer(normalTex, vec2(6.0f, 3.0f)));
    mat2->normalTex = sphereNormalTex;
    Material* mat3 = scene.take(new MaterialGGX(vec3(1.0f), vec2(0.01f, 0.1f)));

    quat rot = toQuat(radians(0.0f), vec3(0.0f, 1.0f, 0.0f));
    Transformation tra0(vec3(-1.5f, -4.7f, -4.0f), toQuat(radians(30.0f), vec3(0.0f, 1.0f, 0.0f)),  vec3(0.3f));
    Transformation tra1(vec3(-0.5f, -4.7f, -4.0f), toQuat(radians(160.0f), vec3(0.0f, 1.0f, 0.0f)), vec3(0.3f));
    Transformation tra2(vec3(+0.5f, -4.7f, -4.0f), rot,                                             vec3(0.3f));
    Transformation tra3(vec3(+1.5f, -4.7f, -4.0f), rot,                                             vec3(0.3f));

    scene.take(new MeshInstance(scene.take(generateTorus(tra0, 0.4f, 400, 400)), mat0));
#if 1
#if 0
    // unhitable boundary: we go directly into the "fog"
    scene.take(new HitableMedium(scene.takeUnhitable(new MeshInstance(dat0, nullptr)), 24.0f, mat0));
#else
    // hitable transparent boundary
    std::vector<const Hitable*> transparentBoundary = scene.take(
            new MeshInstance(scene.take(generateOctahedron(tra1)),
                scene.take(new MaterialGlass(
                        MaterialGlass::transparentColorToAbsorption(vec3(1.0f)), 1.5f))));
    scene.take(new Medium(transparentBoundary, 2.5f, mat1));
#endif
    scene.take(new Sphere(mat2, tra2)); //MeshInstance(dat2, mat2));
    scene.take(new MeshInstance(scene.take(generateIcosahedron(tra3)), mat3));
#endif

    Material* light0 = scene.take(new LightSpot(radians(20.0f), vec3(73.0f, 118.0f, 139.0f) ));
    Material* light1 = scene.take(new LightSpot(radians(20.0f), vec3(243.0f, 108.0f, 100.0f)));
    Material* light2 = scene.take(new LightSpot(radians(20.0f), vec3(191.0f, 197.0f, 85.0f) ));
    Material* light3 = scene.take(new LightSpot(radians(20.0f), vec3(165.0f, 69.0f, 179.0f) ));

    quat lrot = toQuat(radians(90.0f), vec3(1.0f, 0.0f, 0.0f));
    Transformation lt0(vec3(-1.5f, -2.5f, -4.0f), lrot, vec3(0.3f));
    Transformation lt1(vec3(-0.5f, -2.5f, -4.0f), lrot, vec3(0.3f));
    Transformation lt2(vec3(+0.5f, -2.5f, -4.0f), lrot, vec3(0.3f));
    Transformation lt3(vec3(+1.5f, -2.5f, -4.0f), lrot, vec3(0.3f));

    Mesh* lamp0 = scene.take(generateQuad(lt0));
    Mesh* lamp1 = scene.take(generateQuad(lt1));
    Mesh* lamp2 = scene.take(generateQuad(lt2));
    Mesh* lamp3 = scene.take(generateQuad(lt3));

    scene.take(new MeshInstance(lamp0, light0), hotSpotType);
    scene.take(new MeshInstance(lamp1, light1), hotSpotType);
    scene.take(new MeshInstance(lamp2, light2), hotSpotType);
    scene.take(new MeshInstance(lamp3, light3), hotSpotType);
}

int main(void)
{
    MPICoordinator mpiCoordinator;

    bool preview = true;

    unsigned int width        = 3840;
    unsigned int height       = 2160;
    unsigned int samples_sqrt = 200;
    Parameters params;
    if (preview) {
        width /= 4;
        height /= 4;
        samples_sqrt /= 10;
        //params.maxPathComponents = 4;
        //params.rrThreshold = 0.0f;
    }

    Scene scene;
    createScene(scene, true);

    SensorRGB sensor(width, height);

    Projection projection(radians(50.0f), sensor.aspectRatio());
    Transformation camT(vec3(0.0f, -4.5f, -1.2f));
    Camera camera(Optics(projection), camT);

    scene.updateBVH(0.0f, 0.0f);
    mcpt(mpiCoordinator, sensor, camera, scene, samples_sqrt, 0.0f, 0.0f, params);
    if (mpiCoordinator.mainProcess()) {
        const TGD::Array<float>& hdrImg = sensor.result();
        TGD::save(hdrImg, "img.tgd");
#if 0
        TGD::Array<uint8_t> ppImg = toSRGB(scaleLuminance(hdrImg, 8.0f));
        TGD::save(ppImg, "img-postproc.tgd");
#endif
#if 0
        GroundTruth gt = getGroundTruth(sensor, camera, scene);
        TGD::save(gt.cameraSpaceGeometryNormals, "normals.tgd");
        TGD::save(gt.cameraSpacePositions, "positions.tgd");
#endif
    }

    return 0;
}
