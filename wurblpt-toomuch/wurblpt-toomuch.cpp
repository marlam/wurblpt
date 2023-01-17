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

class ObjectAnimation : public Animation
{
    // from 0 to 18 seconds
public:
    int objectIndex;
    Transformation baseT;

    ObjectAnimation(int i) : objectIndex(i), baseT()
    {
        baseT.rotate(toQuat(radians(18.0f + (objectIndex + 1) * 72.0f), vec3(0.0f, 1.0f, 0.0f)));
        baseT.translate(vec3(2.0f, 1.9f, 0.0f));
        if (i == 4 || i == 3 || i == 1 || i == 0) /* make some objects a little smaller */
            baseT.scale(vec3(0.15f));
        else
            baseT.scale(vec3(0.2f));
        baseT.rotate(toQuat(radians(15.0f), vec3(1.0f, 1.0f, 0.0f)));
    }

    virtual Transformation at(float t) const
    {
        Transformation T = baseT;
        T.rotate(toQuat(radians(t * 20.0f), vec3(0.0f, 1.0f, 0.0f)));
        return T;
    }
};

class CameraAnimation : public Animation
{
    // from 0 to 36 seconds
public:
    CameraAnimation() {}

    virtual Transformation at(float t) const
    {
        Transformation T;
        T.translation = toQuat(radians(t * -10.0f), vec3(0.0f, 1.0f, 0.0f)) * vec3(3.0f, 3.0f, 0.0f);
        T.rotation = toQuat(radians(vec3(-30.0f, 100.0f + t * -10.0f, 0.0f)));
        return T;
    }
};

class LightAnimation : public Animation
{
    // from 0 to 18 seconds
public:
    int lightIndex;
    Transformation baseT;

    LightAnimation(int i) : lightIndex(i), baseT()
    {
        baseT.rotate(toQuat(radians(18.0f + (lightIndex + 1) * 72.0f), vec3(0.0f, 1.0f, 0.0f)));
        baseT.translate(vec3(1.0f, 1.3f, 0.0f));
        baseT.scale(vec3(0.3f, 0.3f, 0.03f));
        baseT.rotate(toQuat(radians(90.0f), vec3(0.0f, 1.0f, 0.0f)));
    }

    virtual Transformation at(float t) const
    {
        Transformation T = baseT;
        T.rotate(toQuat(radians(t * 20.0f + lightIndex * 72.0f), vec3(-1.0f, 0.0f, 0.0f)));
        return T;
    }
};

class LightTexture : public Texture
{
public:
    int lightIndex;

    LightTexture(int i) : lightIndex(i) {}

    virtual vec4 value(const vec2& /* texcoords */, float t) const override
    {
        vec3 colors[6] = {
            vec3(0.8f, 0.3f, 0.3f),
            vec3(0.8f, 0.6f, 0.3f),
            vec3(0.4f, 0.8f, 0.3f),
            vec3(0.3f, 0.3f, 0.8f),
            vec3(0.3f, 0.8f, 0.8f),
            vec3(0.8f, 0.3f, 0.3f)
        };
        t /= 2; // transition from one color to next takes 2 seconds
        int it = t;
        int baseColor = (lightIndex + it) % 5;
        return vec4(mix(colors[baseColor], colors[baseColor + 1], fract(t)), 1.0f);
    }
};

void createScene(Scene& scene, const Parameters& params)
{
    Transformation boxTransformation(vec3(0.0f, 4.0f, 0.0f), quat::null(), vec3(4.0f));
    Material* boxFloorMaterial = scene.take(new MaterialLambertian(vec3(0.8f)));
    Material* boxFloorMaterial2 = scene.take(new MaterialTwoSided(boxFloorMaterial, boxFloorMaterial));
    Material* boxWallMaterial = scene.take(new MaterialLambertian(vec3(0.8f)));
    Material* boxWallMaterial2 = scene.take(new MaterialTwoSided(boxWallMaterial, boxWallMaterial));
    scene.take(new MeshInstance(scene.take(generateCubeSide(3, boxTransformation)), boxFloorMaterial2));
    for (int side = 0; side < 6; side++)
        if (side != 3)
            scene.take(new MeshInstance(scene.take(generateCubeSide(side, boxTransformation)), boxWallMaterial2));

    Material* sphereMaterial = scene.take(new MaterialGGX(vec3(1.0f), vec2(0.001f)));
    scene.take(new Sphere(vec3(0.0f, 0.6f, 0.0f), 0.6f, sphereMaterial), HotSpot);

    Material* lightBacksideMaterial = scene.take(new MaterialLambertian(vec3(0.2f)));
    for (int i = 0; i < 5; i++) {
        int lightAnimationIndex = scene.take(new LightAnimation(i));
        Texture* lightTexture = scene.take(new LightTexture(i));
        Material* lightMaterial = scene.take(new LightDiffuse(vec3(10.0f), lightTexture));
        scene.take(new MeshInstance(scene.take(generateCubeSide(4)), lightMaterial, lightAnimationIndex), HotSpot);
        scene.take(new MeshInstance(scene.take(generateCubeSide(5)), lightMaterial, lightAnimationIndex), HotSpot);
        for (int side = 0; side < 4; side++)
            scene.take(new MeshInstance(scene.take(generateCubeSide(side)), lightBacksideMaterial, lightAnimationIndex));
    }

    Material* pillarMaterial = scene.take(new MaterialGlass(vec3(0.0f), 1.5f));
    for (int i = 0; i < 5; i++) {
        // pillars
        Transformation pillarTransformation;
        pillarTransformation.rotate(toQuat(radians(18.0f + (i + 1) * 72.0f), vec3(0.0f, 1.0f, 0.0f)));
        pillarTransformation.translate(vec3(2.0f, 0.8f + 1.1f * params.minHitDistance, 0.0f)); // slightly above ground
        pillarTransformation.scale(vec3(0.2f, 0.8f, 0.2f));
        scene.take(new MeshInstance(scene.take(generateClosedCylinder(pillarTransformation)), pillarMaterial));

        // objects
        Transformation objectTransformation; // handled by animation
        int objectAnimationIndex = scene.take(new ObjectAnimation(i));
        Material* objectMaterial = scene.take(new MaterialModPhong(vec3(0.7f), vec3(0.3f), 100.0f));
        scene.take(new MeshInstance(scene.take(
                    (i == 0 ? generateCube(objectTransformation)
                     : i == 1 ? generateClosedCone(objectTransformation)
                     : i == 2 ? generateTorus(objectTransformation)
                     : i == 3 ? generateIcosahedron(objectTransformation)
                     : generateClosedCylinder(objectTransformation))), objectMaterial, objectAnimationIndex));
    }
}

int main(int argc, char* argv[])
{
    int frameno = -1; // default: all frames
    if (argc == 2) {
        frameno = atoi(argv[1]);
        fprintf(stderr, "Rendering only frame %d\n", frameno);
    }

    bool preview = true;
    unsigned int width        = (preview ? 400 : 1600);
    unsigned int height       = (preview ? 300 : 1200);
    unsigned int samples_sqrt = (preview ?  10 :   40);

    Parameters params;
    Scene scene;
    createScene(scene, params);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(50.0f), sensor.aspectRatio()));

    Camera camera(optics, new CameraAnimation);

    std::filesystem::remove("toomuch.tgd");
    std::filesystem::remove("toomuch-postproc.tgd");
    float start = 0.0f;
    float end = 36.0f;
    float frameDuration = 1.0f / 25.0f;
    unsigned int frameCount = (end - start) / frameDuration;
    if (frameCount < 1)
        frameCount = 1;
    for (unsigned int frame = 0; frame < frameCount; frame++) {
        if (frameno >= 0 && static_cast<unsigned int>(frameno) != frame)
            continue;
        char frameString[12];
        snprintf(frameString, sizeof(frameString), "%04u", frame);

        float t0 = start + frame * frameDuration;
        float t1 = t0 + frameDuration;

        scene.updateBVH(t0, t1);
        mcpt(sensor, camera, scene, samples_sqrt, t0, t1, params);
        const TGD::Array<float>& hdrImg = sensor.result();

        TGD::save(hdrImg, std::string("toomuch-") + frameString + ".tgd");
        TGD::save(toSRGB(hdrImg), std::string("toomuch-") + frameString + ".png");
    }

    return 0;
}
