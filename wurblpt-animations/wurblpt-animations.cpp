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
 * https://polyhaven.com/a/cobblestone_floor_08
 * https://polyhaven.com/a/brick_wall_001
 * License CC0 */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

class ObjectAnimation final : public Animation
{
public:
    int objectIndex;
    Transformation baseT;

    ObjectAnimation(int i) : objectIndex(i), baseT()
    {
        baseT.rotate(toQuat(radians(18.0f + (objectIndex + 1) * 72.0f), vec3(0.0f, 1.0f, 0.0f)));
        baseT.translate(vec3(2.0f, 1.75f, 0.0f));
        baseT.scale(vec3(0.12f));
        baseT.rotate(toQuat(radians(15.0f), vec3(1.0f, 1.0f, 0.0f)));
    }

    // from 0 to 18 seconds
    virtual Transformation at(float t) const
    {
        Transformation T = baseT;
        T.rotate(toQuat(radians(t * 20.0f), vec3(0.0f, 1.0f, 0.0f)));
        return T;
    }
};

class CameraAnimation final : public Animation
{
public:
    CameraAnimation() {}

    // from 0 to 36 seconds
    virtual Transformation at(float t) const
    {
        Transformation T;
        T.translation = toQuat(radians(t * -10.0f), vec3(0.0f, 1.0f, 0.0f)) * vec3(4.0f, 2.5f, 0.0f);
        T.rotation = toQuat(radians(vec3(-20.0f, 93.5f + t * -10.0f, 0.0f)));
        return T;
    }
};

class ObjectDiffuseTexture final : public Texture
{
public:
    int objectIndex;

    ObjectDiffuseTexture(int i) : objectIndex(i) {}

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
        int baseColor = (objectIndex + it) % 5;
        return vec4(mix(colors[baseColor], colors[baseColor + 1], fract(t)), 1.0f);
    }

    virtual vec2 texelSize() const override
    {
        return vec2(1.0f);
    }

    virtual unsigned int componentCount() const override
    {
        return 3;
    }

    virtual TGD::Type componentType() const override
    {
        return TGD::uint8;
    }
};

void createScene(Scene& scene)
{
    Texture* floorDifTex = scene.take(createTextureImage("cobblestone_floor_08_diff_1k.jpg", LinearizeSRGB_Auto, vec2(10.0f), vec2(0.0f)));
    Texture* floorSpcTex = scene.take(createTextureImage("cobblestone_floor_08_rough_1k.jpg", LinearizeSRGB_Off, vec2(10.0f), vec2(0.0f), vec4(0.2f)));
    Texture* floorNrmTex = scene.take(createTextureImage("cobblestone_floor_08_nor_gl_1k.jpg", LinearizeSRGB_Off, vec2(10.0f), vec2(0.0f)));
    MaterialModPhong* floorMaterial = new MaterialModPhong(
            vec3(0.5f, 0.5f, 0.3f), floorDifTex, vec3(0.5f, 0.5f, 0.3f), floorSpcTex, 1000.0f);
    floorMaterial->normalTex = floorNrmTex;
    scene.take(floorMaterial);
    Mesh* quadData = scene.take(generateQuad());
    assert(quadData->haveTangents);
    scene.take(new MeshInstance(quadData, floorMaterial,
                Transformation(vec3(0.0f),
                    toQuat(radians(-90.0f), vec3(1.0f, 0.0f, 0.0f)),
                    vec3(5.0f))));

    Texture* cylinderDifTex = scene.take(createTextureImage("brick_wall_001_diffuse_1k.jpg", LinearizeSRGB_Auto));
    Texture* cylinderSpcTex = scene.take(createTextureImage("brick_wall_001_rough_1k.jpg", LinearizeSRGB_Off, vec2(1.0f), vec2(0.0f), vec4(0.1f)));
    Texture* cylinderNrmTex = scene.take(createTextureImage("brick_wall_001_nor_gl_1k.jpg", LinearizeSRGB_Off));
    MaterialModPhong* cylinderMaterial = new MaterialModPhong(
            vec3(0.5f, 0.5f, 0.3f), cylinderDifTex, vec3(0.5f, 0.5f, 0.3f), cylinderSpcTex, 1000.0f);
    cylinderMaterial->normalTex = cylinderNrmTex;
    scene.take(cylinderMaterial);

    Mesh* cylinderData = scene.take(generateClosedCylinder());
    Mesh* cubeData = scene.take(generateCube());
    Mesh* coneData = scene.take(generateClosedCone());
    Mesh* torusData = scene.take(generateTorus());
    Mesh* teapotData = scene.take(generateIcosahedron());
    for (int i = 0; i < 5; i++) {
        Transformation pillarTransformation;
        pillarTransformation.rotate(toQuat(radians(18.0f + (i + 1) * 72.0f), vec3(0.0f, 1.0f, 0.0f)));
        pillarTransformation.translate(vec3(2.0f, 0.8f, 0.0f));
        pillarTransformation.scale(vec3(0.2f, 0.8f, 0.2f));
        // pillar
        scene.take(new MeshInstance(cylinderData, cylinderMaterial, pillarTransformation));
        // objects
        int objectAnimationIndex = scene.take(new ObjectAnimation(i));
        Material* objectMaterial = scene.take(new MaterialModPhong(
                    vec3(0.8f), scene.take(new ObjectDiffuseTexture(i)), vec3(0.2f), nullptr, 100.0f));
        scene.take(new MeshInstance(
                    (i == 0 ? cubeData : i == 1 ? coneData : i == 2 ? torusData : i == 4 ? teapotData : cylinderData),
                    objectMaterial, objectAnimationIndex));
    }

    // light source
    float light_x = 40.0f;
    float light_half_size = 20.0f;
    Mesh* lightData = scene.take(new Mesh(
                { vec3(light_x, 8.0f, -light_half_size), vec3(light_x, 8.0f, +light_half_size),
                  vec3(light_x, 8.0f + 2.0f * light_half_size, +light_half_size), vec3(light_x, 8.0f + 2.0f * light_half_size, -light_half_size) },
                { vec3(-1.0f, 0.0f, 0.0f), vec3(-1.0f, 0.0f, 0.0f), vec3(-1.0f, 0.0f, 0.0f), vec3(-1.0f, 0.0f, 0.0f) },
                { vec2(0.0f, 0.0f), vec2(0.0f, 1.0f), vec2(1.0f, 0.0f), vec2(1.0f, 1.0f) },
                { 0, 1, 2, 0, 2, 3 }));
    scene.take(new MeshInstance(lightData, scene.take(new LightDiffuse(vec3(1.0f)))), HotSpot);
}

int main(int argc, char* argv[])
{
    int frameno = -1; // default: all frames
    if (argc == 2) {
        frameno = atoi(argv[1]);
        fprintf(stderr, "Rendering only frame %d\n", frameno);
    }

    bool preview = false;
    unsigned int width        = 1920;
    unsigned int height       = 1080;
    unsigned int samples_sqrt = 20;
    if (preview) {
        width /= 4;
        height /= 4;
        samples_sqrt /= 2;
    }

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);

    Projection projection(radians(50.0f), sensor.aspectRatio());
    Optics optics(projection);
    CameraAnimation* camAnim = new CameraAnimation;
    Camera camera(optics, camAnim);

    float start = 0.0f;
    float end = 36.0f;
    float frameDuration = 1.0f / 25.0f;
    unsigned int frameCount = (end - start) / frameDuration;
    for (unsigned int frame = 0; frame < frameCount; frame++) {
        if (preview && frame % 10 != 0)
            continue;
        if (frameno >= 0 && static_cast<unsigned int>(frameno) != frame)
            continue;
        char frameString[12];
        snprintf(frameString, sizeof(frameString), "%04u", frame);

        float t0 = start + frame * frameDuration;
        float t1 = t0 + frameDuration;
        scene.updateBVH(t0, t1);

        mcpt(sensor, camera, scene, samples_sqrt, t0, t1);
        const TGD::Array<float>& hdrImg = sensor.result();
        TGD::save(hdrImg, std::string("animations-") + frameString + ".tgd");
        TGD::Array<uint8_t> ppImg = toSRGB(scaleLuminance(hdrImg, 4.0f));
        TGD::save(ppImg, std::string("animations-") + frameString + ".png");

#if 0
        scene.exportToObj(std::string("anim-") + frameString);
#endif
    }

    /* To convert the frames to a video, use the following:
     * ffmpeg -i animations-stereo-%04d.png
     *     -c:v libx265 -preset veryslow -crf 20 -vf format=yuv420p
     *     animations-stereo.mp4
     * (for preview, use animations-stereo-%03d0.png as input)
     */

    return 0;
}
