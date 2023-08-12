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

/* This is the scene from "Ray Tracing: The Next Week" by Peter Shirley. */

/* The texture is from the NASA Blue Marble collection:
 * https://visibleearth.nasa.gov/images/57752/blue-marble-land-surface-shallow-water-and-shaded-topography/57754l */

#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

class SphereTexture final : public TexturePerlinNoise
{
private:
    float _scale;

public:
    SphereTexture(float scale, Prng& prng) : TexturePerlinNoise(prng), _scale(scale)
    {
    }

    virtual vec4 value(const vec2& texcoord, float t) const override
    {
        return vec4(0.5f * (1.0f + sin(100.0f * texcoord.t() + 10.0f * turbulence(_scale * texcoord, t).r())));
    }
};

void createRandomScene(Scene& scene)
{
    Prng prng(42);

    Material* white = scene.take(new MaterialLambertian(vec3(0.73f)));
    Material* ground = scene.take(new MaterialLambertian(vec3(0.48f, 0.83f, 0.53f)));
    Material* light = scene.take(new LightDiffuse(vec3(7.0f)));

    const int nb = 20;
    const float w = 1.0f;
    for (int i = 0; i < nb; i++) {
        for (int j = 0; j < nb; j++) {
            Transformation T(
                    vec3(-10.0f + i * w, 0.01f, -10.0f + j * w),
                    quat::null(),
                    vec3(w, 1.0f * (prng.in01() + 0.001f), w));
            scene.take(new MeshInstance(scene.take(generateCube(T)), ground));
        }
    }

    Transformation lightT(vec3(2.73f, 6.0f, 2.795f),
            toQuat(radians(90.0f), vec3(1.0f, 0.0f, 0.0f)),
            vec3(2.0f, 1.5f, 0.1f));
    scene.take(new MeshInstance(scene.take(generateQuad(lightT)), light), HotSpot);

    AnimationKeyframes* anim = new AnimationKeyframes;
    Transformation T0;
    T0.translate(vec3(4.0f, 4.0f, 2.0f));
    T0.scale(0.5f);
    Transformation T1;
    T1.translate(vec3(4.0f, 4.0f, 2.0f) + vec3(0.3f, 0.0f, 0.0f));
    T1.scale(0.5f);
    anim->addKeyframe(0.0f, T0);
    anim->addKeyframe(1.0f, T1);
    scene.take(new Sphere(scene.take(new MaterialLambertian(vec3(0.7f, 0.3f, 0.1f))), scene.take(anim)));

    scene.take(new Sphere(vec3(2.6f, 1.5f, 0.45f), 0.5f,
                scene.take(new MaterialGlass(vec3(0.0f), 1.5f))));
    scene.take(new Sphere(vec3(0.0f, 1.5f, 1.45f), 0.5f,
                scene.take(new MaterialGGX(vec3(0.8f, 0.8f, 0.9f), vec2(0.1f)))));

    std::vector<const Hitable*> boundary = scene.take(new Sphere(vec3(4.0f, 1.7f, 1.0f), 0.7f, scene.take(new MaterialGlass(vec3(0.0f), 1.5f))));
    scene.take(new Medium(boundary, 6.0f, scene.take(new MaterialPhaseFunctionIsotropic(vec3(0.2f, 0.4f, 0.9f)))));
    boundary = scene.take(new Sphere(vec3(0.0f, 0.0f, 0.0f), 50.0f, scene.take(new MaterialGlass(vec3(0.0f), 1.5f))));
    scene.take(new Medium(boundary, 0.01f, scene.take(new MaterialPhaseFunctionIsotropic(vec3(1.0f)))));

    Texture* imgTex = scene.take(createTextureImage("land_shallow_topo_2048.jpg"));
    Material* eMat = scene.take(new MaterialLambertian(vec3(1.0f), imgTex));
    Transformation eT(vec3(4.0f, 2.2f, 4.0f), toQuat(radians(230.0f), vec3(0.0f, 1.0f, 0.0f)), vec3(1.0f));
    scene.take(new Sphere(eMat, eT));

    Texture* pTex = scene.take(new SphereTexture(0.1f, prng));
    scene.take(new Sphere(vec3(2.2f, 3.0f, 3.0f), 0.8f,
                scene.take(new MaterialLambertian(vec3(1.0f), pTex))));

    for (int j = 0; j < 1000; j++) {
        vec3 transl(-1.0f, 2.7f, 2.95f);
        scene.take(new Sphere(transl + vec3(
                        1.65f * prng.in01(),
                        1.65f * prng.in01(),
                        1.65f * prng.in01()), 0.1f, white));
    }
}

int main(int argc, char* argv[])
{
    /* Three configurations:
     * configuration==0: default; just render the standard image
     * configuration==1: light in flight, progressive
     * configuration==2: light in flight, cumulative
     * Configurations 1 and 2 result in a series of images / video
     * for which you can render either just one frame or all of them.
     */

    if (argc != 1 && argc != 2 && argc != 3) {
        fprintf(stderr, "Usage: %s [configuration] [frame]\n", argv[0]);
        return 1;
    }
    int configuration = (argc >= 2 ? atoi(argv[1]) : 0);
    int frameno = (argc >= 3 ? atoi(argv[2]) : -1);

    bool preview = false;
    unsigned int width        = 500;
    unsigned int height       = 500;
    unsigned int samples_sqrt = (preview ? 20 : 70);
    float t0 = 0.0f;
    float t1 = (configuration == 0 ? 1.0f : 0.0f);

    Scene scene;
    createRandomScene(scene);
    scene.updateBVH(t0, t1);

    Optics optics(Projection(radians(40.0f), float(width) / height),
            LensDistortion(), LensDepthOfField(0.0f, 0.1f));
    Transformation cameraT(vec3(7.5f, 1.5f, -6.0f), toQuat(radians(vec3(10.0f, 150.0f, 0.0f))));
    Camera camera(optics, cameraT);

    float distanceToLightStartFirst = -0.25f;
    float distanceToLightStartLast = 14.0f;
    float distanceToLightWidth = 0.25f;
    int frameCount = (configuration == 0 ? 1 : 200);

    float distanceToLightStartStep = (distanceToLightStartLast - distanceToLightStartFirst) / frameCount;
    for (int frame = 0; frame < frameCount; frame++) {
        if (frameno >= 0 && frame != frameno)
            continue;

        std::string filename = "rttnw";
        if (configuration > 0) {
            char frameString[32] = "";
            snprintf(frameString, sizeof(frameString), "-%s-%04d",
                    configuration == 1 ? "progressive" : "cumulative",
                    frame);
            filename += frameString;
        }
        filename += ".png";

        float distanceToLightMin = distanceToLightStartFirst + frame * distanceToLightStartStep;
        SensorRGB sensor(width, height,
                (configuration == 1 ? distanceToLightMin : 0.0f),
                (configuration > 0 ? distanceToLightMin + distanceToLightWidth : std::numeric_limits<float>::max()));
        mcpt(sensor, camera, scene, samples_sqrt, t0, t1);
        TGD::save(toSRGB(sensor.result()), filename);
    }

    return 0;
}
