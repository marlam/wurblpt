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

class CameraAnimation : public Animation
{
public:
    CameraAnimation()
    {
    }

    /* return Transformation at time t in seconds */
    virtual Transformation at(float t) const
    {
        const vec3 center = vec3(0.0f, 0.0f, -1.0f); // always look at center of checker board
        const vec3 up = vec3(0.0f, 1.0f, 0.0f);

        float alpha = radians(t * 3.0f / 5.0f * 360.0f);
        float radius = mix(1.3f, 0.7f, t / 5.0f);
        float x = radius * cos(alpha);
        float y = radius * sin(alpha);
        float z = mix(1.0f, 0.0f, t / 5.0f);
#if 0
        x = 0.0f;
        y = 0.0f;
        z = 1.0f;
#endif
        vec3 eye(x, y, z);

        return Transformation::fromLookAt(eye, center, up);
    }
};

class ObjectAnimation2 : public Animation
{
public:
    ObjectAnimation2()
    {
    }

    /* return Transformation at time t in seconds */
    virtual Transformation at(float t) const
    {
        float alpha = radians(t / 5.0f * 360.0f);
        float radius = 0.2f;
        float x = radius * cos(alpha);
        float y = radius * sin(alpha);
        float z = 0.0f;
        Transformation T;
        T.translate(vec3(1.0f + x, 1.0f + y, -1.0f + z));
        T.rotate(toQuat(alpha, vec3(0.0f, 1.0f, 0.0f)));
        return T;
    }
};

class ObjectAnimation4 : public Animation
{
public:
    ObjectAnimation4()
    {
    }

    /* return Transformation at time t in seconds */
    virtual Transformation at(float t) const
    {
        float alpha = radians(t / 5.0f * 360.0f);
        float radius = 0.2f;
        float x = radius * cos(alpha);
        float y = radius * sin(alpha);
        float z = 0.0f;
        Transformation T;
        T.translate(vec3(1.0f - x, -1.0f - y, -1.0f + z));
        T.rotate(toQuat(alpha, vec3(0.0f, -1.0f, 0.0f)));
        return T;
    }
};

void createScene(Scene& scene)
{
    Material* uberLight = scene.take(new LightDiffuse(vec3(2.0f)));
    Material* white = scene.take(new MaterialLambertian(vec3(0.8f)));
    Material* red = scene.take(new MaterialLambertian(vec3(0.8f, 0.0f, 0.0f)));
    Material* green = scene.take(new MaterialLambertian(vec3(0.0f, 0.8f, 0.0f)));
    Material* blue = scene.take(new MaterialLambertian(vec3(0.0f, 0.0f, 0.8f)));
    Transformation wallLeftT, wallRightT, wallFrontT, wallBackT, wallTopT, wallBottomT;
    wallLeftT.translate(vec3(-2.0f, 0.0f, 0.0f));
    wallLeftT.scale(vec3(2.0f));
    wallLeftT.rotate(toQuat(radians(+90.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallRightT.translate(vec3(+2.0f, 0.0f, 0.0f));
    wallRightT.scale(vec3(2.0f));
    wallRightT.rotate(toQuat(radians(-90.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallFrontT.translate(vec3(0.0f, 0.0f, +2.0f));
    wallFrontT.scale(vec3(2.0f));
    wallFrontT.rotate(toQuat(radians(180.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallBackT.translate(vec3(0.0f, 0.0f, -2.0f));
    wallBackT.scale(vec3(2.0f));
    wallBackT.rotate(toQuat(radians(0.0f), vec3(0.0f, 1.0f, 0.0f)));
    wallTopT.translate(vec3(0.0f, +2.0, 0.0f));
    wallTopT.scale(vec3(2.0f));
    wallTopT.rotate(toQuat(radians(+90.0f), vec3(1.0f, 0.0f, 0.0f)));
    wallBottomT.translate(vec3(0.0f, -2.0f, 0.0f));
    wallBottomT.scale(vec3(2.0f));
    wallBottomT.rotate(toQuat(radians(-90.0f), vec3(1.0f, 0.0f, 0.0f)));
    Mesh* wallLeft = scene.take(generateQuad(wallLeftT));
    Mesh* wallRight = scene.take(generateQuad(wallRightT));
    Mesh* wallFront = scene.take(generateQuad(wallFrontT));
    Mesh* wallBack = scene.take(generateQuad(wallBackT));
    Mesh* wallTop = scene.take(generateQuad(wallTopT));
    Mesh* wallBottom = scene.take(generateQuad(wallBottomT));
    scene.take(new MeshInstance(wallLeft, red));
    scene.take(new MeshInstance(wallRight, green));
    scene.take(new MeshInstance(wallFront, white));
    scene.take(new MeshInstance(wallBack, white));
    scene.take(new MeshInstance(wallTop, uberLight), HotSpot);
    scene.take(new MeshInstance(wallBottom, blue));

    Texture* texChecker = scene.take(new TextureChecker(
                vec4(0.05f, 0.2f, 0.0f, 0.0f), vec4(0.2f, 0.05f, 0.0f, 0.0f), 8, 8));
    Material* mat = scene.take(new MaterialGGX(vec3(0.75f), nullptr, vec2(0.0f), texChecker));
    //Material* mat = scene.take(new MaterialLambertian(vec4(0.5f)));

    // Variant 1 (top left): Untransformed, Static
    Transformation tObj1;
    tObj1.translate(vec3(-1.0f, +1.0f, -1.0f));
    tObj1.scale(vec3(0.4f));
    Mesh* dataObj1 = scene.take(generateCube(tObj1));
    scene.take(new MeshInstance(dataObj1, mat));
    // Variant 2 (top right): Untransformed, Animated
    Transformation tObj2;
    tObj2.scale(0.4f);
    Mesh* dataObj2 = scene.take(generateCube(tObj2));
    scene.take(new MeshInstance(dataObj2, mat, Transformation(), scene.take(new ObjectAnimation2)));
    // Variant 3 (bottom left): Transformed, Static
    Transformation tObj3;
    tObj3.translate(vec3(-1.0f, -1.0f, -1.0f));
    tObj3.scale(vec3(0.4f));
    Mesh* dataObj34 = scene.take(generateCube());
    scene.take(new MeshInstance(dataObj34, mat, tObj3));
    // Variant 4 (bottom right): Transformed, Animated
    Transformation tObj4;
    tObj4.scale(vec3(0.4f));
    scene.take(new MeshInstance(dataObj34, mat, tObj4, scene.take(new ObjectAnimation4)));
}

int main(void)
{
    unsigned int width = 800;
    unsigned int height = 600;
    unsigned int samples_sqrt = 15;

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);

    vec2 c = vec2(398.5f, 301.3f);
    vec2 f = vec2(400.42f, 300.42f);
    Projection proj(width, height, c, f);
    Optics optics(proj);

    Camera camera(optics, new CameraAnimation());

    float start = 0.0f;
    float end = 5.0f;
    float frameDuration = 1.0f / 25.0f;
    unsigned int frameCount = (end - start) / frameDuration;
    for (unsigned int frame = 0; frame < frameCount; frame++) {
        float t0 = start + frame * frameDuration;
        float t1 = t0;
        scene.updateBVH(t0, t1);
        mcpt(sensor, camera, scene, samples_sqrt, t0, t1);
        const float maxLum = 9000.0f;
        TGD::Array<float> ldrImg = uniformRationalQuantization(sensor.result(), maxLum, 8.0f);
        TGD::Array<uint8_t> srgbImg = toSRGB(ldrImg);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%04u", frame);
        TGD::save(srgbImg, std::string("frame-") + buf + ".pnm");
    }

    return 0;
}
