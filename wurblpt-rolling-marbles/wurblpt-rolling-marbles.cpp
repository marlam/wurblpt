/*
 * Copyright (C) 2023
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


/* Animation of the rolling marbles. Each marble gets its own instance of this animation. */
class MarbleAnimation final : public Animation
{
public:
    float radius;
    float circleRadius;
    float initialAngle;
    float direction;

    MarbleAnimation(float radius, float circleRadius, float initialAngle, float direction) :
        radius(radius),
        circleRadius(circleRadius),
        initialAngle(initialAngle),
        direction(direction)
    {
    }

    // from 0 to 18 seconds
    virtual Transformation at(float t) const
    {
        Transformation T;
        float angle = initialAngle + direction * radians(t * 20.0f);
        float arcLength = circleRadius * angle;
        float rotationsAroundX = arcLength / (2.0f * pi * radius);
        T.translate(vec3(circleRadius * cos(angle), 0.25f, circleRadius * sin(angle)));
        T.rotate(toQuat(rotationsAroundX * 2.0f * pi, vec3(-1.0f, 0.0f, 0.0f)) * toQuat(angle, vec3(0.0f, 1.0f, 0.0f)));
        T.scale(radius);
        return T;
    }
};

/* Conversion of sphere texture coordinates to equal-area map coordinates
 * so that we can apply e.g. bump maps without getting unsightly artifacts
 * in the polar regions.
 * The idea is to map a hemisphere to a circle in an area-preserving way,
 * to get good sampling quality over the hemisphere, and then to map both
 * hemispheres onto the same map, so that the generated maps are consistent
 * at their shared border (at the cost of having identical maps for north
 * and south hemisphere). */
vec2 mapSphere(const vec2& tc)
{
    // recover latitude and longitude from sphere texture coordinates
    float lat = (fract(tc.y()) - 0.5f) * pi;     // in [-pi/2,pi/2]
    float lon = fract(tc.x()) * 2.0f * pi - pi;  // in [-pi,pi]
    // map northern and southern hemisphere onto the same disk
    lat = abs(lat);
    // map from hemisphere to disk using Lambert Equal Area projection
    float r = sqrt2 * sin(0.5f * (pi_2 - lat));
    float alpha = lon - pi_2;
    // compute cartesian coordinates in map, in [0,1]^2
    vec2 uv = r * vec2(cos(alpha), sin(alpha));
    return 0.5f * (uv + vec2(1.0f));
}

/* A normal map based on gradient noise */
class BumpyNormalMap final : public Texture
{
private:
    constexpr static int baseSize = 16;
    TextureGradientNoise baseNoiseTex;

public:
    BumpyNormalMap(Prng& prng) :
        baseNoiseTex(baseSize, baseSize, prng)
    {
    }

    vec4 value(const vec2& texcoord, float t) const override
    {
        vec2 tc = mapSphere(texcoord);
        const float offset = 1.0f / baseSize;
        const float bumpScaling = 1.0f;
        float heightR = baseNoiseTex.value(tc + vec2(+offset, 0.0f), t).r();
        float heightL = baseNoiseTex.value(tc + vec2(-offset, 0.0f), t).r();
        float heightT = baseNoiseTex.value(tc + vec2(0.0f, +offset), t).r();
        float heightB = baseNoiseTex.value(tc + vec2(0.0f, -offset), t).r();
        vec3 tx = vec3(2.0, 0.0, bumpScaling * (heightR - heightL));
        vec3 ty = vec3(0.0, 2.0, bumpScaling * (heightT - heightB));
        vec3 n = normalize(cross(tx, ty));
        return vec4(0.5f * (n + vec3(1.0f)), 1.0f);
    }
};

/* A turbulence texture based on gradient noise */
class TurbulenceTex : public Texture
{
private:
    vec3 baseColor;
    TextureGradientNoise baseNoiseTex;

public:
    TurbulenceTex(const vec3& baseColor, Prng& prng) :
        baseColor(baseColor),
        baseNoiseTex(16, 16, prng)
    {
    }

    vec4 value(const vec2& texcoord, float t) const override
    {
        vec2 tc = mapSphere(texcoord);
        float amp = 1.0f;
        float freq = 1.0f;
        float value = 0.0f;
        for (int i = 0; i < 6; i++) {
            value += amp * abs(baseNoiseTex.value(tc * freq, t).r());
            freq *= 2.0f;
            amp *= 0.5f;
        }
        return vec4(value * baseColor, 1.0f);
    }
};

/* A marble texture based on turbulence */
class MarbleTex final : public TurbulenceTex
{
public:
    MarbleTex(Prng& prng) : TurbulenceTex(vec3(1.0f), prng)
    {
    }

    vec4 value(const vec2& texcoord, float t) const override
    {
        // Note that we want sphere-based texture coordinates here and not the
        // equal-area map coordinates to get the desired effect. The turbulence
        // texture will use equal-area coordinates.
        vec2 uv = texcoord;
        float v = sin(30.0f * uv.y() + 5.0f * TurbulenceTex::value(uv, t).r());
        return vec4(0.8f * vec3(0.5f * (v + 1.0f)), 1.0f);
    }
};

/* A distortion texture based on gradient noise */
class DistortionTex final : public Texture
{
private:
    vec3 baseColor;
    TextureGradientNoise baseNoiseTex;

public:
    DistortionTex(const vec3& baseColor, Prng& prng) :
        baseColor(baseColor),
        baseNoiseTex(16, 16, prng)
    {
    }

    vec4 value(const vec2& texcoord, float t) const override
    {
        // Note that we want sphere-based texture coordinates here and not the
        // equal-area map coordinates to get the desired effect
        vec2 tc = texcoord;
        if (texcoord.y() > 0.5f)
            tc.y() = 2.0f * (tc.y() - 0.5f);
        else
            tc.y() = 2.0f * (0.5f - tc.y());
        tc.x() *= 4.0f;

        float v = tc.y();
        v = min(1.0f, max(0.0f, v + 0.2f * baseNoiseTex.value(tc, t).r()));
        v = min(1.0f, max(0.0f, v + 0.2f * baseNoiseTex.value(vec2(tc.x(), v), t).r()));

        return vec4(v * baseColor, 1.0f);
    }
};

/* Create the scene with random marble materials */
void createScene(Scene& scene)
{
    Transformation roomTransformation;
    roomTransformation.translate(vec3(0.0f, 1.5f, 0.0f));
    roomTransformation.scale(vec3(6.0f, 1.5f, 6.0f));
    Mesh* room = scene.take(generateCube(roomTransformation));
    Material* roomMaterialOneSided = scene.take(new MaterialModPhong(vec3(0.6f), vec3(0.4f), 100.0f));
    Material* roomMaterial = scene.take(new MaterialTwoSided(roomMaterialOneSided, roomMaterialOneSided));
    scene.take(new MeshInstance(room, roomMaterial));

    Transformation lightTransformation;
    lightTransformation.translate(vec3(0.0f, 2.999f, 0.0f));
    lightTransformation.scale(vec3(2.0f));
    lightTransformation.rotate(toQuat(radians(+90.0f), vec3(1.0f, 0.0f, 0.0f)));
    Material* lightMaterial = scene.take(new LightDiffuse(vec3(1.0f)));
    scene.take(new MeshInstance(generateQuad(lightTransformation), lightMaterial), HotSpot);

    Prng prng(31415926);
    constexpr int circles = 4;
    float circleRadii[circles] = { 5.0f, 4.0f, 3.0f, 2.0f };
    int marblesPerCircle[circles] = { 12, 12, 12, 12 };
    float marbleStartAngles[circles] = { 0.0f, radians(7.5f), radians(15.0f), radians(22.5f) };
    float marbleDirections[circles] = { +1.0f, -1.0f, +1.0f, -1.0f };
    float marbleRadius = 0.25f;

    for (int i = 0; i < circles; i++) {
        for (int j = 0; j < marblesPerCircle[i]; j++) {
            float marbleStartAngle = marbleStartAngles[i] + j * radians(360.0f / marblesPerCircle[i]);
            int marbleAnimationIndex = scene.take(new MarbleAnimation(marbleRadius, circleRadii[i], marbleStartAngle, marbleDirections[i]));
            float materialSwitch = prng.in01();
            Material* marbleMaterial;
            bool marbleIsLightSource = false;
            if (materialSwitch < 0.15f) {
                marbleMaterial = new MaterialMirror(
                        vec3(0.8f + (0.2f * prng.in01()),
                             0.8f + (0.2f * prng.in01()),
                             0.8f + (0.2f * prng.in01())));
                float extraSwitch = prng.in01();
                if (extraSwitch < 0.33f) {
                    marbleMaterial->normalTex = scene.take(new BumpyNormalMap(prng));
                }
            } else if (materialSwitch < 0.3f) {
                marbleMaterial = new MaterialGlass(MaterialGlass::transparentColorToAbsorption(
                        vec3(0.98f + 0.02f * prng.in01(),
                             0.98f + 0.02f * prng.in01(),
                             0.98f + 0.02f * prng.in01())),
                        1.5f);
                float extraSwitch = prng.in01();
                if (extraSwitch < 0.33f) {
                    marbleMaterial->normalTex = scene.take(new BumpyNormalMap(prng));
                }
            } else if (materialSwitch < 0.6f) {
                vec3 baseColor = vec3(
                        0.3f + 0.7f * prng.in01() * prng.in01(),
                        0.3f + 0.7f * prng.in01() * prng.in01(),
                        0.3f + 0.7f * prng.in01() * prng.in01());
                float extraSwitch = prng.in01();
                if (extraSwitch < 0.33f) {
                    marbleMaterial = new MaterialLambertian(vec3(0.0f), scene.take(new TurbulenceTex(baseColor, prng)));
                } else if (extraSwitch < 0.66f) {
                    marbleMaterial = new MaterialLambertian(vec3(0.0f), scene.take(new DistortionTex(baseColor, prng)));
                } else {
                    marbleMaterial = new MaterialLambertian(baseColor);
                }
            } else if (materialSwitch < 0.65f) {
                marbleMaterial = new MaterialModPhong(
                        vec3(0.0f), scene.take(new MarbleTex(prng)),
                        vec3(0.2f), nullptr, 200.0f);
            } else {
                float diffuse = 0.8f;
                float specular = 1.0f - diffuse;
                float shininess = 100.0f + 600.0f * prng.in01();
                marbleMaterial = new MaterialModPhong(
                        diffuse * vec3(
                            0.3f + 0.7f * prng.in01() * prng.in01(),
                            0.3f + 0.7f * prng.in01() * prng.in01(),
                            0.3f + 0.7f * prng.in01() * prng.in01()),
                        vec3(specular * prng.in01()), shininess);
            }
            scene.take(marbleMaterial);
            scene.take(new Sphere(marbleMaterial, marbleAnimationIndex), marbleIsLightSource ? HotSpot : ColdSpot);
        }
    }
}

/* Main */
int main(int argc, char* argv[])
{
    if (argc != 1 && argc != 2 && argc != 3) {
        fprintf(stderr, "Usage: %s [configuration] [frame]\n", argv[0]);
        return 0;
    }
    int configuration = (argc >= 2 ? atoi(argv[1]) : -1);
    int frameno = (argc == 3 ? atoi(argv[2]) : -1);

    unsigned int width;
    unsigned int height;
    Camera::SurroundMode surroundMode = Camera::Surround_Off;
    float stereoscopicDistance = 0.0f;
    unsigned int samples_sqrt = 25;
    std::string marker;
    switch (configuration) {
    case 0: // 2d
        width = 3840;
        height = 2160;
        break;
    case 1: // 3d
        width = 3840;
        height = 2160 * 2;
        stereoscopicDistance = 0.07f;
        marker = "-tb";
        break;
    case 2: // 180° 2d
        width = 2160;
        height = 2160;
        surroundMode = Camera::Surround_180;
        marker = "-180";
        break;
    case 3: // 180° 3d
        width = 2160;
        height = 2160 * 2;
        surroundMode = Camera::Surround_180;
        stereoscopicDistance = 0.07f;
        marker = "-180-tb";
        break;
    case 4: // 360° 2d
        width = 4320;
        height = 2160;
        surroundMode = Camera::Surround_360;
        marker = "-360";
        break;
    case 5: // 360° 3d
        width = 4320;
        height = 2160 * 2;
        surroundMode = Camera::Surround_360;
        stereoscopicDistance = 0.07f;
        marker = "-360-tb";
        break;
    default: // preview 2d
        width = 1920;
        height = 1080;
        samples_sqrt = 10;
        break;
    }
    Parameters params;
    params.maxPathComponents = 6;

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(70.0f), (stereoscopicDistance > 0.0f ? 2.0f : 1.0f) * sensor.aspectRatio()));
    vec3 lookfrom(0.0f, 1.7f, 0.0f);
    vec3 lookat(0.0f, 1.7f, -1.0f);
    Camera camera(surroundMode, stereoscopicDistance, optics, Transformation::fromLookAt(lookfrom, lookat));

    float start = 0.0f;
    float end = 18.0f;
    float frameDuration = 1.0f / 25.0f;
    int frameCount = (end - start) / frameDuration;
    for (int frame = 0; frame < frameCount; frame++) {
        if (frameno >= 0 && frameno != frame)
            continue;
        else if (configuration < 0 && frame % 10 != 0)
            continue;
        char frameString[12];
        snprintf(frameString, sizeof(frameString), "%04d", frame);

        float t0 = start + frame * frameDuration;
        float t1 = t0 + 0.5f * frameDuration;
        //t1 = t0; // disabling motion blur for debugging
        scene.updateBVH(t0, t1);

        mcpt(sensor, camera, scene, samples_sqrt, t0, t1, params);
        const TGD::Array<float>& hdrImg = sensor.result();
        TGD::save(hdrImg, std::string("rolling-marbles-") + frameString + marker + ".tgd");
        TGD::Array<uint8_t> ppImg = toSRGB(scaleLuminance(hdrImg, 1.5f));
        TGD::save(ppImg, std::string("rolling-marbles-") + frameString + marker + ".png");

        //scene.exportToObj("rolling-marbles");
    }

    return 0;
}
