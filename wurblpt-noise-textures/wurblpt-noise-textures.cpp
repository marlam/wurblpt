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

class NoiseBasedTexture : public Texture
{
public:
    const Texture* noiseTex;
    const Texture* worleyTex;
    static constexpr float minFreq = 1.0f;
    static constexpr int freqs = 6;

    NoiseBasedTexture(const Texture* noiseTex, const Texture* worleyTex) :
        noiseTex(noiseTex), worleyTex(worleyTex)
    {
    }

    /* effects based on gradient noise */

    vec4 noise(const vec2& tc, float t) const
    {
        return vec4(0.5f * (noiseTex->value(tc, t).x() + 1.0f));
    }

    vec4 fractalsum(const vec2& tc, float t) const
    {
        float amp = 0.6f;
        float freq = minFreq;
        float value = 0.0f;
        for (int i = 0; i < freqs; i++) {
            value += amp * 0.5f * (noiseTex->value(tc * freq, t).x() + 1.0f);
            freq *= 2.0f;
            amp *= 0.5f;
        }
        return vec4(value);
    }

    vec4 turbulence(const vec2& tc, float t) const
    {
        float amp = 0.8f;
        float freq = minFreq;
        float value = 0.0f;
        for (int i = 0; i < freqs; i++) {
            value += amp * std::abs(noiseTex->value(tc * freq, t).x());
            freq *= 2.0f;
            amp *= 0.5f;
        }
        return vec4(value);
    }

    vec4 disturbance(const vec2&tc, float t) const
    {
        float v = tc.y();
        v = std::min(1.0f, std::max(0.0f, v + 0.2f * noiseTex->value(tc, t).x()));
        v = std::min(1.0f, std::max(0.0f, v + 0.2f * noiseTex->value(vec2(tc.x(), v), t).x()));
        return vec4(v);
    }

    vec4 marble(const vec2& tc, float t) const
    {
        vec2 uv = 0.2f * tc;
        float v = std::sin(100.0f * uv.y() + 10.0f * turbulence(uv, t).x());
        return vec4(0.5f * (v + 1.0f));
    }

    /* effects based on worley noise */

    vec4 F1(const vec2& tc, float t) const
    {
        float v = worleyTex->value(tc, t).x();
        v = std::min(1.0f, v);
        return vec4(v);
    }

    vec4 F2(const vec2& tc, float t) const
    {
        float v = worleyTex->value(tc, t).y();
        v = std::min(1.0f, v);
        return vec4(v);
    }

    vec4 F3(const vec2& tc, float t) const
    {
        float v = worleyTex->value(tc, t).z();
        v = std::min(1.0f, v);
        return vec4(v);
    }

    vec4 F2F1(const vec2& tc, float t) const
    {
        vec4 w = worleyTex->value(tc, t);
        float v = w.y() - w.x();
        v = std::min(1.0f, v);
        return vec4(v);
    }

    vec4 crumple(const vec2& tc, float t) const
    {
        float amp = 1.0f;
        float freq = 1.0f;
        float value = 0.0f;
        for (int i = 0; i < 4; i++) {
            value += amp * F1(freq * tc, t).x();
            amp *= 0.5f;
            freq *= 2.0f;
        }
        return vec4(value);
    }

    /* choose one effect based on time t */

    virtual vec4 value(const vec2& tc, float t) const override
    {
        if (t < 1.0f) {
            return noise(tc, t);
        } else if (t < 2.0f) {
            return fractalsum(tc, t);
        } else if (t < 3.0f) {
            return turbulence(tc, t);
        } else if (t < 4.0f) {
            return disturbance(tc, t);
        } else if (t < 5.0f) {
            return marble(tc, t);
        } else if (t < 6.0f) {
            return F1(tc, t);
        } else if (t < 7.0f) {
            return F2(tc, t);
        } else if (t < 8.0f) {
            return F3(tc, t);
        } else if (t < 9.0f) {
            return F2F1(tc, t);
        } else {
            return crumple(tc, t);
        }
    }
};

void createScene(Scene& scene)
{
    Prng prng(137);

    Texture* noiseTex = scene.take(new TextureGradientNoise(16, 16, prng));
    Texture* worleyTex = scene.take(new TextureWorleyNoise(32, prng));
    Texture* diffuseTex = scene.take(new NoiseBasedTexture(noiseTex, worleyTex));

    MaterialModPhong* cubeMaterial = new MaterialModPhong(vec3(0.5f), vec3(0.5f), 120.0f);
    cubeMaterial->diffuseTex = diffuseTex;
    scene.take(cubeMaterial);

    Texture* sphereDiffuseTex = scene.take(new TextureTransformer(diffuseTex, vec2(6.0f, 3.0f)));

    MaterialModPhong* sphereMaterial = new MaterialModPhong(vec3(0.5f), vec3(0.5f), 120.0f);
    sphereMaterial->diffuseTex = sphereDiffuseTex;
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
    unsigned int width  = 800;
    unsigned int height = 600;
    unsigned int samples_sqrt = 10;

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(37.0f), sensor.aspectRatio()));
    Camera camera(optics, Transformation(vec3(0.0f, 0.0f, 5.0f)));

    scene.updateBVH();
    for (int t = 0; t < 10; t++) {
        mcpt(sensor, camera, scene, samples_sqrt, t, t);
        const TGD::Array<float>& hdrImg = sensor.result();
        TGD::Array<uint8_t> ldrImg = toSRGB(hdrImg);
        TGD::save(ldrImg, "noise-example-" + std::to_string(t) + ".png");
    }

    return 0;
}
