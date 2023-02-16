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

#pragma once

#include "material.hpp"
#include "prng.hpp"
#include "sampler.hpp"


namespace WurblPT {

class MaterialLambertian final : public Material
{
private:
    const vec4 _albedo;
    const Texture* _tex;
    const bool haveNIR;

public:
    MaterialLambertian(const vec4& albedo, const Texture* tex = nullptr) :
        _albedo(albedo), _tex(tex), haveNIR(true)
    {
    }

    MaterialLambertian(const vec3& albedo, const Texture* tex = nullptr) :
        _albedo(albedo, 0.0f), _tex(tex), haveNIR(false)
    {
    }

    vec4 albedoAt(const vec2& texcoords, float t) const
    {
        vec4 a = _tex ? _tex->value(texcoords, t) : _albedo;
        if (!haveNIR)
            a.a() = average(a.rgb());
        return a;
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& prng) const override
    {
        if (hit.backside)
            return ScatterRecord(ScatterNone);

#if 1
        vec3 cosineDir = Sampler::cosineDirection(prng.in01x2());
        float cosTheta = cosineDir.z(); // == dot(cosineDir, vec3(0.0f, 0.0f, 1.0f));
        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        vec3 dir = normalize(ts.toWorldSpace(cosineDir));
        float p = cosTheta * inv_pi;
        vec4 att = albedoAt(hit.texcoords, ray.time) * p;
#else
        /* without material importance sampling; for testing only */
        vec3 hemDir = Sampler::onUnitHemisphere(prng.in01x2());
        float cosTheta = hemDir.z; // == dot(cosineDir, vec3(0.0f, 0.0f, 1.0f));
        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        vec3 dir = normalize(ts.toWorldSpace(hemDir));
        float p = 0.5f * inv_pi;
        vec4 att = albedoAt(hit.texcoords, ray.time) * cosTheta * inv_pi;
#endif

        return ScatterRecord(ScatterRandom, dir, att, p, ray.refractiveIndex);
    }

    virtual ScatterRecord scatterToDirection(const Ray& ray, const HitRecord& hit, const vec3& direction) const override
    {
        vec4 att(0.0f);
        float p = 0.0f;
        float cosTheta = dot(normalAt(hit, ray.time), direction);
        if (cosTheta > 0.0f) {
#if 1
            p = cosTheta * inv_pi;
            att = albedoAt(hit.texcoords, ray.time) * p;
#else
            /* without material importance sampling; for testing only */
            p = 0.5f * inv_pi;
            att = albedoAt(hit.texcoords, ray.time) * cosTheta * inv_pi;
#endif
        }
        return ScatterRecord(ScatterRandom, direction, att, p, ray.refractiveIndex);
    }

    virtual void exportToMtl(
            ObjMaterial& objMaterial,
            std::ostream& geometryOut, std::ostream& materialOut,
            unsigned int& globalVertexIndex,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        objMaterial.Kd = _albedo.rgb();
        if (_tex) {
            objMaterial.map_Kd = _tex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
        if (normalTex) {
            objMaterial.norm = normalTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
    }
};

}
