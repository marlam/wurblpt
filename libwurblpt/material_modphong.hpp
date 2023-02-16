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
#include "texture.hpp"
#include "sampler.hpp"


namespace WurblPT {

class MaterialModPhong final : public Material
{
public:
    vec4 diffuse;
    const Texture* diffuseTex;
    bool diffuseTexHasAlpha;

    vec4 specular;
    const Texture* specularTex;
    bool specularTexHasAlpha;

    float shininess;
    const Texture* shininessTex;

    float opacity;
    const Texture* opacityTex;
    float indexOfRefraction;
    vec4 transmissive;

    vec4 emissive;
    const Texture* emissiveTex;

    bool haveNIR;

    MaterialModPhong() :
        diffuse(0.0f),
        diffuseTex(nullptr),
        diffuseTexHasAlpha(false),
        specular(0.0f),
        specularTex(nullptr),
        specularTexHasAlpha(false),
        shininess(0.0f),
        shininessTex(nullptr),
        opacity(0.0f),
        opacityTex(nullptr),
        indexOfRefraction(1.0f),
        transmissive(0.0f),
        emissive(0.0f),
        emissiveTex(nullptr),
        haveNIR(true)
    {
    }

    MaterialModPhong(
            const vec4& dif, const Texture* difTex,
            const vec4& spc = vec4(0.0f), const Texture* spcTex = nullptr,
            float shi = 100.0f, const Texture* shiTex = nullptr,
            float opa = 1.0f, const Texture* opaTex = nullptr) : MaterialModPhong()
    {
        assert(all(lessThanEqual(diffuse + specular, vec4(1.0f))));
        diffuse = dif;
        diffuseTex = difTex;
        specular = spc;
        specularTex = spcTex;
        shininess = shi;
        shininessTex = shiTex;
        opacity = opa;
        opacityTex = opaTex;
    }

    MaterialModPhong(
            const vec3& dif, const Texture* difTex,
            const vec3& spc = vec3(0.0f), const Texture* spcTex = nullptr,
            float shi = 100.0f, const Texture* shiTex = nullptr,
            float opa = 1.0f, const Texture* opaTex = nullptr) :
        MaterialModPhong(
                vec4(dif, average(dif)), difTex,
                vec4(spc, average(spc)), spcTex,
                shi, shiTex,
                opa, opaTex)
    {
        haveNIR = false;
    }

    MaterialModPhong(
            const vec4& dif,
            const vec4& spc = vec4(0.0f),
            float shi = 100.0f,
            float opa = 1.0f,
            const Texture* difTex = nullptr,
            const Texture* spcTex = nullptr,
            const Texture* shiTex = nullptr,
            const Texture* opaTex = nullptr) :
        MaterialModPhong(dif, difTex, spc, spcTex, shi, shiTex, opa, opaTex)
    {
    }

    MaterialModPhong(
            const vec3& dif,
            const vec3& spc = vec3(0.0f),
            float shi = 100.0f,
            float opa = 1.0f,
            const Texture* difTex = nullptr,
            const Texture* spcTex = nullptr,
            const Texture* shiTex = nullptr,
            const Texture* opaTex = nullptr) :
        MaterialModPhong(vec4(dif, average(dif)), difTex, vec4(spc, average(spc)), spcTex, shi, shiTex, opa, opaTex)
    {
        haveNIR = false;
    }

    float opacityAt(const vec2& texcoords, float t) const
    {
        float opa;
        if (opacityTex) {
            opa = opacityTex->value(texcoords, t).r();
        } else if (diffuseTexHasAlpha) {
            opa = diffuseTex->value(texcoords, t).a();
        } else {
            opa = opacity;
        }
        return opa;
    }

    vec4 diffuseAt(const vec2& texcoords, float t) const
    {
        vec4 kd = diffuseTex ? diffuseTex->value(texcoords, t) : diffuse;
        if (!haveNIR)
            kd.a() = average(kd.rgb());
        return kd;
    }

    vec4 specularAt(const vec2& texcoords, float t) const
    {
        vec4 ks = specularTex ? specularTex->value(texcoords, t) : specular;
        if (specularTexHasAlpha)
            ks = vec4(mix(ks.rgb(), specular.rgb(), ks.a()), ks.a());
        if (!haveNIR)
            ks.a() = average(ks.rgb());
        return ks;
    }

    float shininessAt(const vec2& texcoords, float t) const
    {
        float s = shininess;
        if (shininessTex)
            s *= shininessTex->value(texcoords, t).r();
        return s;
    }

    vec4 emissiveAt(const vec2& texcoords, float t) const
    {
        vec4 ke = emissiveTex ? emissiveTex->value(texcoords, t) : emissive;
        if (!haveNIR)
            ke.a() = average(ke.rgb());
        return ke;
    }

    virtual vec4 emitted(const Ray& ray, const HitRecord& hit) const override
    {
        vec4 e(0.0f);
        if (!hit.backside) {
            e = emissiveAt(hit.texcoords, ray.time);
        }
        return e;
    }

    static vec4 attenuation(const vec3& n, const vec3& v, const vec3& l, const vec4& kd, const vec4& ks, float s, float cosTheta)
    {
        assert(abs(dot(n, n) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(v, v) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(l, l) - 1.0f) < dirSquaredLengthTolerance);
        assert(all(isfinite(kd)));
        assert(min(kd) >= 0.0f);
        assert(max(kd) <= 1.0f);
        assert(all(isfinite(ks)));
        assert(min(ks) >= 0.0f);
        assert(max(ks) <= 1.0f);
        assert(isfinite(s));
        assert(s > 0.0f);
        assert(s < 65536.0f /* arbitrary */);
        assert(cosTheta >= 0.0f);

        vec3 r = reflect(-l, n);
        float cosRV = max(dot(r, v), 0.0f);
        return (kd + 0.5f * ks * (s + 2.0f) * pow(cosRV, s)) * inv_pi * min(cosTheta, 1.0f);
    }

    static float specularProbability(const vec4& kd, const vec4& ks)
    {
        float skd = kd.r() + kd.g() + kd.b() + kd.a();
        float sks = ks.r() + ks.g() + ks.b() + ks.a();
        float sum = skd + sks + 1e-4f /* so that sum > 0 */;
        float p = sks / sum;
        return clamp(p, 0.1f, 0.9f);
    }

    static float pdfValue(const vec3& n, const vec3& v, const vec3& l, float s, float cosTheta, float specProb)
    {
        assert(abs(dot(n, n) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(v, v) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(l, l) - 1.0f) < dirSquaredLengthTolerance);

        float diffusePdfValue = cosTheta * inv_pi;

        // Note that we use reciprocity of the BRDF here and r is the reflected view, not reflected light!
        // See "Importance Sampling of the Phong Reflectance Model" by Jason Lawrence.
        // This is consistent with Ray Tracing Gems, chap. Sampling Transformations Zoo, Sec. 16.6.4,
        // but here theta is between reflect(-v,n) and l, not between reflect(-l, n) and v!
        vec3 r = reflect(-v, n);
        float cosRL = max(dot(r, l), 0.0f);
        float specularPdfValue = 0.5f * inv_pi * (s + 1.0f) * pow(cosRL, s);

        return mix(diffusePdfValue, specularPdfValue, specProb);
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& prng) const override
    {
        float opa = opacityAt(hit.texcoords, ray.time);
        bool transparent = (opa < 1.0f && opa < prng.in01());
        if (transparent) {
            float ourRI = indexOfRefraction;
            float theirRI = 1.0f;
            if (hit.backside)
                std::swap(ourRI, theirRI);
            vec3 n = normalize(normalAt(hit, ray.time));
            vec3 refracted = refract(ray.direction, n, theirRI / ourRI);
            // the following check is necessary, don't remove!
            float l = dot(refracted, refracted);
            if (l < epsilon)
                return ScatterRecord(ScatterNone);
            refracted /= sqrt(l);
            vec4 att = transmissive;
            if (!haveNIR)
                att.a() = average(att.rgb());
            return ScatterRecord(ScatterExplicit, refracted, att, ourRI);
        }

        if (hit.backside)
            return ScatterRecord(ScatterNone);

        vec4 kd = diffuseAt(hit.texcoords, ray.time);
        vec4 ks = specularAt(hit.texcoords, ray.time);
        float s = shininessAt(hit.texcoords, ray.time);
        float specProb = specularProbability(kd, ks);
        // to be filled by both the specular and diffuse case below:
        vec3 dir;
        vec3 n;
        float cosTheta;
        if (prng.in01() < specProb) {
            // specular
            // from Ray Tracing Gems, chap. Sampling Transformations Zoo, Sec. 16.6.4
            // note that the generated direction is around the perfect reflection direction,
            // so it can point below the surface, in which case we should set its contribution
            // to zero. See also "Importance Sampling of the Phong Reflectance Model" by J. Lawrence.
            float r1 = prng.in01();
            float r2 = prng.in01();
            float cosThetaSpec = pow(1.0f - r1, 1.0f / (1.0f + s));
            float discriminant = max(1.0f - cosThetaSpec * cosThetaSpec, 0.0f);
            float sinThetaSpec = sqrt(discriminant);
            float phi = 2.0f * pi * r2;
            float x = cos(phi) * sinThetaSpec;
            float y = sin(phi) * sinThetaSpec;
            float z = cosThetaSpec;
            n = normalAt(hit, ray.time);
            TangentSpace specTS = TangentSpace(reflect(ray.direction, n));
            dir = normalize(specTS.toWorldSpace(vec3(x, y, z)));
            assert(all(isfinite(dir)));
            cosTheta = max(dot(dir, n), 0.0f);
        } else {
            // diffuse
            TangentSpace ts = tangentSpaceAt(hit, ray.time);
            n = ts.normal;
            vec3 cosineDir = Sampler::cosineDirection(prng.in01x2());
            cosTheta = cosineDir.z(); // == dot(cosineDir, vec3(0.0f, 0.0f, 1.0f));
            dir = normalize(ts.toWorldSpace(cosineDir));
        }
        vec4 att = attenuation(n, -ray.direction, dir, kd, ks, s, cosTheta);
        assert(all(isfinite(att)));
        float p = pdfValue(n, -ray.direction, dir, s, cosTheta, specProb);
        assert(isfinite(p) && p >= 0.0f);

        return ScatterRecord(ScatterRandom, dir, att, p, ray.refractiveIndex);
    }

    virtual ScatterRecord scatterToDirection(const Ray& ray, const HitRecord& hit, const vec3& direction) const override
    {
        vec4 att(0.0f);
        float p = 0.0f;
        vec3 n = normalAt(hit, ray.time);
        float cosTheta = dot(n, direction);
        if (cosTheta > 0.0f) {
            vec4 kd = diffuseAt(hit.texcoords, ray.time);
            vec4 ks = specularAt(hit.texcoords, ray.time);
            float s = shininessAt(hit.texcoords, ray.time);
            float specProb = specularProbability(kd, ks);
            att = attenuation(n, -ray.direction, direction, kd, ks, s, cosTheta);
            assert(all(isfinite(att)));
            p = pdfValue(n, -ray.direction, direction, s, cosTheta, specProb);
            assert(isfinite(p) && p >= 0.0f);
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
        objMaterial.Kd = diffuse.rgb();
        objMaterial.Ks = specular.rgb();
        objMaterial.Ns = shininess;
        objMaterial.d = opacity;
        objMaterial.Ke = emissive.rgb();
        if (diffuseTex)
            objMaterial.map_Kd = diffuseTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        if (specularTex)
            objMaterial.map_Ks = specularTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        if (shininessTex)
            objMaterial.map_Ns = shininessTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        if (opacityTex)
            objMaterial.map_d = opacityTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        if (normalTex)
            objMaterial.norm = normalTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        if (emissiveTex)
            objMaterial.map_Ke = emissiveTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
    }
};

}
