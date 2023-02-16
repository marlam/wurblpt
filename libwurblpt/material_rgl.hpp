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
#include "color.hpp"

#include "powitacq.h"
#undef POWITACQ_NAMESPACE_BEGIN
#undef POWITACQ_NAMESPACE_END
#undef POWITACQ_NAMESPACE_DIM
#include "powitacq_rgb.h"
#undef POWITACQ_NAMESPACE_BEGIN
#undef POWITACQ_NAMESPACE_END
#undef POWITACQ_NAMESPACE_DIM


namespace WurblPT {

/* This version takes an RGB dataset from the RGL material database and simply fakes the near-infrared channel
 * by averaging RGB. See below for a version that uses the spectral datasets. */
class MaterialRGL final : public Material
{
private:
    const powitacq_rgb::BRDF brdf;

    static powitacq_rgb::Vector2f toVec2f(const vec2& v) { return powitacq_rgb::Vector2f(v.x(), v.y()); }
    static powitacq_rgb::Vector3f toVec3f(const vec3& v) { return powitacq_rgb::Vector3f(v.x(), v.y(), v.z()); }
    static vec3 fromVec3f(const powitacq_rgb::Vector3f& v) { return vec3(v.x(), v.y(), v.z()); }

public:
    MaterialRGL(const std::string& filenameRgb) : brdf(filenameRgb)
    {
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& prng) const override
    {
        if (hit.backside)
            return ScatterRecord(ScatterNone);

        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        // note that with RGL, wi seems to be incoming ray direction and not incoming light direction,
        // because their sample() function generates wo, which must be the outgoing ray direction.
        vec3 wi = ts.toTangentSpace(-ray.direction);
        vec2 u = prng.in01x2();
        powitacq_rgb::Vector3f powitacq_wo;
        float p;
        vec3 attenuation = fromVec3f(brdf.sample(toVec2f(u), toVec3f(wi), &powitacq_wo, &p));
        vec3 wo = fromVec3f(powitacq_wo);
        if (dot(wo, wo) <= 0.0f)
            return ScatterRecord(ScatterNone);
        vec4 att = vec4(attenuation, average(attenuation));
        att *= p; // sample() returns f * cos / pdf; undo the division here
        vec3 dir = normalize(ts.toWorldSpace(wo));

        return ScatterRecord(ScatterRandom, dir, att, p, ray.refractiveIndex);
    }

    virtual ScatterRecord scatterToDirection(const Ray& ray, const HitRecord& hit, const vec3& direction) const override
    {
        vec4 att(0.0f);
        float p = 0.0f;
        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        if (dot(ts.normal, direction) > 0.0f) {
            // note that with RGL, wi seems to be incoming ray direction and not incoming light direction,
            // because their sample() function generates wo, which must be the outgoing ray direction.
            vec3 wo = ts.toTangentSpace(direction);
            vec3 wi = ts.toTangentSpace(-ray.direction);
            vec3 attenuation = fromVec3f(brdf.eval(toVec3f(wi), toVec3f(wo)));
            att = vec4(attenuation, average(attenuation));
            p = brdf.pdf(toVec3f(wi), toVec3f(wo));
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
        vec3 n = vec3(0.0f, 0.0f, 1.0f);
        objMaterial.Kd = fromVec3f(brdf.eval(toVec3f(-n), toVec3f(n)));
        if (normalTex) {
            objMaterial.norm = normalTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
    }
};

/* This version takes a spectral dataset from the RGL material database, integrates the spectrum to get
 * RGB attenuation, and uses the spectrum sample that is nearest to the specified wavelength for the
 * near-infrared channel. To save computation time, you can switch off RGB integration; in this case
 * the value from the near-infrared channel will also be used for R, G and B. */
class MaterialRGLSpectral final : public Material
{
private:
    const powitacq::BRDF brdf;
    bool integrateRgb;
    constexpr static float visibleWaveLengthMin = 360.0f;
    constexpr static float visibleWaveLengthMax = 780.0f;
    size_t visibleWaveLengthsFirstIndex;
    size_t visibleWaveLengthsLastIndex;
    size_t nirWavelengthNearestIndex;

    static powitacq::Vector2f toVec2f(const vec2& v) { return powitacq::Vector2f(v.x(), v.y()); }
    static powitacq::Vector3f toVec3f(const vec3& v) { return powitacq::Vector3f(v.x(), v.y(), v.z()); }
    static vec3 fromVec3f(const powitacq::Vector3f& v) { return vec3(v.x(), v.y(), v.z()); }

    vec4 toAttenuation(const powitacq::Spectrum& attenuationSpectrum) const
    {
        vec3 xyz(0.0f);
        float N = 0.0f;
        for (size_t i = visibleWaveLengthsFirstIndex; i <= visibleWaveLengthsLastIndex; i++) {
            float lambda = brdf.wavelengths()[i];
            float I = d65(lambda);
            vec3 cmf = color_matching_function(lambda);
            N += I * cmf.y();
            xyz += attenuationSpectrum[i] * I * cmf;
        }
        float integrationFactor = (brdf.wavelengths()[visibleWaveLengthsLastIndex] - brdf.wavelengths()[visibleWaveLengthsFirstIndex])
            / (visibleWaveLengthsLastIndex - visibleWaveLengthsFirstIndex + 1);
        xyz *= integrationFactor;
        N *= integrationFactor;
        xyz *= 100.0f / N;
        vec4 attenuation = vec4(xyz_to_rgb(xyz), attenuationSpectrum[nirWavelengthNearestIndex]);
        return attenuation;
    }

public:
    MaterialRGLSpectral(const std::string& filename, float nirWavelength = 850.0f, bool integrateRgb = true) :
        brdf(filename), integrateRgb(integrateRgb)
    {
        const powitacq::Spectrum& wavelengths = brdf.wavelengths();
#if 0
        for (size_t i = 0; i < wavelengths.size(); i++) {
            fprintf(stderr, "%zu: %g\n", i, wavelengths[i]);
        }
#endif
        visibleWaveLengthsFirstIndex = 0;
        visibleWaveLengthsLastIndex = wavelengths.size() - 1;
        float minDistToNirWavelength;
        for (size_t i = 0; i < wavelengths.size(); i++) {
            if (wavelengths[i] >= visibleWaveLengthMin && (i == 0 || wavelengths[i - 1] < visibleWaveLengthMin))
                visibleWaveLengthsFirstIndex = i;
            if (wavelengths[i] <= visibleWaveLengthMax && (i == wavelengths.size() - 1 || wavelengths[i + 1] > visibleWaveLengthMax))
                visibleWaveLengthsLastIndex = i;
            float distToNirWavelength = abs(wavelengths[i] - nirWavelength);
            if (i == 0) {
                minDistToNirWavelength = distToNirWavelength;
                nirWavelengthNearestIndex = i;
            } else if (distToNirWavelength < minDistToNirWavelength) {
                minDistToNirWavelength = distToNirWavelength;
                nirWavelengthNearestIndex = i;
            }
        }
#if 0
        fprintf(stderr, "visible range: %zu (%g) - %zu (%g); nearest to %g: %zu (%g)\n",
                visibleWaveLengthsFirstIndex, wavelengths[visibleWaveLengthsFirstIndex],
                visibleWaveLengthsLastIndex, wavelengths[visibleWaveLengthsLastIndex],
                nirWavelength, nirWavelengthNearestIndex, wavelengths[nirWavelengthNearestIndex]);
#endif
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& prng) const override
    {
        if (hit.backside)
            return ScatterRecord(ScatterNone);

        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        // note that with RGL, wi seems to be incoming ray direction and not incoming light direction,
        // because their sample() function generates wo, which must be the outgoing ray direction.
        vec3 wi = ts.toTangentSpace(-ray.direction);
        vec2 u = prng.in01x2();
        powitacq::Vector3f powitacq_wo;
        vec3 wo;
        float p;
        vec4 att;
        if (integrateRgb) {
            powitacq::Spectrum attenuationSpectrum = brdf.sample(toVec2f(u), toVec3f(wi), &powitacq_wo, &p);
            wo = fromVec3f(powitacq_wo);
            if (dot(wo, wo) <= 0.0f)
                return ScatterRecord(ScatterNone);
            attenuationSpectrum *= p; // sample() returns f * cos / pdf; undo the division here
            att = toAttenuation(attenuationSpectrum);
        } else {
            float attenuationNIR = brdf.sample(nirWavelengthNearestIndex, toVec2f(u), toVec3f(wi), &powitacq_wo, &p);
            wo = fromVec3f(powitacq_wo);
            if (dot(wo, wo) <= 0.0f)
                return ScatterRecord(ScatterNone);
            att = vec4(attenuationNIR * p);
        }
        vec3 dir = normalize(ts.toWorldSpace(wo));

        return ScatterRecord(ScatterRandom, dir, att, p, ray.refractiveIndex);
    }

    virtual ScatterRecord scatterToDirection(const Ray& ray, const HitRecord& hit, const vec3& direction) const override
    {
        vec4 att(0.0f);
        float p = 0.0f;
        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        if (dot(ts.normal, direction) > 0.0f) {
            // note that with RGL, wi seems to be incoming ray direction and not incoming light direction,
            // because their sample() function generates wo, which must be the outgoing ray direction.
            vec3 wo = ts.toTangentSpace(direction);
            vec3 wi = ts.toTangentSpace(-ray.direction);
            if (integrateRgb) {
                powitacq::Spectrum attenuationSpectrum = brdf.eval(toVec3f(wi), toVec3f(wo));
                att = toAttenuation(attenuationSpectrum);
            } else {
                float attenuationNIR = brdf.eval(nirWavelengthNearestIndex, toVec3f(wi), toVec3f(wo));
                att = vec4(attenuationNIR);
            }
            p = brdf.pdf(toVec3f(wi), toVec3f(wo));
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
        vec3 n = vec3(0.0f, 0.0f, 1.0f);
        powitacq::Spectrum attenuationSpectrum = brdf.eval(toVec3f(-n), toVec3f(n));
        vec4 att = toAttenuation(attenuationSpectrum);
        objMaterial.Kd = att.rgb();
        if (normalTex) {
            objMaterial.norm = normalTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
    }
};

}
