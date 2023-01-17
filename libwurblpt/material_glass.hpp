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
#include "fresnel.hpp"
#include "constants.hpp"


namespace WurblPT {

class MaterialGlass final : public Material
{
private:
    static vec4 toVec4(const vec3& v)
    {
        return vec4(v, average(v));
    }

    static bool allComponentsEqual(const vec4& v)
    {
        return (v.r() == v.g()) && (v.r() == v.b()) && (v.r() == v.a());
    }

public:
    const vec4 absorption;
    const vec4 refractiveIndexOfMaterial;
    const vec4 refractiveIndexOfSurroundingMedium;
    const bool chromaticDispersion;

    MaterialGlass(
            const vec4& absorption,
            const vec4& refractiveIndexOfMaterial,
            const vec4& refractiveIndexOfSurroundingMedium = vec4(refractiveIndexOfVacuum)) :
        Material(nullptr),
        absorption(absorption),
        refractiveIndexOfMaterial(refractiveIndexOfMaterial),
        refractiveIndexOfSurroundingMedium(refractiveIndexOfSurroundingMedium),
        chromaticDispersion(!allComponentsEqual(refractiveIndexOfMaterial)
                || !allComponentsEqual(refractiveIndexOfSurroundingMedium))
    {
    }

    MaterialGlass(
            const vec4& absorption,
            float refractiveIndexOfMaterial,
            float refractiveIndexOfSurroundingMedium = refractiveIndexOfVacuum) :
        MaterialGlass(absorption, vec4(refractiveIndexOfMaterial), vec4(refractiveIndexOfSurroundingMedium))
    {
    }

    MaterialGlass(
            const vec3& absorption,
            const vec3& refractiveIndexOfMaterial,
            const vec3& refractiveIndexOfSurroundingMedium = vec3(refractiveIndexOfVacuum)) :
        MaterialGlass(toVec4(absorption), toVec4(refractiveIndexOfMaterial), toVec4(refractiveIndexOfSurroundingMedium))
    {
    }

    MaterialGlass(
            const vec3& absorption,
            float refractiveIndexOfMaterial,
            float refractiveIndexOfSurroundingMedium = refractiveIndexOfVacuum) :
        MaterialGlass(toVec4(absorption), vec4(refractiveIndexOfMaterial), vec4(refractiveIndexOfSurroundingMedium))
    {
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& prng) const override
    {
        vec4 att(1.0f);
        vec4 ourRI = refractiveIndexOfMaterial;
        vec4 theirRI = refractiveIndexOfSurroundingMedium;
        int riIndex = 0;
        if (chromaticDispersion) {
            riIndex = prng.in01() * 4;
            vec4 atts[4] = {
                vec4(4.0f, 0.0f, 0.0f, 0.0f),
                vec4(0.0f, 4.0f, 0.0f, 0.0f),
                vec4(0.0f, 0.0f, 4.0f, 0.0f),
                vec4(0.0f, 0.0f, 0.0f, 4.0f)
            };
            att = atts[riIndex];
        }
        if (hit.backside) {
            std::swap(ourRI, theirRI);
            float distInVolume = hit.a;
            att *= exp(-absorption * distInVolume);
        }
        // If we want to add a surface color: att *= surfaceColorAt(hit.texcoords, ray.time);

        vec3 n = normalAt(hit, ray.time);
        vec3 refracted = refract(ray.direction, n, theirRI[riIndex] / ourRI[riIndex]);

        bool doReflection = true;
        if (dot(refracted, refracted) > 0.0f) { // so no total internal reflection
            // For a more detailed explanation see Section 3.1 of "Extending the
            // Disney BRDF to a BSDF with Integrated Subsurface Scattering",
            // Brent Burley, Walt Disney Animation Studios, 2015
#if 0
            // Compute Fresnel using the Schlick approximation. If the ray is
            // transmitted into a medium with lower refractive index (e.g. when
            // transitioning out of glass an into air) the Schick approximation
            // has to be computed using the angle of the refracted direction
            // instead of the incident direction.
            float r0 = fresnelSchlickR0(ourRI[riIndex], theirRI[riIndex]);
            float cosTheta = (theirRI[riIndex] > ourRI[riIndex])
                ? -dot(refracted, n)
                : dot(-ray.direction, n);
            float fresnel = fresnelSchlick(r0, cosTheta);
#else
            // Compute Fresnel using the Fresnel equations. This is more
            // precise, especially if the difference between the two indices of
            // refraction is very small.
            float cosIncident = dot(-ray.direction, n);
            float cosTransmitted = -dot(refracted, n);
            float fresnel = fresnelUnpolarized(cosIncident, cosTransmitted, theirRI[riIndex], ourRI[riIndex]);
#endif
            doReflection = prng.in01() < fresnel;
        }

        if (doReflection) {
            // reflection
            vec3 reflected = reflect(ray.direction, n);
            return ScatterRecord(ScatterExplicit, normalize(reflected), att, theirRI);
        } else {
            // refraction
            return ScatterRecord(ScatterExplicit, normalize(refracted), att, ourRI);
        }
    }

    static float transparentColorToAbsorption(float transparentColor, float targetDistance = 0.01f /* 1cm */)
    {
        return max(- log(transparentColor) / targetDistance, 0.0f);
    }

    static vec3 transparentColorToAbsorption(const vec3& transparentColor, float targetDistance = 0.01f /* 1cm */)
    {
        return vec3(
                transparentColorToAbsorption(transparentColor.r(), targetDistance),
                transparentColorToAbsorption(transparentColor.g(), targetDistance),
                transparentColorToAbsorption(transparentColor.b(), targetDistance));
    }

    static vec4 transparentColorToAbsorption(const vec4& transparentColor, float targetDistance = 0.01f /* 1cm */)
    {
        return vec4(
                transparentColorToAbsorption(transparentColor.r(), targetDistance),
                transparentColorToAbsorption(transparentColor.g(), targetDistance),
                transparentColorToAbsorption(transparentColor.b(), targetDistance),
                transparentColorToAbsorption(transparentColor.a(), targetDistance));
    }

    static float absorptionToTransparentColor(float absorption, float targetDistance = 0.01f)
    {
        return exp(-absorption / targetDistance);
    }

    static vec3 absorptionToTransparentColor(const vec3& absorption, float targetDistance = 0.01f)
    {
        return vec3(
                absorptionToTransparentColor(absorption.r(), targetDistance),
                absorptionToTransparentColor(absorption.g(), targetDistance),
                absorptionToTransparentColor(absorption.b(), targetDistance));
    }

    virtual void exportToMtl(
            ObjMaterial& objMaterial,
            std::ostream& geometryOut, std::ostream& materialOut,
            unsigned int& globalVertexIndex,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        objMaterial.Kd = absorptionToTransparentColor(absorption.rgb());
        objMaterial.d = 0.0f;
        objMaterial.Ni = average(vec3(refractiveIndexOfMaterial.rgb()));
        if (normalTex) {
            objMaterial.map_bump = normalTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
    }
};

}
