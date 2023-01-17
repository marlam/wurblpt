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
#include "tangentspace.hpp"
#include "fresnel.hpp"


namespace WurblPT {

class MaterialGGX final : public Material
{
public:
    vec4 albedo;
    vec2 roughness;
    const Texture* albedoTex;
    const Texture* roughnessTex;

    MaterialGGX(const vec4& alb, const Texture* albTex, const vec2& r, const Texture* rTex) :
        albedo(alb), roughness(r),
        albedoTex(albTex),
        roughnessTex(rTex)
    {
    }

    MaterialGGX() :
        MaterialGGX(vec4(0.0f), nullptr, vec2(0.0f), nullptr)
    {
    }

    MaterialGGX(const vec4& alb, const vec2& r, const Texture* albTex = nullptr, const Texture* rTex = nullptr) :
        MaterialGGX(alb, albTex, r, rTex)
    {
    }

    MaterialGGX(const vec3& alb, const Texture* albTex, const vec2& r, const Texture* rTex) :
        MaterialGGX(vec4(alb, average(alb)), albTex, r, rTex)
    {
    }

    MaterialGGX(const vec3& alb, const vec2& r, const Texture* albTex = nullptr, const Texture* rTex = nullptr) :
        MaterialGGX(vec4(alb, average(alb)), albTex, r, rTex)
    {
    }

    vec4 albedoAt(const vec2& texcoords, float t) const
    {
        vec4 alb = albedo;
        if (albedoTex) {
            alb = albedoTex->value(texcoords, t);
        }
        return alb;
    }

    vec2 roughnessAt(const vec2& texcoords, float t) const
    {
        vec2 r = roughness;
        if (roughnessTex) {
            r = vec2(roughnessTex->value(texcoords, t).rg());
        }
        return r;
    }

    static float Lambda(const vec3& tangentSpaceVector, const vec2& roughness)
    {
        // From "Sampling the GGX Distribution of Visible Normals" by E. Heitz,
        // JCGT vol 7 no 4 2018, Eq. 2
        vec2 a2 = roughness * roughness;
        vec3 tsv2 = tangentSpaceVector * tangentSpaceVector;
        float discriminant = 1.0f + (a2.x() * tsv2.x() + a2.y() * tsv2.y()) / tsv2.z();
        return 0.5f * (-1.0f + sqrt(discriminant));
    }

    static float G1(const vec3& tangentSpaceVector, const vec2& roughness)
    {
        // From "Sampling the GGX Distribution of Visible Normals" by E. Heitz,
        // JCGT vol 7 no 4 2018, Eq. 2
        return 1.0f / (1.0f + Lambda(tangentSpaceVector, roughness));
    }

    static float G2(const vec3& tangentSpaceV, const vec3& tangentSpaceL, const vec2& roughness)
    {
        // From "Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs"
        // by E. Heitz, Eq. 99, which is recommended over Eq. 98 that is used in
        // "Microfacet models for refraction through rough surfaces" by Walter et al.,
        // Proc. Eurographics Symposium on Rendering 2007, Sec. 5.1 Eq. 23.

        // Here we assume that dotLH and dotVH are > 0, so the ChiPlus functions are both 1.
        return 1.0f / (1.0f + Lambda(tangentSpaceV, roughness)
                            + Lambda(tangentSpaceL, roughness));
    }

    static float D(const vec3& tangentSpaceH, const vec2& roughness)
    {
        // From "Sampling the GGX Distribution of Visible Normals" by E. Heitz, JCGT vol 7 no 4 2018, Eq. 1
        vec2 a2 = roughness * roughness;
        vec3 tsh2 = tangentSpaceH * tangentSpaceH;
        float t = tsh2.x() / a2.x() + tsh2.y() / a2.y() + tsh2.z();
        float D = 1.0f / (pi * roughness.x() * roughness.y() * t * t);
        return D;
    }

    static float DV(const vec3& tangentSpaceH, const vec3& tangentSpaceV, float dotVH, const vec2& roughness)
    {
        // From "Sampling the GGX Distribution of Visible Normals" by E. Heitz,
        // JCGT vol 7 no 4 2018, Eq. 3
        // Here we assume dotVH > 0
        float dotVZ = tangentSpaceV.z(); // == dot(tangentSpaceV, vec3(0.0f, 0.0f, 1.0f));
        float DV = G1(tangentSpaceV, roughness) * dotVH * D(tangentSpaceH, roughness) / dotVZ;
        return DV;
    }

    static vec3 sampleVNDF(const vec3& tangentSpaceV, const vec2& roughness, Prng& prng)
    {
        // This is the code from "Sampling the GGX Distribution of Visible Normals"
        // by E. Heitz, JCGT vol 7 no 4 2018, Listing 1 (in Appendix A).
        // We first translate our variable names to the ones of E. Heitz so that we can
        // use his code with minimal modifications.
        // Note that the returned vector is in tangent space!
        const vec3& Ve = tangentSpaceV;
        float alpha_x = roughness.x();
        float alpha_y = roughness.y();
        float U1 = prng.in01();
        float U2 = prng.in01();

        // Code from listing 1:

        // Section 3.2: transforming the view direction to the hemisphere configuration
        vec3 Vh = normalize(vec3(alpha_x * Ve.x(), alpha_y * Ve.y(), Ve.z()));
        // Section 4.1: orthonormal basis (with special case if cross product is zero)
        float lensq = Vh.x() * Vh.x() + Vh.y() * Vh.y();
        vec3 T1 = lensq > 0.0f ? vec3(-Vh.y(), Vh.x(), 0.0f) * inversesqrt(lensq) : vec3(1.0f, 0.0f, 0.0f);
        vec3 T2 = cross(Vh, T1);
        // Section 4.2: parameterization of the projected area
        float r = sqrt(U1);
        float phi = 2.0f * pi * U2;
        float t1 = r * cos(phi);
        float t2 = r * sin(phi);
        float s = 0.5f * (1.0f + Vh.z());
        t2 = (1.0f - s) * sqrt(1.0f - t1 * t1) + s * t2;
        // Section 4.3: reprojection onto hemisphere
        vec3 Nh = t1 * T1 + t2 * T2 + sqrt(max(0.0f, 1.0f - t1 * t1 - t2 * t2)) * Vh;
        // Section 3.4: transforming the normal back to the ellipsoid configuration
        vec3 Ne = normalize(vec3(alpha_x * Nh.x(), alpha_y * Nh.y(), max(0.0f, Nh.z())));
        return Ne;
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& prng) const override
    {
        if (hit.backside)
            return ScatterRecord(ScatterNone);

        vec3 view = -ray.direction;
        vec2 roughness = roughnessAt(hit.texcoords, ray.time);
        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        vec3 tangentSpaceV = ts.toTangentSpace(view);
        vec3 tangentSpaceH = sampleVNDF(tangentSpaceV, roughness, prng);
        vec3 tangentSpaceL = reflect(-tangentSpaceV, tangentSpaceH);
        vec3 light = ts.toWorldSpace(tangentSpaceL);
        // the following check is necessary, don't remove!
        float l = dot(light, light);
        if (l < epsilon)
            return ScatterRecord(ScatterNone);
        vec3 dir = light / sqrt(l);

        // The angle between view and h is the same as the angle between light and h,
        // and we assume both are less than 90 degrees because h was sampled from the
        // visible normals. So dotLH==dotVH > 0.
        float dotVH = dot(tangentSpaceV, tangentSpaceH);

        // From "Sampling the GGX Distribution of Visible Normals" by E. Heitz,
        // JCGT vol 7 no 4 2018, Eq. 17
        float p = DV(tangentSpaceH, tangentSpaceV, dotVH, roughness) / (4.0f * dotVH);
        if (!isfinite(p) || p < 0.0f)
            return ScatterRecord(ScatterNone);

        // For pure material importance sampling, we would want to implement Eq. 19
        // from "Sampling the GGX Distribution of Visible Normals" by E. Heitz,
        // JCGT vol 7 no 4 2018, because in the term attenuation/pdfValue, several
        // costly terms cancel out and we would end up with a version that gives
        // low variance.
        // However here we need to compute attenuation and pdfValue separately
        // so that the result can be used in Multiple Importance Sampling, which
        // should be our default use case.
        // So we implement Eq. 15 from that paper instead (and multiply with
        // cos(theta_i)=dot(N,L), so that term cancels), which is also what we do in
        // scatterToDirection() below.
        vec4 att(0.0f);
        float dotNL = dot(ts.normal, light);
        float dotNV = dot(ts.normal, view);
        if (dotNL > 0.0f && dotNV > 0.0f) {
            float Dval = D(tangentSpaceH, roughness);
            vec4 albedo = albedoAt(hit.texcoords, ray.time);
            vec4 Fval = fresnelSchlick(albedo, dotVH);
            float Gval = G2(tangentSpaceV, tangentSpaceL, roughness);
            att = Dval * Fval * Gval / (4.0f * dotNV);
        }

        return ScatterRecord(ScatterRandom, dir, att, p, ray.refractiveIndex);
    }

    virtual ScatterRecord scatterToDirection(const Ray& ray, const HitRecord& hit, const vec3& direction) const override
    {
        vec4 att(0.0f);
        float p = 0.0f;
        vec3 view = -ray.direction;
        vec3 light = direction;
        TangentSpace ts = tangentSpaceAt(hit, ray.time);
        float dotNL = dot(ts.normal, light);
        float dotNV = dot(ts.normal, view);
        if (dotNL > 0.0f && dotNV > 0.0f) {
            // Preparations
            vec2 roughness = roughnessAt(hit.texcoords, ray.time);
            vec3 tangentSpaceV = ts.toTangentSpace(view);
            vec3 tangentSpaceL = ts.toTangentSpace(light);
            vec3 tangentSpaceH = normalize(tangentSpaceV + tangentSpaceL);
            float dotVH = dot(tangentSpaceV, tangentSpaceH);
            if (dotVH > 0.0f) {
                // pdf value
                // From "Sampling the GGX Distribution of Visible Normals" by E. Heitz,
                // JCGT vol 7 no 4 2018, Eq. 17
                p = DV(tangentSpaceH, tangentSpaceV, dotVH, roughness) / (4.0f * dotVH);
                // Attenuation
                float Dval = D(tangentSpaceH, roughness);
                vec4 albedo = albedoAt(hit.texcoords, ray.time);
                vec4 Fval = fresnelSchlick(albedo, dotVH);
                float Gval = G2(tangentSpaceV, tangentSpaceL, roughness);
                att = Dval * Fval * Gval / (4.0f * dotNV);
            }
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
        objMaterial.Ks = albedo.rgb();
        float r = 0.5f * (roughness.x() + roughness.y());
        objMaterial.Ns = 2.0f / (r * r * r * r) - 2.0f; // from http://graphicrants.blogspot.com/2013/08/specular-brdf-reference.html
        if (albedoTex) {
            objMaterial.map_Ks = albedoTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
        // shininess texture export does not make sense because we have roughness in the texture
        if (normalTex) {
            objMaterial.map_bump = normalTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
    }
};

}
