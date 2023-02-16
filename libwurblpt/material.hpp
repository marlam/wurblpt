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

#include "ray.hpp"
#include "hitable.hpp"
#include "texture.hpp"
#include "tangentspace.hpp"
#include "scene_component.hpp"


namespace WurblPT {

typedef enum {
    ScatterNone,            // no scattering; ray is not reflected
    ScatterExplicit,        // explicit scatter direction (no importance sampling possible)
    ScatterRandom           // random scatter direction for importance sampling
} ScatterType;

class ScatterRecord
{
public:
    ScatterType type;     // see above
    vec3 direction;       // the scatter direction
    vec4 attenuation;     // the attenuation for the direction
    float pdf;            // the value of the probability density function for the direction,
                          // if the direction is randomly generated
    vec4 refractiveIndex; // the refractive index (a new one if we entered a
                          // transparent object, otherwise the old one)

    ScatterRecord(ScatterType t) :
        type(t), direction(0.0f), attenuation(0.0f), pdf(0.0f), refractiveIndex(0.0f)
    {
        assert(t == ScatterNone);
    }

    ScatterRecord(ScatterType t, const vec3& dir, const vec4& att, const vec4& refrInd) :
        type(t), direction(dir), attenuation(att), pdf(0.0f), refractiveIndex(refrInd)
    {
        assert(t == ScatterExplicit);
        assert(all(isfinite(dir)));
        assert(abs(dot(dir, dir) - 1.0f) < dirSquaredLengthTolerance);
        assert(all(isfinite(attenuation)));
        assert(min(attenuation) >= 0.0f);
    }

    ScatterRecord(ScatterType t, const vec3& dir, const vec4& att, float p, const vec4& refrInd) :
        type(t), direction(dir), attenuation(att), pdf(p), refractiveIndex(refrInd)
    {
        assert(t == ScatterRandom);
        assert(all(isfinite(dir)));
        assert(all(equal(dir, vec3(0.0f))) || abs(dot(dir, dir) - 1.0f) < dirSquaredLengthTolerance);
        assert(all(isfinite(attenuation)));
        assert(min(attenuation) >= 0.0f);
        assert(isfinite(p) && p >= 0.0f);
    }
};

class ObjMaterial
{
public:
    vec3 Kd, Ks, Ke;
    float Ns, Ni, d;
    std::string map_Kd, map_Ks, map_Ke;
    std::string map_Ns, map_Ni, map_d;
    std::string norm;

    ObjMaterial() : Kd(0.0f), Ks(0.0f), Ke(0.0f), Ns(0.0f), Ni(1.0f), d(1.0f)
    {
    }

    void exportToMtl(std::ostream& materialOut)
    {
        materialOut << "Ka 0 0 0" << std::endl;
        // TODO: use std::format
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%.8g %.8g %.8g", Kd.r(), Kd.g(), Kd.b());
        materialOut << "Kd " << buf << std::endl;
        std::snprintf(buf, sizeof(buf), "%.8g %.8g %.8g", Ks.r(), Ks.g(), Ks.b());
        materialOut << "Ks " << buf << std::endl;
        std::snprintf(buf, sizeof(buf), "%.8g %.8g %.8g", Ke.r(), Ke.g(), Ke.b());
        materialOut << "Ke " << buf << std::endl;
        std::snprintf(buf, sizeof(buf), "%.8g", Ns);
        materialOut << "Ns " << buf << std::endl;
        std::snprintf(buf, sizeof(buf), "%.8g", Ni);
        materialOut << "Ni " << buf << std::endl;
        std::snprintf(buf, sizeof(buf), "%.8g", d);
        materialOut << "d " << buf << std::endl;
        if (map_Kd.size() > 0)
            materialOut << "map_Kd " << map_Kd << std::endl;
        if (map_Ks.size() > 0)
            materialOut << "map_Ks " << map_Ks << std::endl;
        if (map_Ke.size() > 0)
            materialOut << "map_Ke " << map_Ke << std::endl;
        if (map_Ns.size() > 0)
            materialOut << "map_Ns " << map_Ns << std::endl;
        if (map_Ni.size() > 0)
            materialOut << "map_Ni " << map_Ni << std::endl;
        if (map_d.size() > 0)
            materialOut << "map_d " << map_d << std::endl;
        if (norm.size() > 0)
            materialOut << "norm " << norm << std::endl;
    }
};

class Material : public SceneComponent
{
public:
    const Texture* normalTex;

    Material(const Texture* nt = nullptr) : normalTex(nt)
    {
    }

    virtual ~Material()
    {
    }

    /* Scatter the ray based on the given hit record.
     * This is the first function called by the tracer to find out which kind
     * of scatter event to use:
     * - None at all (ScatterNone)
     *   E.g. light sources. Requires no further information in the ScatterRecord.
     * - A non-random, explicit scatter ray (ScatterExplicit).
     *   Requires an explicit direction, and attenuation and index of refraction
     *   for it. Examples: glass, mirror.
     * - A random scatter ray tuned for importance sampling (ScatterRandom).
     *   This requires a randomly generated direction, attenuation, probability
     *   density function value, and index of refraction for that direction.
     * If the material returns only ScatterNone or ScatterExplicit records, it
     * does not need to implement the scatterToDirection() function below.
     * Note: typical opaque materials should return ScatterNone if their back side is
     * hit because that should not happen.
     */
    virtual ScatterRecord scatter(
            const Ray& /* ray */,
            const HitRecord& /* hit */,
            Prng& /* rnd */) const
    {
        return ScatterRecord(ScatterNone);
    }

    /* Scatter the ray based on the given hit record into the given direction.
     * This is required for importance sampling and only called when the material
     * returns ScatterRandom records from scatter().
     * It must return a ScatterRandom record with the exact same direction that is
     * given to this function, and appropriate values for attenuation, probability
     * density function, and index of refraction for that direction.
     */
    virtual ScatterRecord scatterToDirection(
            const Ray& /* ray */,
            const HitRecord& /* hit */,
            const vec3& /* direction */) const
    {
        return ScatterRecord(ScatterNone);
    }

    /* Light sources implement this function. It returns the emitted radiance in W/m^2. */
    virtual vec4 emitted(const Ray& /* ray */, const HitRecord& /* hit */) const
    {
        return vec4(0.0f);
    }

    /* Light sources for ToF cameras should return true here: */
    virtual bool isTofLight(const HitRecord& /* hit */) const
    {
        return false;
    }

    /* Get a normal from the normal map (if available).
     * Use this if you don't need a tangent space, otherwise use tangentSpaceAt(). */
    vec3 normalAt(const HitRecord& hit, float t) const
    {
        vec3 n = hit.normal;
        if (normalTex) {
            assert(dot(hit.tangent, hit.tangent) > epsilon);
            n = normalTex->value(hit.texcoords, t).rgb();
            n = 2.0f * n - vec3(1.0f);
            n = normalize(TangentSpace(hit.normal, hit.tangent).toWorldSpace(n));
        }
        return n;
    }

    /* Get a tangent space based on a normal that potentially comes from a normal map.
     * Use this if you need a tangent space and may also need the normal
     * (which is stored in the tangent space). */
    TangentSpace tangentSpaceAt(const HitRecord& hit, float t) const
    {
        TangentSpace ts;
        if (dot(hit.tangent, hit.tangent) > epsilon) {
            ts = TangentSpace(hit.normal, hit.tangent);
            if (normalTex) {
                // Get a normal from the map and convert it to world space
                vec3 n = normalTex->value(hit.texcoords, t).rgb();
                n = 2.0f * n - vec3(1.0f);
                n = normalize(ts.toWorldSpace(n));
                // Update the tangent space for the new normal via Gram-Schmidt orthonormalization:
                vec3 t = normalize(hit.tangent - dot(n, hit.tangent) * n);
                ts = TangentSpace(n, t);
            }
        } else {
            ts = TangentSpace(hit.normal);
        }
        return ts;
    }

    /* Export a material to OBJ. This is the general exporter that implements the SceneComponent
     * interface. Custom materials need only implement the exportToMtl() function below and
     * this function will handle the rest. */
    virtual std::string exportToObj(
            std::ostream& geometryOut, std::ostream& materialOut,
            unsigned int& globalVertexIndex,
            bool sceneExportTopLevel,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        if (sceneExportTopLevel)
            return std::string();

        auto it = sceneExportCache.find(this);
        if (it != sceneExportCache.end())
            return it->second;

        char buf[128];
        std::snprintf(buf, sizeof(buf), "%p", this); // TODO: use std::format when it becomes available
        std::string myName = std::string("material_") + buf;

        materialOut << std::endl << "newmtl " << myName << std::endl;
        ObjMaterial objMaterial;
        exportToMtl(objMaterial, geometryOut, materialOut, globalVertexIndex, animationCache, basePath, baseName, sceneExportCache);
        objMaterial.exportToMtl(materialOut);

        sceneExportCache.insert(std::pair<const SceneComponent*, std::string>(this, myName));
        return myName;
    }

    /* Materials that can be exported to OBJ should implement this function: */
    virtual void exportToMtl(
            ObjMaterial& /* objMaterial */,
            std::ostream& /* geometryOut */, std::ostream& /* materialOut */,
            unsigned int& /* globalVertexIndex */,
            AnimationCache& /* animationCache */,
            const std::filesystem::path& /* basePath */, const std::string& /* baseName */,
            std::map<const SceneComponent*, std::string>& /* sceneExportCache */) const
    {
    }
};

class MaterialTwoSided final : public Material
{
private:
    const Material* _frontSideMaterial;
    const Material* _backSideMaterial;

    static HitRecord toFrontSide(const HitRecord& hit)
    {
        return HitRecord(hit.a, hit.position, hit.normal, hit.tangent, hit.texcoords, false, hit.hitable);
    }

public:
    MaterialTwoSided(const Material* frontSideMaterial, const Material* backSideMaterial) :
        Material(nullptr), _frontSideMaterial(frontSideMaterial), _backSideMaterial(backSideMaterial)
    {
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& rnd) const override
    {
        assert(hit.haveHit);
        return (hit.backside
                ? _backSideMaterial->scatter(ray, toFrontSide(hit), rnd)
                : _frontSideMaterial->scatter(ray, hit, rnd));
    }

    virtual ScatterRecord scatterToDirection(const Ray& ray, const HitRecord& hit, const vec3& direction) const override
    {
        assert(hit.haveHit);
        return (hit.backside
                ? _backSideMaterial->scatterToDirection(ray, toFrontSide(hit), direction)
                : _frontSideMaterial->scatterToDirection(ray, hit, direction));
    }

    virtual vec4 emitted(const Ray& ray, const HitRecord& hit) const override
    {
        assert(hit.haveHit);
        return (hit.backside
                ? _backSideMaterial->emitted(ray, toFrontSide(hit))
                : _frontSideMaterial->emitted(ray, hit));
    }

    virtual bool isTofLight(const HitRecord& hit) const override
    {
        assert(hit.haveHit);
        return (hit.backside
                ? _backSideMaterial->isTofLight(toFrontSide(hit))
                : _frontSideMaterial->isTofLight(hit));
    }

    virtual void exportToMtl(
            ObjMaterial& objMaterial,
            std::ostream& geometryOut, std::ostream& materialOut,
            unsigned int& globalVertexIndex,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        /* we only export the front side material since OBJ does not know two-sided materials */
        _frontSideMaterial->exportToMtl(objMaterial,
                geometryOut, materialOut, globalVertexIndex, animationCache, basePath, baseName, sceneExportCache);
    }
};

}
