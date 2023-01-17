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


namespace WurblPT {

/* This handles ToF light sources just like normal diffuse area light sources are handled
 * in all path tracers. This is better than the old approach which assumed a point light
 * source at all points on the surface and applied the inverse square law. */

class LightTof final : public Material
{
private:
    float _radiance;
    float _cosHalfOpeningAngle;
    const Texture* _tex;

public:
    LightTof(float radiance,
            float openingAngle,
            const Texture* tex = nullptr) :
        _radiance(radiance),
        _cosHalfOpeningAngle(cos(0.5f * openingAngle)),
        _tex(tex)
    {
        assert(openingAngle < pi);
    }

    virtual vec4 emitted(const Ray& ray, const HitRecord& hit) const override
    {
        vec4 e(0.0f);
        if (!hit.backside) {
            float cosine = dot(hit.normal, -ray.direction);
            if (cosine >= _cosHalfOpeningAngle) {
                // inside opening angle
                e.w() = _radiance;
                if (_tex) {
                    e.w() *= _tex->value(hit.texcoords, ray.time).r();
                }
            }
        }
        return e;
    }

    virtual bool isTofLight(const HitRecord& /* hit */) const override
    {
        return true;
    }

    virtual void exportToMtl(
            ObjMaterial& objMaterial,
            std::ostream& geometryOut, std::ostream& materialOut,
            unsigned int& globalVertexIndex,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        objMaterial.Ke = vec3(_radiance);
        if (_tex) {
            objMaterial.map_Ke = _tex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
        }
    }
};

}
