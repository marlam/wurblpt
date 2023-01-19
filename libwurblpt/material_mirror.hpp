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

// This is a special material for a perfect mirror.
// In general, it is better to use a GGX material with appropriate parameters.

class MaterialMirror final : public Material
{
public:
    const vec4 surfaceColor;
    const Texture* surfaceColorTex;
    const bool haveNIR;

    MaterialMirror(const vec4& surfaceColor = vec4(1.0f), const Texture* tex = nullptr) :
        Material(nullptr), surfaceColor(surfaceColor), surfaceColorTex(tex), haveNIR(true)
    {
    }

    MaterialMirror(const vec3& surfaceColor, const Texture* tex = nullptr) :
        Material(nullptr), surfaceColor(surfaceColor, average(surfaceColor)), surfaceColorTex(tex), haveNIR(false)
    {
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& hit, Prng& /* rnd */) const override
    {
        if (hit.backside)
            return ScatterRecord(ScatterNone);
        vec3 reflected = reflect(ray.direction, normalAt(hit, ray.time));
        vec4 att = surfaceColorTex ? surfaceColorTex->value(hit.texcoords, ray.time) : surfaceColor;
        if (!haveNIR)
            att.a() = average(att.rgb());
        return ScatterRecord(ScatterExplicit, normalize(reflected), att, ray.refractiveIndex);
    }

    virtual void exportToMtl(
            ObjMaterial& objMaterial,
            std::ostream& geometryOut, std::ostream& materialOut,
            unsigned int& globalVertexIndex,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        objMaterial.Ks = surfaceColor.rgb();
        float r = 0.001; // roughness for GGX that comes close to mirror
        objMaterial.Ns = 2.0f / (r * r * r * r) - 2.0f; // from http://graphicrants.blogspot.com/2013/08/specular-brdf-reference.html
        if (surfaceColorTex)
            objMaterial.map_Ks = surfaceColorTex->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
    }
};

}
