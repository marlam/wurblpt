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

#include "scene_component.hpp"
#include "hitable_sphere.hpp"
#include "material.hpp"
#include "generator.hpp"


namespace WurblPT {

class Sphere : public SceneComponent
{
private:
    const Material* _material;
    const Transformation _transformation;
    const int _animationIndex;

public:
    Sphere(const Material* material, const Transformation& T) :
        _material(material), _transformation(T), _animationIndex(-1)
    {
    }

    Sphere(const Material* material, int animationIndex = -1) :
        _material(material), _transformation(), _animationIndex(animationIndex)
    {
    }

    Sphere(const vec3& center, float radius, const Material* material) :
        Sphere(material, Transformation(center, quat::null(), vec3(radius)))
    {
    }

    virtual std::vector<Hitable*> createHitables() const override
    {
        return std::vector<Hitable*>(1, new HitableSphere(_transformation, _material, _animationIndex));
    }

    virtual std::string exportToObj(
            std::ostream& geometryOut, std::ostream& materialOut,
            unsigned int& globalVertexIndex,
            bool /* sceneExportTopLevel */,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%p", this); // TODO use std::format when it becomes available
        std::string myName = std::string("sphere_") + buf;
        geometryOut << std::endl << "o " << myName << std::endl;
        if (_material) {
            const std::string materialName = _material->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
            geometryOut << "usemtl " << materialName << std::endl;
        }
        Mesh* mesh = generateSphere(_transformation);
        MeshInstance inst(mesh, _material, _animationIndex);
        inst.exportGeometryDataToObj(geometryOut, globalVertexIndex, animationCache);
        delete mesh;
        return myName;
    }
};

}
