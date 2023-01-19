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

#include <cassert>
#include <cstdio>

#include <vector>
#include <memory>
#include <map>
#include <fstream>

#include "animation.hpp"
#include "texture.hpp"
#include "material.hpp"
#include "envmap.hpp"
#include "hitable.hpp"
#include "mesh.hpp"
#include "bvh.hpp"
#include "scene_component.hpp"


namespace WurblPT {

typedef enum
{
    ColdSpot,
    HotSpot
} HotSpotType;

/* The scene takes ownership of everything! */
class Scene
{
private:
    std::vector<std::unique_ptr<Animation>> _animations;
    std::vector<const Animation*> _animationsForCaching;
    std::vector<std::unique_ptr<Mesh>> _meshes;
    std::vector<std::unique_ptr<SceneComponent>> _components;
    std::vector<std::unique_ptr<Hitable>> _hitables;
    std::vector<const Hitable*> _hotSpots;
    std::vector<std::unique_ptr<Hitable>> _unhitables;
    std::unique_ptr<EnvironmentMap> _envmap;
    std::unique_ptr<BVH> _bvh;
    bool _bvhNeedsRebuild;
    float _bvhT0, _bvhT1;

    std::map<const Material*, int> _materialMap;
    std::vector<std::string> _materialNames;

public:
    Scene() : _bvhNeedsRebuild(true)
    {
    }

    /* Functions to build a scene. You need to give every texture, material, animation,
     * hitable, and environment map to the scene! Use the take() functions for that purpose.
     * The scene will take ownership.
     * Note that you should *not* give camera animations to the scene, since the
     * camera is not part of the scene! For example, an animated camera in a static scene
     * does not require the BVH to be rebuild every frame.
     */

    int take(Animation* anim)
    {
        _animations.push_back(std::unique_ptr<Animation>(anim));
        _animationsForCaching.push_back(anim);
        return _animations.size() - 1;
    }

    Mesh* take(Mesh* mesh)
    {
        _meshes.push_back(std::unique_ptr<Mesh>(mesh));
        return mesh;
    }

    Texture* take(Texture* tex)
    {
        _components.push_back(std::unique_ptr<SceneComponent>(tex));
        return tex;
    }

    Material* take(Material* mat, const std::string& name = std::string())
    {
        _components.push_back(std::unique_ptr<SceneComponent>(mat));
        _materialMap.insert(std::pair<const Material*, int>(mat, _materialNames.size()));
        _materialNames.push_back(name);
        return mat;
    }

    std::vector<const Hitable*> take(SceneComponent* component, HotSpotType hotSpotType = ColdSpot)
    {
        std::vector<Hitable*> hitables = component->createHitables();
        if (hotSpotType == HotSpot) {
            _hotSpots.insert(_hotSpots.end(), hitables.begin(), hitables.end());
        }
        _hitables.reserve(_hitables.size() + hitables.size());
        for (size_t i = 0; i < hitables.size(); i++)
            _hitables.push_back(std::unique_ptr<Hitable>(hitables[i]));
        _components.push_back(std::unique_ptr<SceneComponent>(component));
        _bvhNeedsRebuild = true;
        return std::vector<const Hitable*>(hitables.begin(), hitables.end());
    }

    std::vector<const Hitable*> takeUnhitable(SceneComponent* component)
    {
        std::vector<Hitable*> hitables = component->createHitables();
        _unhitables.reserve(_unhitables.size() + hitables.size());
        for (size_t i = 0; i < hitables.size(); i++)
            _unhitables.push_back(std::unique_ptr<Hitable>(hitables[i]));
        _components.push_back(std::unique_ptr<SceneComponent>(component));
        return std::vector<const Hitable*>(hitables.begin(), hitables.end());
    }

    EnvironmentMap* take(EnvironmentMap* envmap)
    {
        _envmap = std::unique_ptr<EnvironmentMap>(envmap);
        return envmap;
    }

    /* Functions to build / rebuild the bounding volume hierarchy */

    bool bvhNeedsUpdate(float t0 = 0.0f, float t1 = 0.0f) const
    {
        return _bvhNeedsRebuild
            || (_animations.size() > 0 && (_bvhT0 > t0 || _bvhT1 < t1));
    }

    void updateBVH(float t0 = 0.0f, float t1 = 0.0f)
    {
        if (bvhNeedsUpdate(t0, t1)) {
            AnimationCache animationCacheT0(animations(), t0);
            AnimationCache animationCacheT1(animations(), t1);
            std::vector<const Hitable*> hitables(_hitables.size());
            for (size_t i = 0; i < hitables.size(); i++) {
                _hitables[i]->updateBVH(animationCacheT0, animationCacheT1);
                hitables[i] = _hitables[i].get();
            }
            _bvh = std::unique_ptr<BVH>(new BVH);
            _bvh->build(hitables, animationCacheT0, animationCacheT1);
            _bvhNeedsRebuild = false;
            _bvhT0 = t0;
            _bvhT1 = t1;
        } else {
            fprintf(stderr, "Bounding volume hierarchy does not need updating.\n");
        }
    }

    /* Functions for the renderer to access scene information */

    const std::vector<const Animation*>& animations() const
    {
        return _animationsForCaching;
    }

    const std::vector<const Hitable*>& hotSpots() const
    {
        return _hotSpots;
    }

    const EnvironmentMap* environmentMap() const
    {
        return _envmap.get();
    }

    const Hitable& bvh() const
    {
        return *(_bvh.get());
    }

    /* Functions to get the material IDs and names, for Ground Truth generation */

    int materialIndex(const Material* mat) const
    {
        int id = -1;
        auto it = _materialMap.find(mat);
        if (it != _materialMap.end())
            id = it->second;
        assert(id >= 0);
        return id;
    }

    const std::vector<std::string>& materialNames() const
    {
        return _materialNames;
    }

    /* Function to export a scene to OBJ format */

    /*!\brief Export this scene to OBJ format. Only give the base name of the file,
     * the extension(s) will be determined as necessary (.obj for geometry, .mtl for materials,
     * .png or .exr for textures). */
    bool exportToObj(const std::string& baseFileName, float t = 0.0f) const
    {
        std::filesystem::path path(baseFileName);
        std::filesystem::path basePath = path.parent_path();
        std::string baseName = path.filename().string();

        std::ofstream geometryOut(basePath / (baseName + ".obj"), std::ios::out | std::ios::trunc);
        std::ofstream materialOut(basePath / (baseName + ".mtl"), std::ios::out | std::ios::trunc);

        geometryOut << "mtllib " << baseName << ".mtl" << std::endl;

        AnimationCache animationCache(animations(), t);
        unsigned int globalVertexIndex = 1;
        std::map<const SceneComponent*, std::string> sceneExportCache;
        for (size_t i = 0; i < _components.size(); i++) {
            _components[i]->exportToObj(geometryOut, materialOut, globalVertexIndex, true, animationCache, basePath, baseName, sceneExportCache);
        }

        geometryOut.flush();
        materialOut.flush();
        bool success = geometryOut.good() && materialOut.good();
        geometryOut.close();
        materialOut.close();

        return success;
    }
};

}
