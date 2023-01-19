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
#include <vector>

#include "material.hpp"
#include "scene_component.hpp"
#include "geometryproc.hpp"
#include "transformation.hpp"


namespace WurblPT {

class Mesh
{
public:
    // Interleaved data: for each vertex, the sequence of pos, nrm, [tc], [tng] is stored.
    constexpr static size_t positionOffset = 0;
    constexpr static size_t normalOffset = 3;
    constexpr static size_t texcoordOffset = 6;
    constexpr static size_t tangentOffset = 8;
    bool haveTexCoords;
    bool haveTangents;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    static size_t vertexSize(bool haveTexCoords, bool haveTangents)
    {
        return (haveTexCoords && haveTangents ? 11 : haveTexCoords ? 8 : 6);
    }

    /* Member functions for data access, to be used e.g. by the exporter */

    size_t vertexSize() const { return vertexSize(haveTexCoords, haveTangents); }
    size_t vertexCount() const { return vertices.size() / vertexSize(); }
    size_t triangleCount() const { return indices.size() / 3; }
    vec3 position(unsigned int i) const {                        return vec3(vertices.data() + i * vertexSize() + positionOffset); }
    vec3 normal  (unsigned int i) const {                        return vec3(vertices.data() + i * vertexSize() + normalOffset);   };
    vec2 texcoord(unsigned int i) const { assert(haveTexCoords); return vec2(vertices.data() + i * vertexSize() + texcoordOffset); };
    vec3 tangent (unsigned int i) const { assert(haveTangents);  return vec3(vertices.data() + i * vertexSize() + tangentOffset);  };

    /*!\brief Constructor.
     * Positions \a pos, normals \a nrm, and indices \a ind must be given;
     * texture coordinates \tc may be empty (in which case you cannot have tangents!).
     * If you don't have normals, use computerNormals() to generate them.
     * Optionally the geometry is transformed using \a T.
     * Optionally tangents are generated.
     * */
    Mesh(
            const std::vector<vec3>& pos,
            const std::vector<vec3>& nrm,
            const std::vector<vec2>& tc,
            const std::vector<unsigned int>& ind,
            const Transformation& T = Transformation(),
            bool wantTangents = true) :
        indices(ind)
    {
        assert(pos.size() > 0);
        assert(pos.size() == nrm.size());
        assert(pos.size() == tc.size() || tc.size() == 0);
        assert(ind.size() > 0);
        assert(ind.size() % 3 == 0);
        for (size_t i = 0; i < ind.size(); i++)
            assert(ind[i] < pos.size());
        for (size_t i = 0; i < tc.size(); i++)
            assert(all(isfinite(tc[i])));
        for (size_t i = 0; i < pos.size(); i++) {
            assert(all(isfinite(pos[i])));
            assert(all(isfinite(nrm[i])));
        }

        bool allTexCoordsAreZero = true;
        for (size_t i = 0; i < tc.size(); i++) {
            if (notEqual(tc[i], vec2(0.0f, 0.0f))) {
                allTexCoordsAreZero = false;
                break;
            }
        }
        if (allTexCoordsAreZero) {
            assert(!wantTangents);
            wantTangents = false;
        }
        std::vector<vec3> tng;
        if (wantTangents) {
            tng = computeTangents(pos, nrm, tc, ind);
        }

        haveTexCoords = !allTexCoordsAreZero;
        haveTangents = wantTangents;
        vertices.resize(pos.size() * vertexSize());

        bool transform = !T.isIdentity();
        mat4 M;
        mat3 N;
        if (transform) {
            M = T.toMat4();
            N = T.toNormalMatrix();
        }

        for (size_t i = 0; i < vertexCount(); i++) {
            float* data = vertices.data() + i * vertexSize();
            vec3 p = pos[i];
            vec3 n = nrm[i];
            if (transform) {
                p = (M * vec4(p, 1.0f)).xyz();
                n = N * n;
            }
            data[0] = p.x();
            data[1] = p.y();
            data[2] = p.z();
            data[3] = n.x();
            data[4] = n.y();
            data[5] = n.z();
            if (haveTexCoords) {
                vec2 t = tc[i];
                data[6] = t[0];
                data[7] = t[1];
            }
            if (haveTangents) {
                vec3 t = tng[i];
                if (transform)
                    t = N * t;
                data[8] = t.x();
                data[9] = t.y();
                data[10] = t.z();
            }
        }
    };
};

class MeshInstance;
std::vector<Hitable*> createHitablesForMeshInstance(const MeshInstance*);

class MeshInstance : public SceneComponent
{
public:
    // Data from constuctor:
    const Mesh* mesh;
    const Material* material;
    const Transformation transformation;
    const int animationIndex;
    // Derived data for speedup:
    const mat4 transformationM; // transformation as mat4
    const mat3 transformationN; // transformation normal matrix

    MeshInstance(const Mesh* mesh,
            const Material* m,
            const Transformation& t,
            int ai = -1) :
        mesh(mesh),
        material(m),
        transformation(t),
        animationIndex(ai),
        transformationM(transformation.toMat4()),
        transformationN(transformation.toNormalMatrix())
    {
    }

    MeshInstance(const Mesh* mesh,
            const Material* m,
            int ai = -1) :
        MeshInstance(mesh, m, Transformation(), ai)
    {
    }

    void exportGeometryDataToObj(std::ostream& geometryOut, unsigned int& globalVertexIndex, AnimationCache& animationCache) const
    {
        bool isTransformed = !(transformation.isIdentity());
        bool isAnimated = (animationIndex >= 0);
        mat4 AM;
        mat3 AN;
        if (isAnimated) {
            AM = animationCache.getM(animationIndex);
            AN = animationCache.getN(animationIndex);
        }
        char buf[128]; // TODO: use std::format when it becomes available
        for (size_t i = 0; i < mesh->vertexCount(); i++) {
            vec3 p = mesh->position(i);
            if (isTransformed)
                p = (transformationM * vec4(p, 1.0f)).xyz();
            if (isAnimated)
                p = (AM * vec4(p, 1.0f)).xyz();
            std::snprintf(buf, sizeof(buf), "v %.8g %.8g %.8g", p.x(), p.y(), p.z());
            geometryOut << buf << std::endl;
        }
        for (size_t i = 0; i < mesh->vertexCount(); i++) {
            vec3 n = mesh->normal(i);
            if (isTransformed)
                n = transformationN * n;
            if (isAnimated)
                n = AN * n;
            std::snprintf(buf, sizeof(buf), "vn %.8g %.8g %.8g", n.x(), n.y(), n.z());
            geometryOut << buf << std::endl;
        }
        if (mesh->haveTexCoords) {
            for (size_t i = 0; i < mesh->vertexCount(); i++) {
                vec2 tc = mesh->texcoord(i);
                std::snprintf(buf, sizeof(buf), "vt %.8g %.8g", tc[0], tc[1]);
                geometryOut << buf << std::endl;
            }
            for (size_t i = 0; i < mesh->triangleCount(); i++) {
                unsigned int i0 = mesh->indices[3 * i + 0] + globalVertexIndex;
                unsigned int i1 = mesh->indices[3 * i + 1] + globalVertexIndex;
                unsigned int i2 = mesh->indices[3 * i + 2] + globalVertexIndex;
                snprintf(buf, sizeof(buf), "f %u/%u/%u %u/%u/%u %u/%u/%u", i0, i0, i0, i1, i1, i1, i2, i2, i2);
                geometryOut << buf << std::endl;
            }
        } else {
            for (size_t i = 0; i < mesh->triangleCount(); i++) {
                unsigned int i0 = mesh->indices[3 * i + 0] + globalVertexIndex;
                unsigned int i1 = mesh->indices[3 * i + 1] + globalVertexIndex;
                unsigned int i2 = mesh->indices[3 * i + 2] + globalVertexIndex;
                snprintf(buf, sizeof(buf), "f %u//%u %u//%u %u//%u", i0, i0, i1, i1, i2, i2);
                geometryOut << buf << std::endl;
            }
        }
        globalVertexIndex += mesh->vertexCount();
    }

    virtual std::vector<Hitable*> createHitables() const override
    {
        return createHitablesForMeshInstance(this);
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
        const std::string myName = std::string("object_") + buf;
        geometryOut << std::endl << "o " << myName << std::endl;
        if (material) {
            const std::string materialName = material->exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);
            geometryOut << "usemtl " << materialName << std::endl;
        }
        exportGeometryDataToObj(geometryOut, globalVertexIndex, animationCache);
        return myName;
    }
};

}
