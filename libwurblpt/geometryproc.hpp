/*
 * Copyright (C) 2018, 2019, 2020, 2021, 2022
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

#include <vector>

#include "gvm.hpp"


namespace WurblPT {

/* Compute normals from positions. */

typedef enum {
    NormalsFromFirstFace,               /* The vertex normal is set to the normal
                                           of the first facei that the vertex belongs to. */
    NormalsFromFaceAverage,             /* The vertex normal is set to the average
                                           of the face normals of all faces that the
                                           vertex belongs to. */
    NormalsFromWeightedFaceAverage      /* The vertex normal is set to the weighted average
                                           of the face normals of all faces that the vertex
                                           belongs to. The weights depend on the angle that
                                           each face contributes to the vertex. See Thürmer,
                                           G., Wüthrich, C., Computing Vertex Normals from
                                           Polygonal Facets, Journal of Graphics Tools, 3(1),
                                           1998 pps. 43-46. This method is generally
                                           considered to result in high quality normals for
                                           many different types of geometry (see e.g. the
                                           comments in Meshlab). */
} NormalSource;

inline std::vector<vec3> computeNormals(
        const std::vector<vec3>& positions,
        const std::vector<unsigned int> indices,
        NormalSource normalSource = NormalsFromWeightedFaceAverage)
{
    size_t vertexCount = positions.size();
    size_t triangleCount = indices.size() / 3;
    std::vector<vec3> faceNormals(triangleCount);  // normal for each face
    std::vector<std::vector<unsigned int>> vertexFaces(vertexCount); // list of faces for each vertex

    for (size_t i = 0; i < triangleCount; i++) {
        unsigned int i0 = indices[3 * i + 0];
        unsigned int i1 = indices[3 * i + 1];
        unsigned int i2 = indices[3 * i + 2];
        assert(i0 < vertexCount);
        assert(i1 < vertexCount);
        assert(i2 < vertexCount);
        vec3 v0 = positions[i0];
        vec3 v1 = positions[i1];
        vec3 v2 = positions[i2];
        vec3 e0 = v1 - v0;
        vec3 e1 = v2 - v0;
        vec3 e2 = e1 - e0;
        vec3 faceNormal(0.0f, 0.0f, 1.0f);
        if (dot(e0, e0) > 0.0f && dot(e1, e1) > 0.0f && dot(e2, e2) > 0.0f) {
            vec3 fn = cross(e0, e1);
            if (dot(fn, fn) > 0.0f) {
                faceNormal = normalize(fn);
                assert(all(isfinite(faceNormal)));
                assert(abs(dot(faceNormal, faceNormal) - 1.0f) < dirSquaredLengthTolerance);
            }
        }
        faceNormals[i] = faceNormal;
        vertexFaces[i0].push_back(i);
        vertexFaces[i1].push_back(i);
        vertexFaces[i2].push_back(i);
    }

    std::vector<vec3> normals(vertexCount);
    for (size_t i = 0; i < vertexCount; i++) {
        vec3 n(0.0f);
        if (vertexFaces[i].size() == 0) {
            // vertex without a face: will not be rendered anyway
            n = vec3(0.0f, 0.0f, 1.0f);
        } else if (vertexFaces[i].size() == 1) {
            // only one face: no choice in methods
            n = faceNormals[vertexFaces[i][0]];
        } else {
            if (normalSource == NormalsFromWeightedFaceAverage) {
                for (size_t j = 0; j < vertexFaces[i].size(); j++) {
                    unsigned int faceIndex = vertexFaces[i][j];
                    assert(faceIndex < triangleCount);
                    unsigned int faceVertexIndices[3] = {
                        indices[3 * faceIndex + 0],
                        indices[3 * faceIndex + 1],
                        indices[3 * faceIndex + 2]
                    };
                    assert(faceVertexIndices[0] < vertexCount);
                    assert(faceVertexIndices[1] < vertexCount);
                    assert(faceVertexIndices[2] < vertexCount);
                    vec3 e0, e1;
                    if (i == faceVertexIndices[0]) {
                        e0 = vec3(positions[faceVertexIndices[1]]);
                        e1 = vec3(positions[faceVertexIndices[2]]);
                    } else if (i == faceVertexIndices[1]) {
                        e0 = vec3(positions[faceVertexIndices[2]]);
                        e1 = vec3(positions[faceVertexIndices[0]]);
                    } else {
                        e0 = vec3(positions[faceVertexIndices[0]]);
                        e1 = vec3(positions[faceVertexIndices[1]]);
                    }
                    assert(all(isfinite(e0)));
                    assert(all(isfinite(e1)));
                    e0 = e0 - positions[i];
                    e1 = e1 - positions[i];
                    if (dot(e0, e0) <= 0.0f || dot(e1, e1) <= 0.0f)
                        continue;
                    float x = dot(normalize(e0), normalize(e1));
                    assert(isfinite(x));
                    float alpha = acos(clamp(x, -1.0f, +1.0f));
                    n += alpha * faceNormals[faceIndex];
                }
            }
            if (normalSource == NormalsFromFaceAverage
                    || (normalSource == NormalsFromWeightedFaceAverage && dot(n, n) <= 0.0f)) {
                // use equal weights for each face
                n = vec3(0.0f);
                for (size_t j = 0; j < vertexFaces[i].size(); j++) {
                    unsigned int faceIndex = vertexFaces[i][j];
                    n += faceNormals[faceIndex];
                }
            }
            if (normalSource == NormalsFromFirstFace || dot(n, n) <= 0.0f) {
                unsigned int faceIndex = vertexFaces[i][0];
                n = faceNormals[faceIndex];
            }
            assert(all(isfinite(n)));
            assert(dot(n, n) > 0.0f);
            n = normalize(n);
#if 0
            // Sanity check: the angle between our computed normal and each face
            // normal must always be smaller than 90 degrees. If that is not the
            // case, simply fall back to the first face normal.
            for (size_t j = 0; j < vertexFaces[i].size(); j++) {
                size_t faceIndex = vertexFaces[i][j];
                vec3 fn = faceNormals[faceIndex];
                if (dot(n, fn) >= half_pi<float>()) {
                    n = faceNormals[vertexFaces[i][0]];
                    break;
                }
            }
#endif
        }
        assert(all(isfinite(n)));
        assert(abs(dot(n, n) - 1.0f) < dirSquaredLengthTolerance);
        normals[i] = n;
    }

    return normals;
}

/* Compute tangents from positions, normals, and texcoords. */

inline std::vector<vec3> computeTangents(
        const std::vector<vec3>& positions,
        const std::vector<vec3>& normals,
        const std::vector<vec2>& texcoords,
        const std::vector<unsigned int> indices)
{
    size_t vertexCount = positions.size();
    size_t triangleCount = indices.size() / 3;
    std::vector<vec3> tangents(vertexCount, vec3(0.0f));
    for (size_t t = 0; t < triangleCount; t++) {
        unsigned int i0 = indices[3 * t + 0];
        unsigned int i1 = indices[3 * t + 1];
        unsigned int i2 = indices[3 * t + 2];
        const vec3& P0 = positions[i0];
        const vec3& P1 = positions[i1];
        const vec3& P2 = positions[i2];
        const vec2& tc0 = texcoords[i0];
        const vec2& tc1 = texcoords[i1];
        const vec2& tc2 = texcoords[i2];
        vec3 e1 = P1 - P0;
        vec3 e2 = P2 - P0;
        float s1 = tc1.x() - tc0.x();
        float t1 = tc1.y() - tc0.y();
        float s2 = tc2.x() - tc0.x();
        float t2 = tc2.y() - tc0.y();
        float det = (s1 * t2 - s2 * t1);
        if (abs(det) > epsilon) {
            vec3 tp = 1.0f / det * (t2 * e1 - t1 * e2);
            tangents[i0] += tp;
            tangents[i1] += tp;
            tangents[i2] += tp;
        }
    }
    for (size_t i = 0; i < vertexCount; i++) {
        const vec3& n = normals[i];
        const vec3& tp = tangents[i];
        vec3 t(1.0f, 0.0f, 0.0f); // fall back to a valid tangent of length 1
        // We only have valid tp if we have valid texture coordinates:
        if (dot(tp, tp) > 0.0f) {
            // Gram-Schmidt orthonormalization:
            t = normalize(tp - dot(n, tp) * n);
        }
        tangents[i] = t;
    }
    return tangents;
}

}
