/*
 * Copyright (C) 2017, 2018, 2019, 2020, 2021, 2022
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

#include "mesh.hpp"


namespace WurblPT {

/*! \brief Generates a quad shape.
 * The quad has the corners (-1, -1, 0), (+1, -1, 0), (+1, +1, 0), (-1, +1, 0).
 * The transformation is applied.
 */
inline Mesh* generateQuad(const Transformation& T = Transformation(), int slices = 1)
{
    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= slices; i++) {
        float ty = i / (slices / 2.0f);
        for (int j = 0; j <= slices; j++) {
            float tx = j / (slices / 2.0f);
            float x = -1.0f + tx;
            float y = -1.0f + ty;
            float z = 0.0f;
            positions.push_back(vec3(x, y, z));
            normals.push_back(vec3(0.0f, 0.0f, 1.0f));
            texcoords.push_back(0.5f * vec2(tx, ty));
            if (i < slices && j < slices) {
                indices.push_back((i + 0) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

/*! \brief Generates a cube shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateCube(const Transformation& T = Transformation(), int slices = 1)
{
    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    int verticesPerSide = (slices + 1) * (slices + 1);
    for (int side = 0; side < 6; side++) {
        float nx, ny, nz;
        switch (side) {
        case 0: // front
            nx = 0.0f;
            ny = 0.0f;
            nz = +1.0f;
            break;
        case 1: // back
            nx = 0.0f;
            ny = 0.0f;
            nz = -1.0f;
            break;
        case 2: // left
            nx = -1.0f;
            ny = 0.0f;
            nz = 0.0f;
            break;
        case 3: // right
            nx = +1.0f;
            ny = 0.0f;
            nz = 0.0f;
            break;
        case 4: // top
            nx = 0.0f;
            ny = +1.0f;
            nz = 0.0f;
            break;
        case 5: // bottom
            nx = 0.0f;
            ny = -1.0f;
            nz = 0.0f;
            break;
        }
        for (int i = 0; i <= slices; i++) {
            float ty = i / (slices / 2.0f);
            for (int j = 0; j <= slices; j++) {
                float tx = j / (slices / 2.0f);
                float x, y, z;
                switch (side) {
                case 0: // front
                    x = -1.0f + tx;
                    y = -1.0f + ty;
                    z = 1.0f;
                    break;
                case 1: // back
                    x = 1.0f - tx;
                    y = -1.0f + ty;
                    z = -1.0f;
                    break;
                case 2: // left
                    x = -1.0f;
                    y = -1.0f + ty;
                    z = -1.0f + tx;
                    break;
                case 3: // right
                    x = 1.0f;
                    y = -1.0f + ty;
                    z = 1.0f - tx;
                    break;
                case 4: // top
                    x = -1.0f + ty;
                    y = 1.0f;
                    z = -1.0f + tx;
                    break;
                case 5: // bottom
                    x = 1.0f - ty;
                    y = -1.0f;
                    z = -1.0f + tx;
                    break;
                }
                positions.push_back(vec3(x, y, z));
                normals.push_back(vec3(nx, ny, nz));
                texcoords.push_back(0.5f * vec2(tx, ty));
                if (i < slices && j < slices) {
                    indices.push_back(side * verticesPerSide + (i + 0) * (slices + 1) + (j + 0));
                    indices.push_back(side * verticesPerSide + (i + 0) * (slices + 1) + (j + 1));
                    indices.push_back(side * verticesPerSide + (i + 1) * (slices + 1) + (j + 0));
                    indices.push_back(side * verticesPerSide + (i + 0) * (slices + 1) + (j + 1));
                    indices.push_back(side * verticesPerSide + (i + 1) * (slices + 1) + (j + 1));
                    indices.push_back(side * verticesPerSide + (i + 1) * (slices + 1) + (j + 0));
                }
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

/*! \brief Generates a cube side shape.
 * The side index must be one of 0 (posx), 1 (negx), 2 (posy), 3 (negy), 4 (posz), 5 (negz).
 * Useful for creating cubes with different materials on each side.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateCubeSide(int side, const Transformation& T = Transformation(), int slices = 1)
{
    assert(side >= 0 && side < 6);
    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    float nx, ny, nz;
    switch (side) {
    case 4:
        nx = 0.0f;
        ny = 0.0f;
        nz = +1.0f;
        break;
    case 5:
        nx = 0.0f;
        ny = 0.0f;
        nz = -1.0f;
        break;
    case 1:
        nx = -1.0f;
        ny = 0.0f;
        nz = 0.0f;
        break;
    case 0:
    default:
        nx = +1.0f;
        ny = 0.0f;
        nz = 0.0f;
        break;
    case 2:
        nx = 0.0f;
        ny = +1.0f;
        nz = 0.0f;
        break;
    case 3:
        nx = 0.0f;
        ny = -1.0f;
        nz = 0.0f;
        break;
    }
    for (int i = 0; i <= slices; i++) {
        float ty = i / (slices / 2.0f);
        for (int j = 0; j <= slices; j++) {
            float tx = j / (slices / 2.0f);
            float x, y, z;
            switch (side) {
            case 4:
                x = -1.0f + tx;
                y = -1.0f + ty;
                z = 1.0f;
                break;
            case 5:
                x = 1.0f - tx;
                y = -1.0f + ty;
                z = -1.0f;
                break;
            case 1:
                x = -1.0f;
                y = -1.0f + ty;
                z = -1.0f + tx;
                break;
            case 0:
            default:
                x = 1.0f;
                y = -1.0f + ty;
                z = 1.0f - tx;
                break;
            case 2:
                x = -1.0f + ty;
                y = 1.0f;
                z = -1.0f + tx;
                break;
            case 3:
                x = 1.0f - ty;
                y = -1.0f;
                z = -1.0f + tx;
                break;
            }
            positions.push_back(vec3(x, y, z));
            normals.push_back(vec3(nx, ny, nz));
            texcoords.push_back(0.5f * vec2(tx, ty));
            if (i < slices && j < slices) {
                indices.push_back((i + 0) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

/*! \brief Generates a disk shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateDisk(const Transformation& T = Transformation(), float innerRadius = 0.0f, int slices = 40)
{
    const int loops = 1;

    assert(innerRadius >= 0.0f);
    assert(innerRadius <= 1.0f);
    assert(slices >= 4);
    assert(loops >= 1);

    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= loops; i++) {
        float ty = static_cast<float>(i) / loops;
        float r = innerRadius + ty * (1.0f - innerRadius);
        for (int j = 0; j <= slices; j++) {
            float tx = static_cast<float>(j) / slices;
            float alpha = tx * (2.0f * pi) + pi_2;
            positions.push_back(vec3(r * cos(alpha), r * sin(alpha), 0.0f));
            normals.push_back(vec3(0.0f, 0.0f, 1.0f));
            texcoords.push_back(vec2(1.0f - tx, ty));
            if (i < loops && j < slices) {
                indices.push_back((i + 0) * (slices + 1) + (j + 0));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
                indices.push_back((i + 1) * (slices + 1) + (j + 1));
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

/*! \brief Generates a sphere shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateSphere(const Transformation& T = Transformation(), int slices = 40, int stacks = 20)
{
    assert(slices >= 4);
    assert(stacks >= 2);

    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= stacks; i++) {
        float ty = static_cast<float>(i) / stacks;
        float lat = ty * pi;
        for (int j = 0; j <= slices; j++) {
            float tx = static_cast<float>(j) / slices;
            float lon = tx * (2.0f * pi) - pi_2;
            float sinlat = sin(lat);
            float coslat = cos(lat);
            float sinlon = sin(lon);
            float coslon = cos(lon);
            float x = sinlat * coslon;
            float y = coslat;
            float z = sinlat * sinlon;
            positions.push_back(vec3(x, y, z));
            normals.push_back(vec3(x, y, z));
            texcoords.push_back(vec2(1.0f - tx, 1.0f - ty));
            if (i < stacks && j < slices) {
                indices.push_back((i + 0) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

/* Helper function to generate cylinder variants */
inline Mesh* generateCylinder(bool closed, const Transformation& T, int slices)
{
    const int stacks = 1;

    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    // the cylinder
    for (int i = 0; i <= stacks; i++) {
        float ty = static_cast<float>(i) / stacks;
        for (int j = 0; j <= slices; j++) {
            float tx = static_cast<float>(j) / slices;
            float alpha = tx * (2.0f * pi) - pi_2;
            float x = cos(alpha);
            float y = -(ty * 2.0f - 1.0f);
            float z = sin(alpha);
            positions.push_back(vec3(x, y, z));
            normals.push_back(vec3(x, 0.0f, z));
            texcoords.push_back(vec2(1.0f - tx, 1.0f - ty));
            if (i < stacks && j < slices) {
                indices.push_back((i + 0) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
            }
        }
    }
    // the caps
    if (closed) {
        for (int side = 0; side < 2; side++) {
            unsigned int start_index = positions.size();
            const int loops = 1;
            const float innerRadius = 0.0f;
            for (int i = 0; i <= loops; i++) {
                float ty = static_cast<float>(i) / loops;
                float r = innerRadius + ty * (1.0f - innerRadius);
                for (int j = 0; j <= slices; j++) {
                    float tx = static_cast<float>(j) / slices;
                    float alpha = tx * (2.0f * pi) + pi_2;
                    if (side == 0) {
                        positions.push_back(vec3(r * cos(alpha), +1.0f, r * sin(alpha)));
                        normals.push_back(vec3(0.0f, +1.0f, 0.0f));
                    } else {
                        positions.push_back(vec3(r * cos(alpha), -1.0f, r * sin(alpha)));
                        normals.push_back(vec3(0.0f, -1.0f, 0.0f));
                    }
                    texcoords.push_back(vec2(1.0f - tx, ty));
                    if (i < loops && j < slices) {
                        indices.push_back(start_index + (i + 0) * (slices + 1) + (j + 0));
                        if (side == 0) {
                            indices.push_back(start_index + (i + 0) * (slices + 1) + (j + 1));
                            indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 0));
                        } else {
                            indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 0));
                            indices.push_back(start_index + (i + 0) * (slices + 1) + (j + 1));
                        }
                        indices.push_back(start_index + (i + 0) * (slices + 1) + (j + 1));
                        if (side == 0) {
                            indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 1));
                            indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 0));
                        } else {
                            indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 0));
                            indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 1));
                        }
                    }
                }
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

/*! \brief Generates a cylinder shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateCylinder(const Transformation& T = Transformation(), int slices = 40)
{
    return generateCylinder(false, T, slices);
}

/*! \brief Generates a closed cylinder shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateClosedCylinder(const Transformation& T = Transformation(), int slices = 40)
{
    return generateCylinder(true, T, slices);
}

/* Helper function to generate cone shapes */
inline Mesh* generateCone(bool closed, const Transformation& T, int slices = 40, int stacks = 20)
{
    assert(slices >= 4);
    assert(stacks >= 2);

    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    // the cone
    for (int i = 0; i <= stacks; i++) {
        float ty = static_cast<float>(i) / stacks;
        for (int j = 0; j <= slices; j++) {
            float tx = static_cast<float>(j) / slices;
            float alpha = tx * (2.0f * pi) - pi_2;
            float x = ty * cos(alpha);
            float y = -(ty * 2.0f - 1.0f);
            float z = ty * sin(alpha);
            float nx = x;
            float ny = 0.5f;
            float nz = z;
            float nl = sqrt(nx * nx + ny * ny + nz * nz);
            positions.push_back(vec3(x, y, z));
            normals.push_back(vec3(nx, ny, nz) / nl);
            texcoords.push_back(vec2(1.0f - tx, 1.0f - ty));
            if (i < stacks && j < slices) {
                indices.push_back((i + 0) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
                indices.push_back((i + 0) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 1));
                indices.push_back((i + 1) * (slices + 1) + (j + 0));
            }
        }
    }
    // the cap
    if (closed) {
        unsigned int start_index = positions.size();
        const int loops = 1;
        const float innerRadius = 0.0f;
        for (int i = 0; i <= loops; i++) {
            float ty = static_cast<float>(i) / loops;
            float r = innerRadius + ty * (1.0f - innerRadius);
            for (int j = 0; j <= slices; j++) {
                float tx = static_cast<float>(j) / slices;
                float alpha = tx * (2.0f * pi) + pi_2;
                positions.push_back(vec3(r * cos(alpha), -1.0f, r * sin(alpha)));
                normals.push_back(vec3(0.0f, -1.0f, 0.0f));
                texcoords.push_back(vec2(1.0f - tx, ty));
                if (i < loops && j < slices) {
                    indices.push_back(start_index + (i + 0) * (slices + 1) + (j + 0));
                    indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 0));
                    indices.push_back(start_index + (i + 0) * (slices + 1) + (j + 1));
                    indices.push_back(start_index + (i + 0) * (slices + 1) + (j + 1));
                    indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 0));
                    indices.push_back(start_index + (i + 1) * (slices + 1) + (j + 1));
                }
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}
/*! \brief Generates a cone shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateCone(const Transformation& T = Transformation(), int slices = 40, int stacks = 20)
{
    return generateCone(false, T, slices, stacks);
}

/*! \brief Generates a closed cone shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateClosedCone(const Transformation& T = Transformation(), int slices = 40, int stacks = 20)
{
    return generateCone(true, T, slices, stacks);
}

/*! \brief Generates a torus shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateTorus(const Transformation& T = Transformation(), float innerRadius = 0.4f, int sides = 40, int rings = 40)
{
    assert(innerRadius >= 0.0f);
    assert(innerRadius < 1.0f);
    assert(sides >= 4);
    assert(rings >= 4);

    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    float ringradius = (1.0f - innerRadius) / 2.0f;
    float ringcenter = innerRadius + ringradius;

    for (int i = 0; i <= sides; i++) {
        float ty = static_cast<float>(i) / sides;
        float alpha = ty * (2.0f * pi) - pi_2;
        float c = cos(alpha);
        float s = sin(alpha);
        for (int j = 0; j <= rings; j++) {
            float tx = static_cast<float>(j) / rings;
            float beta = tx * (2.0f * pi) - pi;

            float x = ringcenter + ringradius * cos(beta);
            float y = 0.0f;
            float z = ringradius * sin(beta);
            float rx = c * x + s * y;
            float ry = c * y - s * x;
            float rz = z;
            positions.push_back(vec3(rx, ry, rz));

            float rcx = c * ringcenter;
            float rcy = - s * ringcenter;
            float rcz = 0.0f;
            float nx = rx - rcx;
            float ny = ry - rcy;
            float nz = rz - rcz;
            float nl = sqrt(nx * nx + ny * ny + nz * nz);
            normals.push_back(vec3(nx, ny, nz) / nl);

            texcoords.push_back(vec2(1.0f - tx, 1.0f - ty));
            if (i < sides && j < rings) {
                indices.push_back((i + 0) * (rings + 1) + (j + 0));
                indices.push_back((i + 0) * (rings + 1) + (j + 1));
                indices.push_back((i + 1) * (rings + 1) + (j + 0));
                indices.push_back((i + 0) * (rings + 1) + (j + 1));
                indices.push_back((i + 1) * (rings + 1) + (j + 1));
                indices.push_back((i + 1) * (rings + 1) + (j + 0));
            }
        }
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

/* Helper functions to create platonic solids */
inline void addTriangleToSolid(
        std::vector<vec3>& positions,
        std::vector<vec3>& normals,
        std::vector<vec2>& texcoords,
        std::vector<unsigned int>& indices,
        const vec3& v0, const vec3& v1, const vec3& v2,
        const vec2& tc0, const vec2& tc1, const vec2& tc2)
{
    positions.push_back(v0);
    positions.push_back(v1);
    positions.push_back(v2);
    vec3 e0 = v1 - v0;
    vec3 e1 = v2 - v0;
    vec3 n = normalize(cross(e0, e1));
    normals.push_back(n);
    normals.push_back(n);
    normals.push_back(n);
    texcoords.push_back(tc0);
    texcoords.push_back(tc1);
    texcoords.push_back(tc2);
    indices.push_back(positions.size() - 3);
    indices.push_back(positions.size() - 2);
    indices.push_back(positions.size() - 1);
}

/*! \brief Generates a regular tetrahedron shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateTetrahedron(const Transformation& T = Transformation())
{
    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    // base vertices
    float a = 1.0f / 3.0f;
    float b = sqrt(8.0f / 9.0f);
    float c = sqrt(2.0f / 9.0f);
    float d = sqrt(2.0f / 3.0f);
    vec3 v0 = vec3(  -c,   -a,    d);
    vec3 v1 = vec3(   b,   -a, 0.0f);
    vec3 v2 = vec3(  -d,   -a,   -d);
    vec3 v3 = vec3(0.0f, 1.0f, 0.0f);

    // all triangles get the same tex coords
    vec2 tc0 = vec2(0.0f, 0.0f);
    vec2 tc1 = vec2(1.0f, 0.0f);
    vec2 tc2 = vec2(0.5f, 1.0f);

    // create triangles
    addTriangleToSolid(positions, normals, texcoords, indices, v0, v1, v3, tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v1, v2, v3, tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v2, v0, v3, tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v2, v1, v0, tc0, tc1, tc2);

    return new Mesh(positions, normals, texcoords, indices, T);
}

/*! \brief Generates a regular octahedron shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateOctahedron(const Transformation& T = Transformation())
{
    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    // base vertices
    vec3 v0 = vec3( 0.0f, -1.0f,  0.0f);
    vec3 v1 = vec3( 0.0f,  0.0f, -1.0f);
    vec3 v2 = vec3(+1.0f,  0.0f,  0.0f);
    vec3 v3 = vec3( 0.0f,  0.0f, +1.0f);
    vec3 v4 = vec3(-1.0f,  0.0f,  0.0f);
    vec3 v5 = vec3( 0.0f, +1.0f,  0.0f);

    // all triangles get the same tex coords
    vec2 tc0 = vec2(0.0f, 0.0f);
    vec2 tc1 = vec2(1.0f, 0.0f);
    vec2 tc2 = vec2(0.5f, 1.0f);

    // create triangles
    addTriangleToSolid(positions, normals, texcoords, indices, v1, v2, v0, tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v2, v3, v0, tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v3, v4, v0, tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v4, v1, v0, tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v1, v5, v2, tc1, tc2, tc0);
    addTriangleToSolid(positions, normals, texcoords, indices, v2, v5, v3, tc1, tc2, tc0);
    addTriangleToSolid(positions, normals, texcoords, indices, v3, v5, v4, tc1, tc2, tc0);
    addTriangleToSolid(positions, normals, texcoords, indices, v4, v5, v1, tc1, tc2, tc0);

    return new Mesh(positions, normals, texcoords, indices, T);
}

/*! \brief Generates a regular icosahedron shape.
 * The geometry is centered on the origin and fills [-1,+1]^3.
 * The transformation is applied.
 */
inline Mesh* generateIcosahedron(const Transformation& T = Transformation())
{
    std::vector<vec3> positions, normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;

    // base vertices
    float r = 2.0f / (1 + sqrt(5.0f)); // 1 / golden ratio
    vec3 v[] = {
        vec3(0.0f, +r, -1.0f),
        vec3(+r, +1.0f, 0.0f),
        vec3(-r, +1.0f, 0.0f),
        vec3(0.0f, +r, +1.0f),
        vec3(0.0f, -r, +1.0f),
        vec3(-1.0f, 0.0f, +r),
        vec3(0.0f, -r, -1.0f),
        vec3(+1.0f, 0.0f, -r),
        vec3(+1.0f, 0.0f, +r),
        vec3(-1.0f, 0.0f, -r),
        vec3(+r, -1.0f, 0.0f),
        vec3(-r, -1.0f, 0.0f)
    };

    // all triangles get the same tex coords
    vec2 tc0 = vec2(0.0f, 0.0f);
    vec2 tc1 = vec2(1.0f, 0.0f);
    vec2 tc2 = vec2(0.5f, 1.0f);

    // create triangles
    addTriangleToSolid(positions, normals, texcoords, indices, v[2], v[1], v[0], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[1], v[2], v[3], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[5], v[4], v[3], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[4], v[8], v[3], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[7], v[6], v[0], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[6], v[9], v[0], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[11], v[10], v[4], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[10], v[11], v[6], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[9], v[5], v[2], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[5], v[9], v[11], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[8], v[7], v[1], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[7], v[8], v[10], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[2], v[5], v[3], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[8], v[1], v[3], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[9], v[2], v[0], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[1], v[7], v[0], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[11], v[9], v[6], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[7], v[10], v[6], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[5], v[11], v[4], tc0, tc1, tc2);
    addTriangleToSolid(positions, normals, texcoords, indices, v[10], v[8], v[4], tc0, tc1, tc2);

    return new Mesh(positions, normals, texcoords, indices, T);
}

}
