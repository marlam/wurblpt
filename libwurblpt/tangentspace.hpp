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

#include "gvm.hpp"
#include "constants.hpp"


namespace WurblPT {

// For a surface patch with normal n, the tangent t points into the direction
// of the u texture coordinate, and the bitangent b points into the direction
// of the v texture coordinate. In the corresponding tangent space, the normal
// is (0,0,1), the tangent is (1,0,0), and the bitangent is (0,1,0).
//
// This class provides functions to transform vectors to and from the tangent
// space for a given surface patch.
//
// If there are no texture coordinates and therefore no clearly defined
// tangent and bitangent, this class will use arbitrary ones.
class TangentSpace
{
public:
    vec3 normal;
    vec3 tangent;
    vec3 bitangent;

    TangentSpace()
    {
    }

    TangentSpace(const vec3& n)
        : normal(n)
    {
        assert(abs(dot(normal, normal) - 1.0f) < dirSquaredLengthTolerance);

        // From Duff et al. "Building an Orthonormal Basis, Revisited", JCGT vol 6 no 1, 2017
        float sign = std::copysignf(1.0f, normal.z());
        float a = -1.0f / (sign + normal.z());
        float b = normal.x() * normal.y() * a;
        tangent = vec3(1.0f + sign * normal.x() * normal.x() * a, sign * b, -sign * normal.x());
        bitangent = vec3(b, sign + normal.y() * normal.y() * a, -normal.y());

        assert(abs(dot(tangent, tangent) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(bitangent, bitangent) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(normal, tangent)) < cosOrthoAngleTolerance);
        assert(abs(dot(normal, bitangent)) < cosOrthoAngleTolerance);
        assert(abs(dot(tangent, bitangent)) < cosOrthoAngleTolerance);
    }

    TangentSpace(const vec3& n, const vec3& t)
        : normal(n), tangent(t), bitangent(cross(n, t))
    {
        assert(abs(dot(normal, normal) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(tangent, tangent) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(normal, tangent)) < cosOrthoAngleTolerance);

        assert(abs(dot(bitangent, bitangent) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(normal, bitangent)) < cosOrthoAngleTolerance);
        assert(abs(dot(tangent, bitangent)) < cosOrthoAngleTolerance);
    }

    TangentSpace(const vec3& n, const vec3& t, const vec3& b)
        : normal(n), tangent(t), bitangent(b)
    {
        assert(abs(dot(normal, normal) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(tangent, tangent) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(bitangent, bitangent) - 1.0f) < dirSquaredLengthTolerance);
        assert(abs(dot(normal, tangent)) < cosOrthoAngleTolerance);
        assert(abs(dot(normal, bitangent)) < cosOrthoAngleTolerance);
        assert(abs(dot(tangent, bitangent)) < cosOrthoAngleTolerance);
    }

    mat3 matrixToTangentSpace() const
    {
        return mat3(vec3(tangent.x(),
                    bitangent.x(),
                    normal.x()),
                                vec3(tangent.y(),
                                     bitangent.y(),
                                     normal.y()),
                                                  vec3(tangent.z(),
                                                       bitangent.z(),
                                                       normal.z()));
    }

    // the result of this should be normalized if it is used as a direction in further rays
    vec3 toTangentSpace(const vec3& v) const
    {
        return matrixToTangentSpace() * v;
    }
    vec3 fromWorldSpace(const vec3& v) const
    {
        return toTangentSpace(v);
    }

    mat3 matrixFromTangentSpace() const
    {
        return mat3(tangent, bitangent, normal);
    }

    // the result of this should be normalized if it is used as a direction in further rays
    vec3 fromTangentSpace(const vec3& v) const
    {
        return matrixFromTangentSpace() * v;
    }
    vec3 toWorldSpace(const vec3& v) const
    {
        return fromTangentSpace(v);
    }
};

}
