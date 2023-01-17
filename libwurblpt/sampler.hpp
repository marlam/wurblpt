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

#include "gvm.hpp"
#include "tangentspace.hpp"


namespace WurblPT {

class Sampler
{
public:
    /*! \brief Returns a uniformly distributed point in the unit disk.
     * Input must be uniformly distributed in [0,1). */
    static vec2 inUnitDisk(const vec2& u)
    {
        // This is second the method from PBR3 13.6.2
        vec2 uOffset = 2.0f * u - vec2(1.0f, 1.0f);
        vec2 result;
        if (uOffset == vec2(0.0f, 0.0f)) {
            result = vec2(0.0f, 0.0f);
        } else {
            float theta, r;
            if (abs(uOffset.x()) > abs(uOffset.y())) {
                r = uOffset.x();
                theta = pi_4 * (uOffset.y() / uOffset.x());
            } else {
                r = uOffset.y();
                theta = pi_2 - pi_4 * (uOffset.x() / uOffset.y());
            }
            result = r * vec2(cos(theta), sin(theta));
        }
        return result;
    }

    /*! \brief Returns a uniformly distributed point on the unit disk (circle).
     * Input must be uniformly distributed in [0,1). */
    static vec2 onUnitDisk(float u0)
    {
        float phi = u0 * 2.0f * pi;
        return vec2(cos(phi), sin(phi));
    }

    /*! \brief Returns a uniformly distributed point on the unit sphere.
     * Input must be uniformly distributed in [0,1). */
    static vec3 onUnitSphere(const vec2& u)
    {
        // This is the method from PBR3 13.6.1
        float z = 1.0f - 2.0f * u.x();
        float r = sqrt(max(0.0f, 1.0f - z * z));
        float phi = 2.0f * pi * u.y();
        return vec3(r * cos(phi), r * sin(phi), z);
    }

    /*! \brief Returns a uniformly distributed point on the unit hemisphere.
     * Input must be uniformly distributed in [0,1). */
    static vec3 onUnitHemisphere(const vec2& u)
    {
        // From PBR3 13.6.1
        float z = u.x();
        float r = sqrt(max(0.0f, 1.0f - z * z));
        float phi = 2.0f * pi * u.y();
        return vec3(r * cos(phi), r * sin(phi), z);
    }

    /*! \brief Returns barycentric coordinates with uniform distribution in a triangle.
     * Input must be uniformly distributed in [0,1). */
    static vec3 inTriangle(const vec2& u)
    {
        // from PBR3 section 13.6.5
        float su0 = sqrt(u.x());
        float b0 = 1.0f - su0;
        float b1 = u.y() * su0;
        return vec3(b0, b1, 1.0f - b0 - b1);
    }

    /*! \brief Returns a cosine distributed direction around z axis.
     * Input must be uniformly distributed in [0,1). */
    static vec3 cosineDirection(const vec2& u)
    {
        // Using the method from PBR3 section 13.6.3:
        vec2 d = inUnitDisk(u);
        float z = sqrt(max(0.0f, 1.0f - dot(d, d)));
        return vec3(d, z);
    }

    /*! \brief returns a direction to a sphere.
     * Input must be uniformly distributed in [0,1). */
    static vec3 toSphere(const vec3& direction, float cosThetaMax, const vec2& u)
    {
        // from PBR3 section 13.6.4
        float cosTheta = (1.0f - u.x()) + u.x() * cosThetaMax;
        float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
        float phi = u.y() * 2.0f * pi;
        vec3 vectorAroundZ = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
        return normalize(TangentSpace(direction).toWorldSpace(vectorAroundZ));
    }
};

}
