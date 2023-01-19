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
#include "ray.hpp"


namespace WurblPT {

class AABB
{
public:
    vec3 lo, hi;

    AABB()
    {
    }

    AABB(const vec3& lo, const vec3& hi) : lo(lo), hi(hi)
    {
    }

    vec3 center() const
    {
        return 0.5f * (lo + hi);
    }

    int longestAxis() const
    {
        int axis = 2;
        vec3 l = hi - lo;
        if (l[0] > l[1] && l[0] > l[2]) {
            axis = 0;
        } else if (l[1] > l[2]) {
            axis = 1;
        }
        return axis;
    }

    float surfaceArea() const
    {
        vec3 l = hi - lo;
        return 2.0f * (l.x() * l.y() + l.y() * l.z() + l.x() * l.z());
    }

    bool mayHit(const Ray& ray, float amin, float amax, const vec3& invRayDirection) const
    {
        /* From "A Ray-Box Intersection Algorithm and Efficient Dynamic Voxel Rendering"
         * by A. Majercik, C. Crassin, P. Shirley, M. McGuire
         * https://jcgt.org/published/0007/03/04/
         * Listing 1.
         * This gives a significant performance improvement compared to
         * "An Efficient and Robust Ray-Box Intersection Algorithm" by Williams, Barrus, Morley, Shirley.
         * See this blog post for a derivation:
         * https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
         */
        vec3 t0 = (lo - ray.origin) * invRayDirection;
        vec3 t1 = (hi - ray.origin) * invRayDirection;
        vec4 tmin = vec4(amin, min(t0, t1));
        vec4 tmax = vec4(amax, max(t0, t1));
        return max(tmin) <= min(tmax);
    }
};

inline AABB merge(const AABB& aabb0, const AABB& aabb1)
{
    return AABB(min(aabb0.lo, aabb1.lo), max(aabb0.hi, aabb1.hi));
}

}
