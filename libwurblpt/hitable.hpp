/*
 * Copyright (C) 2019, 2020, 2021, 2022
 * Computer Graphics Group, University of Siegen (written by Martin Lambers)
 * Copyright (C) 2022, 2023, 2024, 2025
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

#include "aabb.hpp"
#include "prng.hpp"
#include "animation.hpp"


namespace WurblPT {

class Ray;
class Material;
class Hitable;

class HitRecord
{
public:
    bool haveHit;       // do we have a hit? Otherwise the following fields are irrelevant.
    float a;            // hit position = ray.origin + a * ray.direction
    vec3 position;      // hit position = ray.origin + a * ray.direction
    vec3 normal;        // normal at hit position; always points towards the ray, also for back sides
    vec3 tangent;       // tangent in the direction of texcoords.x; not always available or reliable
    vec2 texcoords;     // texcoords at hit position
    bool backside;      // flag: was the hit on the backside? (the normal is flipped then)
    const Hitable* hitable;

    HitRecord() : haveHit(false)
    {
    }

    HitRecord(float a) : // special case used for HitableTriangle::pdfValue() where we don't need full info
        haveHit(true), a(a)
    {
    }

    HitRecord(float a, const vec3& p, const vec3& n, const vec3& t, const vec2& tc, bool bs, const Hitable* h) :
        haveHit(true), a(a), position(p), normal(n), tangent(t), texcoords(tc), backside(bs), hitable(h)
    {
    }
};

class RayIntersectionHelper
{
public:
    // for ray/aabb intersection tests:
    vec3 invDirection;
    // for ray/triangle intersection tests:
    ivec3 k;
    vec3 S;

    RayIntersectionHelper() : invDirection(0.0f), k(0), S(0.0f)
    {
    }

    RayIntersectionHelper(const Ray& ray)
    {
        /* Helper for Ray/AABB intersection tests according to */
        invDirection = 1.0f / ray.direction;

        /* Helpers for Ray/Triangle intersection tests.
         * From "Watertight Ray/Triangle Intersection" by Woop, Benthin, Wald,
         * Appendix A Listing 1. */

        /* calculate dimension where the ray direction is maximal */
        vec3 absdir = abs(ray.direction);
        if (absdir.z() >= absdir.y() && absdir.z() >= absdir.x())
            k.z() = 2;
        else if (absdir.y() >= absdir.x())
            k.z() = 1;
        else
            k.z() = 0;
        k.x() = k.z() + 1;
        if (k.x() == 3)
            k.x() = 0;
        k.y() = k.x() + 1;
        if (k.y() == 3)
            k.y() = 0;
        /* swap kx and ky dimension to preserve winding direction of triangles */
        if (ray.direction[k.z()] < 0.0f) {
            int tmp = k.x();
            k.x() = k.y();
            k.y() = tmp;
        }
        /* calculate shear constants */
        S.x() = ray.direction[k.x()] * invDirection[k.z()];
        S.y() = ray.direction[k.y()] * invDirection[k.z()];
        S.z() = invDirection[k.z()];
    }
};

class Hitable
{
public:
    Hitable()
    {
    }

    virtual ~Hitable()
    {
    }

    virtual const Material* material() const
    {
        return nullptr;
    }

    virtual int animationIndex() const
    {
        return -1;
    }

    virtual void updateBVH(AnimationCache& /* animationCacheT0 */, AnimationCache& /* animationCacheT1 */)
    {
    }

    virtual AABB aabb(AnimationCache& /* animationCacheT0 */, AnimationCache& /* animationCacheT1 */) const
    {
        return AABB(vec3(0.0f), vec3(0.0f));
    }

    virtual HitRecord hit(const Ray& /* ray */, const RayIntersectionHelper& /* rayHelper */,
            float /* amin */, float /* amax */, float /* minHitDistance */,
            AnimationCache& /* animationCache */,
            Prng& /* prng */) const
    {
        return HitRecord();
    }

    virtual float pdfValue(const vec3& /* origin */, const vec3& /* direction */,
            AnimationCache& /* animationCache */, Prng& /* prng */) const
    {
        return 0.0f;
    }

    virtual vec3 direction(const vec3& /* origin */,
            AnimationCache& /* animationCache */,
            Prng& /* prng */) const
    {
        return vec3(0.0f);
    }
};

}
