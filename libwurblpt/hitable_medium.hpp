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

#include "hitable.hpp"
#include "bvh.hpp"
#include "prng.hpp"


namespace WurblPT {

/* A medium, e.g. participating media, subsurface scattering etc.
 * Current restrictions: no importance sampling
 */
class HitableMedium final : public Hitable
{
private:
    const Material* _material;
    const std::vector<const Hitable*> _boundaryHitables;
    const float _density;
    BVH _boundaryBVH;

public:
    HitableMedium(const std::vector<const Hitable*>& boundary, float density, const Material* phaseFunction) :
        _material(phaseFunction),
        _boundaryHitables(boundary),
        _density(density)
    {
    }

    virtual const Material* material() const override
    {
        return _material;
    }

    virtual void updateBVH(AnimationCache& animationCacheT0, AnimationCache& animationCacheT1) override
    {
        _boundaryBVH.build(_boundaryHitables, animationCacheT0, animationCacheT1);
    }

    virtual AABB aabb(AnimationCache& animationCacheT0, AnimationCache& animationCacheT1) const override
    {
        return _boundaryBVH.aabb(animationCacheT0, animationCacheT1);
    }

    virtual HitRecord hit(const Ray& ray, const RayIntersectionHelper& rayHelper,
            float amin, float amax, float minHitDistance,
            AnimationCache& animationCache,
            Prng& prng) const override
    {
        HitRecord hr;
        HitRecord hr1 = _boundaryBVH.hit(ray, rayHelper, -maxval, maxval, minHitDistance,
                animationCache, prng);
        if (hr1.haveHit) {
            HitRecord hr2 = _boundaryBVH.hit(ray, rayHelper, hr1.a + minHitDistance, maxval, minHitDistance,
                    animationCache, prng);
            if (hr2.haveHit) {
                if (hr1.a < amin)
                    hr1.a = amin;
                if (hr2.a > amax)
                    hr2.a = amax;
                if (hr1.a < hr2.a) {
                    if (hr1.a < 0.0f)
                        hr1.a = 0.0f;
                    float distance_inside_boundary = hr2.a - hr1.a;
                    float hit_distance = - (1.0f / _density) * log(prng.in01());
                    if (hit_distance < distance_inside_boundary) {
                        float a = hr1.a + hit_distance;
                        hr = HitRecord(a, ray.at(a), vec3(0.0f), vec3(0.0f), vec2(0.0f), false, this);
                    }
                }
            }
        }
        return hr;
    }
};

}
