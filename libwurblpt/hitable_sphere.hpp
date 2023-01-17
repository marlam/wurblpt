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


namespace WurblPT {

class HitableSphere final : public Hitable
{
private:
    const Material* _material;
    const int _animationIndex;
    const vec3 _center;
    const float _radius;
    const quat _rotation;

    static HitRecord constructHitRecord(const Ray& ray, float a,
            const vec3& center,
            const quat& rotation,
            const Hitable* hitable)
    {
        // position and normal
        vec3 p = ray.at(a);
        vec3 n = normalize(p - center);

        // texture coordinates
        vec3 rn = rotation * n;
        float alpha = atan(rn.x(), rn.z());
        float beta = asin(clamp(rn.y(), -1.0f, +1.0f));
        float u = 0.5f * inv_pi * (alpha + pi);
        float v = inv_pi * (beta + 0.5f * pi);
        vec2 tc = vec2(u, v);

        // tangent
        vec3 t = vec3(cos(alpha), 0.0f, -sin(alpha));
        if (abs(dot(rn, t)) >= cosOrthoAngleTolerance) {
            // can happen at the poles
            t = vec3(0.0f);
        }

        // backside handling
        bool backside = false;
        if (dot(n, -ray.direction) < 0.0f) {
            backside = true;
            n = -n;
        }

        return HitRecord(a, p, n, t, tc, backside, hitable);
    }

public:
    HitableSphere(const Transformation& T, const Material* material, int animationIndex = -1) :
        _material(material), _animationIndex(animationIndex),
        _center(T.translation), _radius(max(T.scaling)), _rotation(T.rotation)
    {
    }

    virtual const Material* material() const override
    {
        return _material;
    }

    virtual int animationIndex() const override
    {
        return _animationIndex;
    }

    virtual AABB aabb(AnimationCache& animationCacheT0, AnimationCache& animationCacheT1) const override
    {
        if (animationIndex() < 0) {
            return AABB(_center - vec3(_radius), _center + vec3(_radius));
        } else {
            const Transformation& T0 = animationCacheT0.get(animationIndex());
            const Transformation& T1 = animationCacheT1.get(animationIndex());
            vec3 c0 = T0 * _center;
            vec3 c1 = T1 * _center;
            float r0 = max(T0.scaling) * _radius;
            float r1 = max(T1.scaling) * _radius;
            return merge(
                    AABB(c0 - vec3(r0), c0 + vec3(r0)),
                    AABB(c1 - vec3(r1), c1 + vec3(r1)));
        }
    }

    virtual HitRecord hit(const Ray& ray, const RayIntersectionHelper& /* rayHelper */,
            float amin, float amax, float /* minHitDistance */,
            AnimationCache& animationCache,
            Prng& /* prng */) const override
    {
        vec3 center = _center;
        float radius = _radius;

        quat rotation = _rotation;
        if (animationIndex() >= 0) {
            assert(animationCache.t() == ray.time);
            const Transformation& T = animationCache.get(animationIndex());
            center += T.translation;
            radius *= max(T.scaling);
            rotation = _rotation * T.rotation;
        }

        // Compute a1,a2 while avoiding cancellation, see
        // https://marlam.de/path-tracing/course/ part 3

        vec3 oc = ray.origin - center;
        float aq = -dot(oc, ray.direction);
        vec3 tmp = oc - dot(oc, ray.direction) * ray.direction;
        float discriminant = radius * radius - dot(tmp, tmp);

        HitRecord hr;
        if (discriminant > 0.0f) {
            float a1, a2;
            if (aq < 0.0f) {
                a2 = aq - std::sqrt(discriminant);
                a1 = 2.0f * aq - a2;
            } else {
                a1 = aq + std::sqrt(discriminant);
                a2 = 2.0f * aq - a1;
            }
            if (a2 > amin && a2 < amax) {
                hr = constructHitRecord(ray, a2, center, rotation, this);
            } else if (a1 > amin && a1 < amax) {
                hr = constructHitRecord(ray, a1, center, rotation, this);
            }
        }
        return hr;
    }

    virtual float pdfValue(const vec3& origin, const vec3& direction,
            AnimationCache& animationCache,
            Prng& prng) const override
    {
        vec3 center = _center;
        float radius = _radius;

        if (animationIndex() >= 0) {
            const Transformation& T = animationCache.get(animationIndex());
            center = T * center;
            radius *= max(T.scaling);
        }

        float value = 0.0f;
        vec3 cmo = center - origin;
        float distanceSquared = dot(cmo, cmo);
        float radiusSquared = radius * radius;
        if (distanceSquared <= radiusSquared) {
            // We are inside the sphere. The solid angle is thus 4pi,
            // and any direction will hit the sphere.
            value = 0.25f * inv_pi;
        } else {
            // We are outside the sphere. The sphere area is therefore
            // visible as a circle with a solid angle of at most 2pi.
            HitRecord hr = this->hit(Ray(origin, direction, animationCache.t(), 0.0f /* does not matter */),
                    RayIntersectionHelper() /* unused */, 0.0f, maxval, 0.0f,
                    animationCache, prng);
            if (hr.haveHit) {
                float discriminant = 1.0f - radiusSquared / distanceSquared;
                float cosThetaMax = (discriminant > 0.0f ? sqrt(discriminant) : 0.0f);
                float solidAngle = 2.0f * pi * (1.0f - cosThetaMax);
                value = 1.0f / solidAngle;
            }
        }
        return value;
    }

    virtual vec3 direction(const vec3& origin,
            AnimationCache& animationCache,
            Prng& prng) const override
    {
        vec3 center = _center;
        float radius = _radius;

        if (animationIndex() >= 0) {
            const Transformation& T = animationCache.get(animationIndex());
            center += T.translation;
            radius *= max(T.scaling);
        }

        vec3 dir;
        vec3 cmo = center - origin;
        float distanceSquared = dot(cmo, cmo);
        float radiusSquared = radius * radius;
        if (distanceSquared <= radiusSquared) {
            // We are inside the sphere. Any direction will hit the sphere.
            dir = Sampler::onUnitSphere(prng.in01x2());
        } else {
            // We are outside the sphere. Generate a direction that hits it.
            float discriminant = 1.0f - radiusSquared / distanceSquared;
            float cosThetaMax = (discriminant > 0.0f ? sqrt(discriminant) : 0.0f);
            dir = Sampler::toSphere(normalize(cmo), cosThetaMax, prng.in01x2());
        }
        return dir;
    }
};

}
