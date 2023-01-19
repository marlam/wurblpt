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

#include "hitable.hpp"
#include "mesh.hpp"
#include "sampler.hpp"


namespace WurblPT {

template<bool TRANSFORM, bool ANIMATE, bool HAVE_TEXCOORDS, bool HAVE_TANGENTS>
class HitableTriangle final : public Hitable
{
private:
    /* Pack as much information as possible into the member variables
     * to 1) keep sizeof(HitableTriangle) small and 2) avoid pointer
     * dereferences to MeshInstance or Mesh if they are not
     * strictly necessary. See the following encode*() functions that
     * are used by the constructor. */
    const uint64_t _encodedInstancePtr;
    const float* _vertices;
    const uint64_t _encodedIndices;

    // encode instance pointer with additional information

    static uint64_t encodeInstancePtr(const MeshInstance* instance)
    {
        // make sure the template arguments are correct
        assert(TRANSFORM == !(instance->transformation.isIdentity()));
        assert(ANIMATE == (instance->animationIndex >= 0));
        assert(HAVE_TEXCOORDS == instance->mesh->haveTexCoords);
        assert(HAVE_TANGENTS == instance->mesh->haveTangents);

        // make sure we can misuse the four lowest bits of a pointer:
        static_assert(__STDCPP_DEFAULT_NEW_ALIGNMENT__ >= 16);

        int animationIndex = instance->animationIndex;
        // the encoded animation index must fit into 4 bits;
        // the highest value 15 means that it has to be looked up from the instance
        unsigned int encodedAnimationIndex = (ANIMATE ? animationIndex : 0);
        if (encodedAnimationIndex > 15)
            encodedAnimationIndex = 15;
        uint64_t p;
        if (!TRANSFORM && (!ANIMATE || encodedAnimationIndex <= 15)) {
            // We need only instance->material, so we store that
            // pointer directly and avoid any dereference of instance.
            p = reinterpret_cast<uint64_t>(instance->material);
        } else {
            p = reinterpret_cast<uint64_t>(instance);
        }
        if (ANIMATE)
            p |= encodedAnimationIndex;
        return p;
    }

    // decode instance pointer information

    const MeshInstance* instance() const
    {
        assert(TRANSFORM || (ANIMATE && (_encodedInstancePtr & 0xFU) >= 0xFU));
        return reinterpret_cast<const MeshInstance*>(_encodedInstancePtr & 0xFFFFFFFFFFFFFFF0ULL);
    }

public:
    virtual const Material* material() const override
    {
        const Material* m;
        if (!TRANSFORM && (!ANIMATE || (_encodedInstancePtr & 0xFU) < 0xFU))
            m = reinterpret_cast<const Material*>(_encodedInstancePtr & 0xFFFFFFFFFFFFFFF0ULL);
        else
            m = instance()->material;
        return m;
    }

    virtual int animationIndex() const override
    {
        int ai = -1;
        if (ANIMATE) {
            ai = _encodedInstancePtr & 0xFU;
            if (ai == 15)
                ai = instance()->animationIndex;
        }
        return ai;
    }

private:

    // encode indices

    static uint64_t encodeIndices(const unsigned int* indices)
    {
        uint64_t i0 = indices[0];
        uint64_t i1 = indices[1];
        uint64_t i2 = indices[2];
        uint64_t p;
        if (i0 <= 0x1FFFFFULL && i1 <= 0x1FFFFFULL && i2 <= 0x1FFFFFULL) {
            p = 1ULL | (i0 << 1ULL) | (i1 << 22ULL) | (i2 << 43ULL);
        } else {
            p = reinterpret_cast<uint64_t>(indices);
        }
        return p;
    }

    // decode indices

    uvec3 indices() const
    {
        uvec3 i;
        if (_encodedIndices & 1ULL) {
            i[0] = (_encodedIndices >>  1ULL) & 0x1FFFFFULL;
            i[1] = (_encodedIndices >> 22ULL) & 0x1FFFFFULL;
            i[2] = (_encodedIndices >> 43ULL);
        } else {
            i = uvec3(reinterpret_cast<const unsigned int*>(_encodedIndices));
        }
        return i;
    }

    // the following functions return untransformed, unanimated data:

    vec3 position(unsigned int vertexIndex) const
    {
        const float* p = _vertices + vertexIndex * Mesh::vertexSize(HAVE_TEXCOORDS, HAVE_TANGENTS);
        return vec3(p + Mesh::positionOffset);
    }

    vec3 normal(unsigned int vertexIndex) const
    {
        const float* p = _vertices + vertexIndex * Mesh::vertexSize(HAVE_TEXCOORDS, HAVE_TANGENTS);
        return vec3(p + Mesh::normalOffset);
    }

    vec2 texcoord(unsigned int vertexIndex) const
    {
        assert(HAVE_TEXCOORDS);
        const float* p = _vertices + vertexIndex * Mesh::vertexSize(HAVE_TEXCOORDS, HAVE_TANGENTS);
        return vec2(p + Mesh::texcoordOffset);
    }

    vec3 tangent(unsigned int vertexIndex) const
    {
        assert(HAVE_TANGENTS);
        const float* p = _vertices + vertexIndex * Mesh::vertexSize(HAVE_TEXCOORDS, HAVE_TANGENTS);
        return vec3(p + Mesh::tangentOffset);
    }

    // the following implements the hit() functionality both for the special case
    // of pdfValue() and for the full hit() function

    static uint32_t signMask(float a)
    {
        return std::signbit(a) << 31;
    }

    static float xorf(float a, uint32_t b)
    {
        union { float f; uint32_t u; } u;
        u.f = a;
        u.u = u.u ^ b;
        return u.f;
    }

    HitRecord hit(const Ray& ray, const RayIntersectionHelper& rayHelper, float amin, float amax, AnimationCache& animationCache,
            bool fullInfo, vec3* v0v1, vec3* v0v2) const
    {
        /* Adapted from "Watertight Ray/Triangle Intersection" by Woop, Benthin, Wald, Appendix A Listing 2. */

        uvec3 i = indices();
        vec3 v0 = position(i[0]);
        vec3 v1 = position(i[1]);
        vec3 v2 = position(i[2]);

        mat3 transformationN;
        if (TRANSFORM) {
            const MeshInstance* inst = instance();
            const mat4& transformationM = inst->transformationM;
            transformationN = inst->transformationN;
            v0 = (transformationM * vec4(v0, 1.0f)).xyz();
            v1 = (transformationM * vec4(v1, 1.0f)).xyz();
            v2 = (transformationM * vec4(v2, 1.0f)).xyz();
        }
        mat3 animationN;
        if (ANIMATE) {
            assert(animationCache.t() == ray.time);
            int ai = animationIndex();
            const mat4& animationM = animationCache.getM(ai);
            animationN = animationCache.getN(ai);
            v0 = (animationM * vec4(v0, 1.0f)).xyz();
            v1 = (animationM * vec4(v1, 1.0f)).xyz();
            v2 = (animationM * vec4(v2, 1.0f)).xyz();
        }

        /* calculate vertices relative to ray origin */
        const vec3 A = v0 - ray.origin;
        const vec3 B = v1 - ray.origin;
        const vec3 C = v2 - ray.origin;
        /* perform shear and scale of vertices */
        const float Ax = A[rayHelper.k.x()] - rayHelper.S.x() * A[rayHelper.k.z()];
        const float Ay = A[rayHelper.k.y()] - rayHelper.S.y() * A[rayHelper.k.z()];
        const float Bx = B[rayHelper.k.x()] - rayHelper.S.x() * B[rayHelper.k.z()];
        const float By = B[rayHelper.k.y()] - rayHelper.S.y() * B[rayHelper.k.z()];
        const float Cx = C[rayHelper.k.x()] - rayHelper.S.x() * C[rayHelper.k.z()];
        const float Cy = C[rayHelper.k.y()] - rayHelper.S.y() * C[rayHelper.k.z()];
        /* calculate scaled barycentric coordinates */
        float U = Cx * By - Cy * Bx;
        float V = Ax * Cy - Ay * Cx;
        float W = Bx * Ay - By * Ax;
        /* Fallback to test against edges using double precision. Note that we cannot
         * test for exactly zero, contrary to what the paper and PBR3 3.6.2 say;
         * we then get misses at triangle edges (see the Ground Truth background produced
         * by wurblpt-pmd-example). The following test against double-precision epsilon
         * fixes our problems and the fallback is very seldomly used, which is
         * consistent to what PBR3 writes about that matter. */
        if (abs(U) < float(epsilon_v<long double>) || abs(V) < float(epsilon_v<long double>) || abs(W) < float(epsilon_v<long double>)) {
            double CxBy = double(Cx) * double(By);
            double CyBx = double(Cy) * double(Bx);
            U = CxBy - CyBx;
            double AxCy = double(Ax) * double(Cy);
            double AyCx = double(Ay) * double(Cx);
            V = AxCy - AyCx;
            double BxAy = double(Bx) * double(Ay);
            double ByAx = double(By) * double(Ax);
            W = BxAy - ByAx;
        }
        if ((U < 0.0f || V < 0.0f || W < 0.0f) && (U > 0.0f || V > 0.0f || W > 0.0f))
            return HitRecord();
        /* calculate determinant */
        float det = U + V + W;
        if (det == 0.0f)
            return HitRecord();
        /* Calculate scaled z-coordinates of vertices and use them to calculate the hit distance */
        const float Az = rayHelper.S.z() * A[rayHelper.k.z()];
        const float Bz = rayHelper.S.z() * B[rayHelper.k.z()];
        const float Cz = rayHelper.S.z() * C[rayHelper.k.z()];
        const float T = U * Az + V * Bz + W * Cz;
        float detSign = signMask(det);
        if (xorf(T, detSign) < amin * xorf(det, detSign) || xorf(T, detSign) > amax * xorf(det, detSign))
            return HitRecord();
        /* normalize U, V, W, and T */
        const float invDet = 1.0f / det;
        const float a = invDet * T;

        /* Early out for the special case of pdfValue() */
        if (!fullInfo) {
            *v0v1 = v1 - v0;
            *v0v2 = v2 - v0;
            return HitRecord(a);
        }

        /* The rest computes the full HitRecord for the real hit() function */
        bool backfacing = (det < 0.0f);
        const vec3 bary = invDet * vec3(U, V, W);

        // There are three variants to compute the hit position:
        // 1) hitpos = ray.at(t);
        // 2) hitpos = bary[0] * v0 + bary[1] * v1 + bary[2] * v2;
        // 3) hitpos = v0 + u * v0v1 + v * v0v2;
        // It is unclear which is the most precise, thus we go for
        // the cheapest.
        vec3 hitpos = ray.at(a);

        vec3 n0 = normal(i[0]);
        vec3 n1 = normal(i[1]);
        vec3 n2 = normal(i[2]);
        vec3 hitnrm = bary[0] * n0 + bary[1] * n1 + bary[2] * n2;
        if (TRANSFORM)
            hitnrm = transformationN * hitnrm;
        if (ANIMATE)
            hitnrm = animationN * hitnrm;
        hitnrm = normalize(hitnrm);
        if (backfacing)
            hitnrm = -hitnrm;

        vec2 hittc(0.0f);
        if (HAVE_TEXCOORDS) {
            vec2 tc0 = texcoord(i[0]);
            vec2 tc1 = texcoord(i[1]);
            vec2 tc2 = texcoord(i[2]);
            hittc = bary[0] * tc0 + bary[1] * tc1 + bary[2] * tc2;
        }

        vec3 hittan(0.0f);
        if (HAVE_TANGENTS) {
            vec3 t0 = tangent(i[0]);
            vec3 t1 = tangent(i[1]);
            vec3 t2 = tangent(i[2]);
            hittan = bary[0] * t0 + bary[1] * t1 + bary[2] * t2;
            if (dot(hittan, hittan) > 0.0f) {
                if (TRANSFORM)
                    hittan = transformationN * hittan;
                if (ANIMATE)
                    hittan = animationN * hittan;
                // Gram-Schmidt orthonormalization to improve quality:
                hittan = normalize(hittan - dot(hitnrm, hittan) * hitnrm);
            }
        }

        return HitRecord(a, hitpos, hitnrm, hittan, hittc, backfacing, this);
    }

public:
    HitableTriangle(const MeshInstance* instance, const unsigned int* indices) :
        _encodedInstancePtr(encodeInstancePtr(instance)),
        _vertices(instance->mesh->vertices.data()),
        _encodedIndices(encodeIndices(indices))
    {
        static_assert(sizeof(HitableTriangle) == 32 /* vpointer + members */);
    }

    virtual AABB aabb(AnimationCache& animationCacheT0, AnimationCache& animationCacheT1) const override
    {
        uvec3 i = indices();
        vec3 v0 = position(i[0]);
        vec3 v1 = position(i[1]);
        vec3 v2 = position(i[2]);
        if (TRANSFORM) {
            const mat4& transformationM = instance()->transformationM;
            v0 = (transformationM * vec4(v0, 1.0f)).xyz();
            v1 = (transformationM * vec4(v1, 1.0f)).xyz();
            v2 = (transformationM * vec4(v2, 1.0f)).xyz();
        }

        if (!ANIMATE) {
            return AABB(min(v0, v1, v2), max(v0, v1, v2));
        } else {
            int ai = animationIndex();
            const Transformation& T0 = animationCacheT0.get(ai);
            const Transformation& T1 = animationCacheT1.get(ai);
            const mat4& M0 = animationCacheT0.getM(ai);
            vec3 v00 = (M0 * vec4(v0, 1.0f)).xyz();
            vec3 v01 = (M0 * vec4(v1, 1.0f)).xyz();
            vec3 v02 = (M0 * vec4(v2, 1.0f)).xyz();
            if (T0 == T1) {
                return AABB(min(v00, v01, v02), max(v00, v01, v02));
            } else {
                const mat4& M1 = animationCacheT0.getM(ai);
                vec3 v10 = (M1 * vec4(v0, 1.0f)).xyz();
                vec3 v11 = (M1 * vec4(v1, 1.0f)).xyz();
                vec3 v12 = (M1 * vec4(v2, 1.0f)).xyz();
                AABB aabb = merge(
                        AABB(min(v00, v01, v02), max(v00, v01, v02)),
                        AABB(min(v10, v11, v12), max(v10, v11, v12)));
                if (T0.rotation != T1.rotation) {
                    // There is no clean way to solve this, see PBRT3 chapter 2.9.4.
                    // We sample a set of intermediate triangle shapes and merge their
                    // bounding boxes. This reduces the problem of potentially missed
                    // parts, but does not solve it.
                    // We assume here that transformations between t0 and t1 are not
                    // too far apart so that we cannot miss much.
                    unsigned int samples = 4;
                    float cos_half_angle = dot(
                            vec4(T0.rotation.x, T0.rotation.y, T0.rotation.z, T0.rotation.w),
                            vec4(T1.rotation.x, T1.rotation.y, T1.rotation.z, T1.rotation.w));
                    if (abs(cos_half_angle) < 1.0f)
                        samples += degrees(acos(cos_half_angle)) * 2.0f;
                    for (unsigned int i = 1; i < samples - 1; i++) {
                        float alpha = i / (samples - 1.0f);
                        Transformation T = mix(T0, T1, alpha);
                        vec3 vt0 = T * v0;
                        vec3 vt1 = T * v1;
                        vec3 vt2 = T * v2;
                        aabb = merge(aabb,
                                AABB(min(vt0, vt1, vt2), max(vt0, vt1, vt2)));
                    }
                }
                return aabb;
            }
        }
    }

    virtual HitRecord hit(const Ray& ray, const RayIntersectionHelper& rayHelper,
            float amin, float amax, float /* minHitDistance */,
            AnimationCache& animationCache,
            Prng& /* prng */) const override
    {
        return hit(ray, rayHelper, amin, amax, animationCache, true, nullptr, nullptr);
    }

    virtual float pdfValue(const vec3& origin, const vec3& direction,
            AnimationCache& animationCache,
            Prng& /* prng */) const override
    {
        float value = 0.0f;
        Ray ray(origin, direction, animationCache.t(), 0.0f /* does not matter */);
        vec3 v0v1, v0v2;
        HitRecord hr = hit(ray, RayIntersectionHelper(ray), 0.0f, maxval, animationCache, false, &v0v1, &v0v2);
        if (hr.haveHit) {
            vec3 edgeCross = cross(v0v1, v0v2);
            float edgeCrossLength = sqrt(dot(edgeCross, edgeCross));
            vec3 faceNormal = edgeCross / edgeCrossLength;
            float faceArea = 0.5f * edgeCrossLength;
            float cosine = abs(dot(faceNormal, -direction));
            float distance_squared = hr.a * hr.a;
            value = distance_squared / (cosine * faceArea);
        }
        return value;
    }

    virtual vec3 direction(const vec3& origin,
            AnimationCache& animationCache,
            Prng& prng) const override
    {
        uvec3 i = indices();
        vec3 v0 = position(i[0]);
        vec3 v1 = position(i[1]);
        vec3 v2 = position(i[2]);
        vec3 bary = Sampler::inTriangle(prng.in01x2());
        vec3 p = vec3(bary.x() * v0 + bary.y() * v1 + bary.z() * v2);
        if (TRANSFORM) {
            p = (instance()->transformationM * vec4(p, 1.0f)).xyz();
        }
        if (ANIMATE) {
            p = (animationCache.getM(animationIndex()) * vec4(p, 1.0f)).xyz();
        }
        vec3 dir = normalize(p - origin);
        return dir;
    }
};

inline std::vector<Hitable*> createHitablesForMeshInstance(const MeshInstance* instance)
{
    std::vector<Hitable*> h;
    h.resize(instance->mesh->triangleCount());
    bool transform = !instance->transformation.isIdentity();
    bool animate = (instance->animationIndex >= 0);
    bool haveTexCoords = instance->mesh->haveTexCoords;
    bool haveTangents = instance->mesh->haveTangents;
    for (size_t i = 0; i < instance->mesh->triangleCount(); i++) {
        const unsigned int* indices = instance->mesh->indices.data() + 3 * i;
        if (transform) {
            if (animate) {
                if (haveTexCoords) {
                    if (haveTangents)
                        h[i] = new HitableTriangle<true, true, true, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<true, true, true, false>(instance, indices);
                } else {
                    if (haveTangents)
                        h[i] = new HitableTriangle<true, true, false, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<true, true, false, false>(instance, indices);
                }
            } else {
                if (haveTexCoords) {
                    if (haveTangents)
                        h[i] = new HitableTriangle<true, false, true, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<true, false, true, false>(instance, indices);
                } else {
                    if (haveTangents)
                        h[i] = new HitableTriangle<true, false, false, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<true, false, false, false>(instance, indices);
                }
            }
        } else {
            if (animate) {
                if (haveTexCoords) {
                    if (haveTangents)
                        h[i] = new HitableTriangle<false, true, true, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<false, true, true, false>(instance, indices);
                } else {
                    if (haveTangents)
                        h[i] = new HitableTriangle<false, true, false, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<false, true, false, false>(instance, indices);
                }
            } else {
                if (haveTexCoords) {
                    if (haveTangents)
                        h[i] = new HitableTriangle<false, false, true, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<false, false, true, false>(instance, indices);
                } else {
                    if (haveTangents)
                        h[i] = new HitableTriangle<false, false, false, true>(instance, indices);
                    else
                        h[i] = new HitableTriangle<false, false, false, false>(instance, indices);
                }
            }
        }
    }
    return h;
}

}
