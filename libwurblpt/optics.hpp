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

#include <limits>

#include "gvm.hpp"
#include "sampler.hpp"
#include "prng.hpp"


namespace WurblPT {

class Projection
{
public:
    float t, b, r, l;    // frustum at near=1

    // Create from Frustum:
    Projection(float l_, float r_, float b_, float t_) :
        t(t_), b(b_), r(r_), l(l_)
    {
    }

    // Create from vertical opening angle (in degrees):
    Projection(float vfov, float aspectRatio) :
        t(tan(vfov * 0.5f)),
        b(-t),
        r(t * aspectRatio),
        l(-r)
    {
    }

    // Create from camera intrinsic parameters as used by OpenCV:
    Projection(unsigned int width, unsigned int height,
            const vec2& centerPixel, const vec2& focalLength) :
        t((height - centerPixel.y()) / focalLength.y()),
        b((0.0f   - centerPixel.y()) / focalLength.y()),
        r((width  - centerPixel.x()) / focalLength.x()),
        l((0.0f   - centerPixel.x()) / focalLength.x())
    {
    }

    Projection() : Projection(radians(60.0f), 4.0f / 3.0f)
    {
    }

    float vFov() const
    {
        return atan(t) - atan(b);
    }

    float hFov() const
    {
        return atan(r) - atan(l);
    }

    float aspectRatio() const
    {
        return (r - l) / (t - b);
    }

    vec2 center() const
    {
        return vec2(l / (l - r), b / (b - t));
    }

    vec2 focalLength() const
    {
        return vec2(1.0f / (r - l), 1.0f / (t - b));
    }

    vec2 inverseFocalLength() const
    {
        return vec2(r - l, t - b);
    }

    vec2 centerPixel(unsigned int w, unsigned int h) const
    {
        return center() * vec2(w, h);
    }

    vec2 focalLengthInPixels(unsigned int w, unsigned int h) const
    {
        return focalLength() * vec2(w, h);
    }
};

/* This implements three lens distortions models:
 *
 * 1. RadialAndPlanar
 *
 *    Supports parameters k1, k2, p1, p2. Closed-form undistort().
 *
 *    M. Lambers, H. Sommerhoff, and A. Kolb. "Realistic Lens Distortion Rendering".
 *    In: Proc. Int. Conf. in Central Europe on Computer Graphics, Visualization and
 *    Computer Vision (WSCG). June 2018. http://wscg.zcu.cz/wscg2018/2018-WSCG-Papers-Separated.html
 *
 *    There is deactivated code below that also considers k3.
 *
 *    Both variants of this method (with or without k3) may fail for specific parameter sets,
 *    which likely is related to the assumption that the entries of the Jacobi matrix are << 1
 *    (see also "Geometric camera calibration using circular control points" by Heikkilä).
 *
 * 2. RadialOnly
 *
 *    Supports parameters k1, k2, k3. Closed-form undistort().
 *
 *    P. Drap and J. Lefèvre. "An exact formula for calculating inverse radial lens
 *    distortions." MDPI Sensors 16, no. 6 (2016): 807.
 *
 * 3. OpenCV
 *
 *    Supports k1, k2, k3, p1, p2. Could be extended to support more.
 *
 *    Iterative undistort() function. For the vast majority of cases, only very few
 *    iterations are necessary to limit the error to < 0.001 pixels. However there are
 *    cases where more than 100 iterations are required to reach that limit, and there
 *    are even failure cases with errors larger than several pixels!
 *
 * For image generation, we need the undistort() function to go from distorted
 * pixel coordinates in the output image to undistorted coordinates for ray
 * generation.
 *
 * If your distortion parameters allow it, you may want to use the RadialAndPlanar
 * or RadialOnly models to get cheaper undistort() without failure cases. However
 * many real-world parameters require the full OpenCV model...
 */
class LensDistortion
{
private:
    /* For the iterative undistort() of the OpenCV model: */
    static constexpr int maxIterations = 256;
    static constexpr float epsilon = 0.001f;
    // the epsilon is applied to the euclidean distance in pixel space

public:
    typedef enum {
        None,
        RadialAndPlanar,
        RadialOnly,
        OpenCV
    } Type;

    class Helper {
    public:
        vec2 center;
        vec2 focalLength;
        vec2 inverseFocalLength;
        unsigned int width;
        unsigned int height;
    };

    Type type;
    float k1, k2, k3, p1, p2;
    float b1, b2, b3, b4;

    LensDistortion() :
        type(None),
        k1(0.0f), k2(0.0f), k3(0.0f), p1(0.0f), p2(0.0f),
        b1(0.0f), b2(0.0f), b3(0.0f), b4(0.0f)
    {
    }

    // Constructor for the RadialAndPlanar model:
    LensDistortion(float k1_, float k2_, float p1_, float p2_) :
        type((k1_ == 0.0f && k2_ == 0.0f && p1_ == 0.0f && p2_ == 0.0f) ? None : RadialAndPlanar),
        k1(k1_), k2(k2_), k3(0.0f), p1(p1_), p2(p2_),
        b1(0.0f), b2(0.0f), b3(0.0f), b4(0.0f)
    {
    }

    // Constructor for the RadialOnly model:
    LensDistortion(float k1_, float k2_, float k3_) :
        type((k1_ == 0.0f && k2_ == 0.0f && k3_ == 0.0f) ? None : RadialOnly),
        k1(k1_), k2(k2_), k3(k3_), p1(0.0f), p2(0.0f),
        b1(-k1),
        b2(3.0f * k1 * k1 - k2),
        b3(-12.0f * k1 * k1 * k1 + 8.0f * k1 * k2 - k3),
        b4(55.0f * k1 * k1 * k1 * k1 - 55.0f * k1 * k1 * k2 + 5.0f * k2 * k2 + 10.0f * k1 * k3)
    {
    }

    // Constructor for the OpenCV model:
    LensDistortion(float k1_, float k2_, float k3_, float p1_, float p2_) :
        type((k1_ == 0.0f && k2_ == 0.0f && k3_ == 0.0f && p1_ == 0.0f && p2_ == 0.0f) ? None : OpenCV),
        k1(k1_), k2(k2_), k3(k3_), p1(p1_), p2(p2_),
        b1(0.0f), b2(0.0f), b3(0.0f), b4(0.0f)
    {
    }

    Helper getHelper(const Projection& projection, unsigned int width, unsigned int height) const
    {
        Helper h;
        h.center = projection.center();
        h.focalLength = projection.focalLength();
        h.inverseFocalLength = projection.inverseFocalLength();
        h.width = width;
        h.height = height;
        return h;
    }

    // p and q are in normalized image space coordinates [0,1]x[0,1]
    void distort(float& p, float& q, const Helper& helper) const
    {
        if (type == RadialAndPlanar || type == RadialOnly || type == OpenCV) {
            // The models are actually all the same, just with the unsupported
            // parameters set to zero (for RadialAndPlanar: k3=0; for RadialOnly: p1=p2=0).
            float s = (p - helper.center.x()) * helper.inverseFocalLength.x();
            float t = (q - helper.center.y()) * helper.inverseFocalLength.y();
            float r2 = s * s + t * t;
            float r4 = r2 * r2;
            float r6 = r4 * r2;
            float rd = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
            float term1 = 2.0f * s * t;
            float term2 = r2 + 2.0f * s * s;
            float term3 = r2 + 2.0f * t * t;
            float new_s = s * rd + p1 * term1 + p2 * term2;
            float new_t = t * rd + p1 * term3 + p2 * term1;
            p = new_s * helper.focalLength.x() + helper.center.x();
            q = new_t * helper.focalLength.y() + helper.center.y();
        }
    }

    // p and q are in normalized image space coordinates [0,1]x[0,1]
    void undistort(float& p, float& q, const Helper& helper) const
    {
        if (type == RadialAndPlanar) {
            float s = (p - helper.center.x()) * helper.inverseFocalLength.x();
            float t = (q - helper.center.y()) * helper.inverseFocalLength.y();
            float r2 = s * s + t * t;
            float r4 = r2 * r2;
#if 0
            /* This is a variant by Hendrik Sommerhoff that also considers k3.
             * To activate it, you also need a constructor for this method that takes k3 as an argument. */
            float r6 = r4 * r2;
            float r8 = r4 * r4;
            float d1 = k1 * r2 + k2 * r4 + k3 * r6;
            float d2 = 1.0f / (4.0f * k1 * r2 + 6.0f * k2 * r4 + 10.0f * k3 * r8 + 8.0f * p1 * t + 8.0f * p2 * s + 1.0f);
#else
            float d1 = k1 * r2 + k2 * r4;
            float d2 = 1.0f / (4.0f * k1 * r2 + 6.0f * k2 * r4 + 8.0f * p1 * t + 8.0f * p2 * s + 1.0f);
#endif
            p = (s - d2 * (d1 * s + 2.0f * p1 * s * t + p2 * (r2 + 2.0f * s * s))) * helper.focalLength.x() + helper.center.x();
            q = (t - d2 * (d1 * t + p1 * (r2 + 2.0f * t * t) + 2.0f * p2 * s * t)) * helper.focalLength.y() + helper.center.y();
        } else if (type == RadialOnly) {
            float s = (p - helper.center.x()) * helper.inverseFocalLength.x();
            float t = (q - helper.center.y()) * helper.inverseFocalLength.y();
            float r2 = s * s + t * t;
            float r4 = r2 * r2;
            float r6 = r4 * r2;
            float r8 = r4 * r4;
            float d = 1.0f + b1 * r2 + b2 * r4 + b3 * r6 + b4 * r8;
            p = s * d * helper.focalLength.x() + helper.center.x();
            q = t * d * helper.focalLength.y() + helper.center.y();
        } else if (type == OpenCV) {
            float s = (p - helper.center.x()) * helper.inverseFocalLength.x();
            float t = (q - helper.center.y()) * helper.inverseFocalLength.y();
            float s0 = s;
            float t0 = t;

            int i = 0;
            float squaredError = std::numeric_limits<float>::max();
            for (; i < maxIterations && squaredError >= epsilon * epsilon; i++) {
                // compute next estimate
                float r2 = s * s + t * t;
                float r4 = r2 * r2;
                float r6 = r4 * r2;
                float rd = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
                float invrd = 1.0f / rd;
                float ds = 2.0f * p1 * s * t + p2 * (r2 + 2.0f * s * s);
                float dt = 2.0f * p2 * s * t + p1 * (r2 + 2.0f * t * t);
                s = (s0 - ds) * invrd;
                t = (t0 - dt) * invrd;
                // compute error
                float ep = s * helper.focalLength.x() + helper.center.x();
                float eq = t * helper.focalLength.y() + helper.center.y();
                distort(ep, eq, helper);
                vec2 offsetInPixels = (vec2(ep, eq) - vec2(p, q)) * vec2(helper.width, helper.height);
                squaredError = dot(offsetInPixels, offsetInPixels);
            }
            //fprintf(stderr, "\n%03d iterations, error=%g at p=%g, q=%g\n", i, sqrt(squaredError), p, q);
            p = s * helper.focalLength.x() + helper.center.x();
            q = t * helper.focalLength.y() + helper.center.y();
        }
    }
};

class LensDepthOfField
{
public:
    float lensRadius;
    float focusDist;

    LensDepthOfField(float aperture = 0.0f, float focusDist_ = 1.0f) :
        lensRadius(aperture * 0.5f),
        focusDist(focusDist_)
    {
    }

    // Given a point P on an image plane at z=-1 in canonical camera space,
    // compute an offset and modify P in order to create the depth of field effect
    vec3 offset(vec3& P, Prng& prng) const
    {
        vec3 O(0.0f);
        if (lensRadius > 0.0f) {
            P *= focusDist;
            O = vec3(lensRadius * Sampler::inUnitDisk(prng.in01x2()), 0.0f);
        }
        return O;
    }
};

class Optics
{
public:
    Projection projection;
    LensDistortion distortion;
    LensDepthOfField depthOfField;

    Optics(const Projection& P = Projection(),
            const LensDistortion& LD = LensDistortion(),
            const LensDepthOfField& LDOF = LensDepthOfField()) :
        projection(P),
        distortion(LD),
        depthOfField(LDOF)
    {
    }
};

}
