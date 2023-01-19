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

#include <memory>

#include "transformation.hpp"
#include "optics.hpp"
#include "animation.hpp"
#include "ray.hpp"
#include "prng.hpp"
#include "constants.hpp"


namespace WurblPT {

class Camera
{
public:
    enum SurroundMode {
        Surround_Off,
        Surround_180,
        Surround_360
    };

    SurroundMode surroundMode;
    float stereoscopicDistance;

    Optics optics;
    Transformation transformation;
    std::unique_ptr<const Animation> animation;

    class RayHelper
    {
    public:
        Transformation cameraAtT0;
        LensDistortion::Helper lensDistortionHelper;
    };

public:
    // You can have either a transformation or an animation for a camera, but not both.
    // If you use an animation, the camera will take ownership of it.

    // For surround cameras, the sensor resolution should be 1:1 for 180° and 2:1 for 360°.
    // For stereoscopic cameras (stereoscopicDistance > 0), double the height of the
    // sensor resolution because the left view will be stored in the upper half and
    // the right view in the bottom half.

    // Stereoscopic surround videos can never be entirely correct because a stereoscopic
    // view is only valid for one view orientation, but in surround videos you want the
    // user to choose the view orientation.
    // An approximation is made where each view direction is taken as the base orientation
    // of the stereoscopic camera. This is equivalent to the approach recommended by Google:
    // https://developers.google.com/static/vr/jump/rendering-ods-content.pdf

    // Note that surround cameras ignore the optics completely.

    Camera(SurroundMode surroundMode, float stereoscopicDistance, const Optics& optics, const Transformation& transformation = Transformation()) :
        surroundMode(surroundMode), stereoscopicDistance(stereoscopicDistance),
        optics(optics), transformation(transformation), animation(nullptr)
    {
    }

    Camera(SurroundMode surroundMode, float stereoscopicDistance, const Optics& optics, const Animation* animation) :
        surroundMode(surroundMode), stereoscopicDistance(stereoscopicDistance),
        optics(optics), transformation(), animation(animation)
    {
    }

    // The following two constructors are shortcuts for non-surround, non-stereoscopic cameras

    Camera(const Optics& optics, const Transformation& transformation = Transformation()) :
        surroundMode(Surround_Off), stereoscopicDistance(0.0f),
        optics(optics), transformation(transformation), animation(nullptr)
    {
    }

    Camera(const Optics& optics, const Animation* animation) :
        surroundMode(Surround_Off), stereoscopicDistance(0.0f),
        optics(optics), transformation(), animation(animation)
    {
    }

    Transformation at(float t = 0.0f) const
    {
        return (animation.get() ? animation->at(t) : transformation);
    }

    RayHelper getRayHelper(float t0, unsigned int width, unsigned int height) const
    {
        RayHelper rh;
        rh.cameraAtT0 = at(t0);
        rh.lensDistortionHelper = optics.distortion.getHelper(optics.projection, width, height);
        return rh;
    }

    // p and q are in normalized image space coordinates [0,1]x[0,1]
    Ray getRay(float p, float q, float t0, float t1, const RayHelper& rayHelper,
            Prng& prng, bool withRandomness = true) const
    {
        /* First compute origin and direction in canonical camera space */

        // Stereoscopic camera handling
        float stereoscopicShift = 0.0f;
        if (stereoscopicDistance > 0.0f) {
            q *= 2.0f;
            if (q < 1.0f) {
                stereoscopicShift = -0.5f * stereoscopicDistance;
            } else {
                q -= 1.0f;
                stereoscopicShift = +0.5f * stereoscopicDistance;
            }
        }

        // Create ray origin O and direction D
        vec3 O, D;
        if (surroundMode == Surround_Off) {
            // Apply lens distortion: p,q correspond to a distorted output image, so we have to apply undistort()
            optics.distortion.undistort(p, q, rayHelper.lensDistortionHelper);
            // Create point on image plane in distance 1
            vec3 P = vec3(
                    mix(optics.projection.l, optics.projection.r, p),
                    mix(optics.projection.b, optics.projection.t, q),
                    -1.0f);
            // Create origin around zero and modify P according to depth of field
            O = vec3(0.0f);
            if (withRandomness)
                O = optics.depthOfField.offset(P, prng);
            // Compute direction
            D = P - O;
            // Apply the stereoscopic shift to O
            O += vec3(stereoscopicShift, 0.0f, 0.0f);
        } else {
            // Create direction according to latitude and longitude
            float lon = (2.0f * p - 1.0f) * pi;
            if (surroundMode == Surround_180)
                lon *= 0.5f;
            float lat = (q - 0.5f) * pi;
            D = vec3(
                    cos(lat) * sin(lon),
                    sin(lat),
                    -cos(lat) * cos(lon));
            // Create the origin according to the stereoscopic shift
            O = vec3(-cos(lon), 0.0f, -sin(lon)) * stereoscopicShift;
        }

        /* Then apply transformation and return the result */

        // Get transformation
        float t = t0;
        Transformation T = rayHelper.cameraAtT0;
        if (t0 != t1 && withRandomness) {
            t += prng.in01() * (t1 - t0);
            T = at(t);
        }
        // Create ray
        vec3 origin = T * O;
        vec3 direction = T.rotation * D;
        return Ray(origin, normalize(direction), t, refractiveIndexOfVacuum);
    }

    class ImageSpaceHelper
    {
    public:
        float P00, P11, P03, P13;
        LensDistortion::Helper lensDistortionHelper;
    };

    ImageSpaceHelper getImageSpaceHelper(unsigned int width, unsigned int height) const
    {
        ImageSpaceHelper ish;
        ish.P00 = 2.0f / (optics.projection.r - optics.projection.l);
        ish.P11 = 2.0f / (optics.projection.t - optics.projection.b);
        ish.P03 = (optics.projection.r + optics.projection.l) / (optics.projection.r - optics.projection.l);
        ish.P13 = (optics.projection.t + optics.projection.b) / (optics.projection.t - optics.projection.b);
        ish.lensDistortionHelper = optics.distortion.getHelper(optics.projection, width, height);
        return ish;
    }

    vec2 cameraSpaceToImageSpace(const vec3& p, const ImageSpaceHelper& helper) const
    {
        assert(surroundMode == Surround_Off);   // TODO: implement for surround cameras
        assert(stereoscopicDistance <= 0.0f);   // TODO: implement for stereoscopic cameras
        // See the toMat4(frustum) in gvm.hpp, but here we are only interested in
        // 2D pixel space coordinates and we don't have near and far plane anyway,
        // so we only use the relevant entries of projection matrix P:
        vec4 projected = vec4(helper.P00 * p.x() + helper.P03 * p.z(), helper.P11 * p.y() + helper.P13 * p.z(), 0.0f /* ignored */, -p.z());
        vec2 ndc = projected.xy() / projected.w();
        vec2 imageCoord = (0.5f * ndc) + vec2(0.5f);
        optics.distortion.distort(imageCoord.x(), imageCoord.y(), helper.lensDistortionHelper);
        return imageCoord;
    }
};

}
