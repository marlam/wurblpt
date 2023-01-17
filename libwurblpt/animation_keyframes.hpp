/*
 * Copyright (C) 2012, 2013, 2014, 2017, 2018, 2019, 2020, 2021
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

#include <vector>

#include "animation.hpp"


namespace WurblPT {

/**
 * \brief The Animation class.
 *
 * This class describes an animation through a set key frames, which are points
 * in time at which the transformation of an object is known.
 *
 * Transformations at arbitrary points in time are interpolated from these key frames.
 *
 * The translation of a target is measured relative to the origin, and its
 * orientation is given by a rotation angle around a rotation axis.
 *
 * Example: if you animate an object and your camera is in the origin, looking
 * along -z, with up vector +y, and everything is measured in meters:
 * - Target translation (0, 0, -0.2) means that the origin of the target is
 *   directly in front of the camera at 20cm distance.
 * - Zero rotation means that the target is upright, while a rotation of 180
 *   degrees around the z axis means that it is upside down.
 */
class AnimationKeyframes : public Animation
{
public:
    /**
     * \brief The Keyframe class.
     *
     * This class describes one key frame, consisting of a point in time and
     * the transformation of a target at this point in time.
     */
    class Keyframe
    {
    public:
        float t;            /**< \brief Key frame time in seconds. */
        Transformation transformation; /**< \brief Transformation of the target at time t. */

        Keyframe() : t(0), transformation() {}
        Keyframe(float time, const Transformation& transf) : t(time), transformation(transf) {}
    };

private:
    std::vector<Keyframe> _keyframes; // list of keyframes, sorted by ascending time

    void findKeyframeIndices(float t, int& lowerIndex, int& higherIndex) const
    {
        // Binary search for the two nearest keyframes.
        // At this point we know that if we have only one keyframe, then its
        // time stamp is the requested time stamp so we have an exact match below.
        int a = 0;
        int b = _keyframes.size() - 1;
        while (b >= a) {
            int c = (a + b) / 2;
            if (_keyframes[c].t < t) {
                a = c + 1;
            } else if (_keyframes[c].t > t) {
                b = c - 1;
            } else {
                // exact match
                lowerIndex = c;
                higherIndex = c;
                return;
            }
        }
        // Now we have two keyframes: b is the lower one and a is the higher one.
        lowerIndex = b;
        higherIndex = a;
    }

public:
    /** \brief Constructor
     *
     * Constructs a default animation (one keyframe at t=0 with default transformation).
     */
    AnimationKeyframes()
    {
    }

    /** \brief Destructor */
    virtual ~AnimationKeyframes()
    {
    }

    /** \brief Constructor
     *
     * Constructs a very simple animation with two keyframes.
     */
    explicit AnimationKeyframes(float t0, const Transformation& T0, float t1, const Transformation& T1)
    {
        addKeyframe(t0, T0);
        addKeyframe(t1, T1);
    }

    /** \brief Constructor
     *
     * Constructs an animation from the given list of keyframes
     */
    explicit AnimationKeyframes(const std::vector<Keyframe>& keyframes) :
        _keyframes(keyframes)
    {
    }

    /** \brief Get the current list of keyframes, sorted by time in ascending order */
    const std::vector<Keyframe>& keyframes()
    {
        return _keyframes;
    }

    /** \brief Add a key frame. If a key frame with the same time stamp already exists,
     * it is overwritten. */
    void addKeyframe(const Keyframe& keyframe)
    {
        if (_keyframes.size() == 0) {
            _keyframes.push_back(keyframe);
        } else if (keyframe.t > endTime()) {
            // fast path for common case
            _keyframes.push_back(keyframe);
        } else if (keyframe.t < startTime()) {
            // fast path for common case
            _keyframes.insert(_keyframes.begin(), keyframe);
        } else {
            int lowerIndex, higherIndex;
            findKeyframeIndices(keyframe.t, lowerIndex, higherIndex);
            if (lowerIndex == higherIndex) {
                _keyframes[lowerIndex] = keyframe;
            } else {
                _keyframes.insert(_keyframes.begin() + higherIndex, keyframe);
            }
        }
    }

    /** \brief Convenience wrapper for \a addKeyframe(). */
    void addKeyframe(float time, const Transformation& transf)
    {
        addKeyframe(Keyframe(time, transf));
    }

    /** \brief Get start time of animation.
     *
     * Returns the time of the first keyframe.
     */
    float startTime() const
    {
        return _keyframes.size() == 0 ? 0.0f : _keyframes.front().t;
    }

    /** \brief Get end time of animation.
     *
     * Returns the time of the first keyframe.
     */
    float endTime() const
    {
        return _keyframes.size() == 0 ? 0.0f : _keyframes.back().t;
    }


    /** \brief Interpolate transformation
     *
     * \param t         Point in time, in seconds
     *
     * Returns the transformation at the given point in time.
     */
    virtual Transformation at(float t) const override
    {
        // Catch corner cases
        if (_keyframes.size() == 0) {
            return Transformation();
        } else if (t <= startTime()) {
            return _keyframes.front().transformation;
        } else if (t >= endTime()) {
            return _keyframes.back().transformation;
        }

        int lowerIndex, higherIndex;
        findKeyframeIndices(t, lowerIndex, higherIndex);
        if (lowerIndex == higherIndex) {
            return _keyframes[lowerIndex].transformation;
        }

        // Compute alpha value for interpolation.
        float alpha = 1.0f - (_keyframes[higherIndex].t - t) / (_keyframes[higherIndex].t - _keyframes[lowerIndex].t);

        // Interpolate transformation
        return mix(
                _keyframes[lowerIndex].transformation,
                _keyframes[higherIndex].transformation, alpha);
    }
};

}
