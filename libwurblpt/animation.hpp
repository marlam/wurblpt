/*
 * Copyright (C) 2019, 2020, 2021
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
#include <vector>

#include "transformation.hpp"


namespace WurblPT {

class Animation
{
public:
    Animation()
    {
    }

    virtual ~Animation()
    {
    }

    virtual Transformation at(float t) const = 0;
};

class AnimationCache
{
private:
    const std::vector<const Animation*>* _animations;
    float _t;
    std::vector<Transformation> _transformations;
    std::vector<mat4> _transformationMs;
    std::vector<mat3> _transformationNs;
    std::vector<bool> _initialized;

    void initIndexIfNecessary(int i)
    {
        assert(i >= 0 && size_t(i) < _animations->size());
        if (!_initialized[i]) {
            _transformations[i] = (*_animations)[i]->at(_t);
            _transformationMs[i] = _transformations[i].toMat4();
            _transformationNs[i] = _transformations[i].toNormalMatrix();
            _initialized[i] = true;
        }
    }

public:
    AnimationCache() : _animations(nullptr), _t(std::numeric_limits<float>::max())
    {
    }

    AnimationCache(const std::vector<const Animation*>& animations) :
        _animations(&animations),
        _t(std::numeric_limits<float>::max()),
        _transformations(animations.size()),
        _transformationMs(animations.size()),
        _transformationNs(animations.size()),
        _initialized(animations.size(), false)
    {
    }

    AnimationCache(const std::vector<const Animation*>& animations, float t) :
        AnimationCache(animations)
    {
        _t = t;
        for (size_t i = 0; i < _animations->size(); i++)
            initIndexIfNecessary(i);
    }

    void init(float t)
    {
        _t = t;
        for (size_t i = 0; i < _animations->size(); i++)
            _initialized[i] = false;
    }

    const Transformation& get(int animationIndex)
    {
        initIndexIfNecessary(animationIndex);
        return _transformations[animationIndex];
    }

    const mat4& getM(int animationIndex)
    {
        initIndexIfNecessary(animationIndex);
        return _transformationMs[animationIndex];
    }

    const mat3& getN(int animationIndex)
    {
        initIndexIfNecessary(animationIndex);
        return _transformationNs[animationIndex];
    }

    float t() const
    {
        return _t;
    }
};

}
