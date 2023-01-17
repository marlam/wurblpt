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

#include "scene_component.hpp"
#include "hitable_medium.hpp"


namespace WurblPT {

/* A medium, e.g. participating media, subsurface scattering etc.
 * Current restrictions: no importance sampling
 */
class Medium final : public SceneComponent
{
private:
    const std::vector<const Hitable*> _boundaryHitables;
    const float _density;
    const Material* _phaseFunction;

public:
    Medium(const std::vector<const Hitable*>& boundary, float density, const Material* phaseFunction) :
        _boundaryHitables(boundary),
        _density(density),
        _phaseFunction(phaseFunction)
    {
    }

    virtual std::vector<Hitable*> createHitables() const
    {
        Hitable* h = new HitableMedium(_boundaryHitables, _density, _phaseFunction);
        return std::vector<Hitable*>(1, h);
    }
};

}
