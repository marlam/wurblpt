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

#include "material.hpp"
#include "prng.hpp"
#include "sampler.hpp"


namespace WurblPT {

class MaterialPhaseFunctionIsotropic final : public Material
{
private:
    const vec4 _albedo;

public:
    MaterialPhaseFunctionIsotropic(const vec4& albedo) :
        _albedo(albedo)
    {
    }

    MaterialPhaseFunctionIsotropic(const vec3& albedo) :
        _albedo(albedo, 0.0f)
    {
    }

    virtual ScatterRecord scatter(const Ray& ray, const HitRecord& /* hit */, Prng& prng) const override
    {
        // TODO: this medium should probably have its own refractive index
        // which should be put into the ScatterRecord.
        return ScatterRecord(ScatterExplicit,
                Sampler::onUnitSphere(prng.in01x2()),
                _albedo,
                ray.refractiveIndex);
    }
};

}
