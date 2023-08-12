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

#include "sensor.hpp"


namespace WurblPT {

class SensorRGB final : public Sensor
{
private:
    TGD::Array<float> _frame;
    const float _minDistToLight, _maxDistToLight;

public:
    SensorRGB(unsigned int width, unsigned int height,
            float minDistToLight = 0.0f, float maxDistToLight = std::numeric_limits<float>::max()) :
        _frame({ width, height }, 3),
        _minDistToLight(minDistToLight), _maxDistToLight(maxDistToLight)
    {
        _frame.componentTagList(0).set("INTERPRETATION", "RED");
        _frame.componentTagList(1).set("INTERPRETATION", "GREEN");
        _frame.componentTagList(2).set("INTERPRETATION", "BLUE");
    }

    virtual unsigned int width() const override
    {
        return _frame.dimension(0);
    }

    virtual unsigned int height() const override
    {
        return _frame.dimension(1);
    }

    virtual void accumulateRadiance(
            const Ray& /* r */,
            unsigned int /* pathComponent */,
            float /* geometricPathLength */,
            const vec4& /* opticalPathLength */,
            float distanceToLight,
            const vec4& radiance,
            const HitRecord& /* hitRecord */,
            float /* t0 */, float /* t1 */,
            float* sampleAccumulator) const override
    {
        for (int i = 0; i < 3; i++) {
            if (distanceToLight >= _minDistToLight && distanceToLight <= _maxDistToLight) {
                sampleAccumulator[i] += radiance[i];
            }
        }
    }

    virtual void finishPixel(unsigned int pixelIndex, float invSamples, const float* sampleAccumulator) override
    {
        _frame[pixelIndex][0] = invSamples * sampleAccumulator[0];
        _frame[pixelIndex][1] = invSamples * sampleAccumulator[1];
        _frame[pixelIndex][2] = invSamples * sampleAccumulator[2];
    }

    virtual TGD::ArrayContainer* pixelArray() override
    {
        return &_frame;
    }

    const TGD::Array<float>& result() const
    {
        return _frame;
    }
};

}
