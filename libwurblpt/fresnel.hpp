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

#include "gvm.hpp"


namespace WurblPT {

inline float fresnelSchlickR0(float ourIndexOfRefraction, float theirIndexOfRefraction)
{
    float r0 = (theirIndexOfRefraction - ourIndexOfRefraction)
             / (theirIndexOfRefraction + ourIndexOfRefraction);
    r0 *= r0;
    return r0;
}

inline float fresnelSchlick(float r0, float cosTheta)
{
    float t = 1.0f - cosTheta;
    float t_squared = t * t;
    return r0 + (1.0f - r0) * t_squared * t_squared * t;
}

inline vec4 fresnelSchlick(const vec4& r0, float cosTheta)
{
    float t = 1.0f - cosTheta;
    float t_squared = t * t;
    return r0 + (vec4(1.0f) - r0) * t_squared * t_squared * t;
}

// Exact, unpolarized Fresnel equation as described in
// https://en.wikipedia.org/wiki/Fresnel_equations#Power_(intensity)_reflection_and_transmission_coefficients
inline float fresnelUnpolarized(
        float cosI, // cosIncident
        float cosT, // cosTransmitted
        float n1,   // refractiveIndexIncident,
        float n2    // refractiveIndexTransmitted
        )
{
    // for s-polarized light:
    float Fs = (n1 * cosI - n2 * cosT) / (n1 * cosI + n2 * cosT);
    Fs *= Fs;
    // for p-polarized light:
    float Fp = (n1 * cosT - n2 * cosI) / (n1 * cosT + n2 * cosI);
    Fp *= Fp;
    // for unpolarized, simply take the average
    return 0.5f * (Fs + Fp);
}

}
