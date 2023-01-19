/*
 * Copyright (C) 2023
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

#include <cstdint>

#include "gvm.hpp"


namespace WurblPT {

/* This is a pseudo random number generator based on the recommendations
 * listed at https://prng.di.unimi.it/ :
 * - it is the xoshiro128+ variant since that is recommended to produce
 *   32 bit values that are then used to produce uniformly distributed
 *   single precision floating point numbers in [0,1)
 * - it used splitmix64() to generate a seed for a given pixel index
 *   with the hope that this reduces correlation between the sequences
 *   generated for different pixels.
 *
 * This uses code from https://prng.di.unimi.it/xoshiro128plus.c
 * Written in 2018 by David Blackman and Sebastiano Vigna,
 * <http://creativecommons.org/publicdomain/zero/1.0/>
 */

class Prng
{
private:
    uint32_t s[4];

    static uint32_t rotl(uint32_t x, int k)
    {
        return (x << k) | (x >> (32 - k));
    }

    uint32_t next()
    {
        const uint32_t result = s[0] + s[3];
        const uint32_t t = s[1] << 9;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl(s[3], 11);
        return result;
    }

    uint64_t splitmix64(uint64_t x)
    {
        uint64_t z = (x += 0x9e3779b97f4a7c15);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
        z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
        return z ^ (z >> 31);
    }

public:
    Prng(unsigned int pixelIndex)
    {
        uint64_t splitmix64Seed = pixelIndex;
        splitmix64Seed += 42; // avoid all-zero seed
        uint64_t s01 = splitmix64(splitmix64Seed);
        uint64_t s23 = splitmix64(s01);
        s[0] = s01 >> 32ULL;
        s[1] = s01 & 0xffffffffULL;
        s[2] = s23 >> 32ULL;
        s[3] = s23 & 0xffffffffULL;
    }

    float in01()
    {
        uint32_t x = next();
        return (x >> 8) * 0x1.0p-24;
    }

    vec2 in01x2()
    {
        return vec2(in01(), in01());
    }
};

}
