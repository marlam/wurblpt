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

#include <random>
#include <limits>
#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

int main(void)
{
    unsigned int spp_sqrt = (1U << 15U);

#if 1
    Prng prng(13);

    vec2 x(0.0f);
    for (unsigned int i = 0; i < spp_sqrt * spp_sqrt; i++) {
        x += Sampler::inUnitDisk(prng.in01x2());
    }
#else
    //std::mt19937_64 generator(42); // 2504 21.5
    std::mt19937 generator(42); // 5000 13.7
    //std::minstd_rand generator(42); // 8 14.9
    //std::ranlux24 generator(42); // 216 94.114
    //std::ranlux48 generator(42); // 120 305.1
    //std::knuth_b generator(42); // 2064 19.5
    std::uniform_real_distribution<float> uniformDistribution(0.0f, 1.0f);
    fprintf(stderr, "%zu\n", sizeof(generator));

    vec2 x(0.0f);
    for (unsigned int i = 0; i < spp_sqrt * spp_sqrt; i++) {
#if 1
        float u0 = uniformDistribution(generator);
        float u1 = uniformDistribution(generator);
#else
        float u0 = std::generate_canonical<float, std::numeric_limits<float>::digits>(generator);
        float u1 = std::generate_canonical<float, std::numeric_limits<float>::digits>(generator);
#endif
        // This is second the method from PBR3 13.6.2
        vec2 u = vec2(u0, u1);
        vec2 uOffset = 2.0f * u - vec2(1.0f, 1.0f);
        vec2 result;
        if (uOffset == vec2(0.0f, 0.0f)) {
            result = vec2(0.0f, 0.0f);
        } else {
            float theta, r;
            if (abs(uOffset.x()) > abs(uOffset.y())) {
                r = uOffset.x();
                theta = pi_4 * (uOffset.y() / uOffset.x());
            } else {
                r = uOffset.y();
                theta = pi_2 - pi_4 * (uOffset.x() / uOffset.y());
            }
            result = r * vec2(cos(theta), sin(theta));
        }
        x += result;
    }
#endif

    return x.x() + x.y();
}
