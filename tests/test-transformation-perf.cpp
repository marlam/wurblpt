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

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

int main(void)
{
    Prng prng(42);

    Transformation T(
            vec3(1.0f + prng.in01() * 17.0f, -prng.in01() * 23.0f, prng.in01() * 42.0f),
            toQuat(radians(-80.0f + prng.in01() * 160.0f), normalize(vec3(0.2f, 0.7f, 1.3f))),
            vec3(0.5f + prng.in01(), 0.5f + prng.in01(), 0.5f + prng.in01()));
    mat4 M = T.toMat4();

    const size_t N = 10000000000ULL;
    vec3 wsum(0.0f);
    for (size_t i = 0; i < N; i++) {
        vec3 v = vec3(2.0f * prng.in01() - 1.0f, 2.0f * prng.in01() - 1.0f, 2.0f * prng.in01() - 1.0f);
#if 1
        // test multiplication of transformation and vector
        vec3 w = T * v;
        wsum += w;
#elif 0
        // test multiplication of mat4 and vector
        vec3 w = (M * vec4(v, 1.0f)).xyz();
        wsum += w;
#else
        // test without multiplication to get the time spend on the prng
        wsum += v;
#endif
    }

    return int(wsum.x()) % 64;
}
