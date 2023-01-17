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

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

static_assert(sizeof(vec2) == 2 * sizeof(float));
static_assert(sizeof(vec3) == 3 * sizeof(float));
static_assert(sizeof(vec4) == 4 * sizeof(float));

int main(void)
{
    vec2 a(1.0f, 2.0f);
    vec3 b = a.xxy();
    fprintf(stderr, "b: %g %g %g\n", b.x(), b.y(), b.z());

    b = a.xxx();
    vec3 c = inversesqrt(b);
    fprintf(stderr, "c: %g %g %g\n", c.x(), c.y(), c.z());

    vec3 d = -d.xyz();
    fprintf(stderr, "d: %g %g %g\n", d.x(), d.y(), d.z());

    d = -1.0f * d.xyz();
    fprintf(stderr, "d: %g %g %g\n", d.x(), d.y(), d.z());

    fprintf(stderr, "%d\n", (d == d.xyz() ? 1 : 0));
    fprintf(stderr, "%d\n", (d.xyz() == d ? 1 : 0));
    fprintf(stderr, "%d\n", (d.xyz() == d.xyz() ? 1 : 0));
    //vec3 d = inversesqrt(b.x()yz);
    //fprintf(stderr, "d: %g %g %g\n", d.x(), d.y(), d.z());
    
    vec3 e;
    e = smoothstep(d.zyx(), d.xyz(), d);
    e = smoothstep(d, d, d);

    fprintf(stderr, "%g\n", clamp(0.6f / 0.0f, 0.0f, 1.0f));

    return 0;
}
