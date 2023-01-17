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

void doit(const Projection& projection, const vec2& origImgSpace)
{
    fprintf(stderr, "\nlrbt=%g %g %g %g", projection.l, projection.r, projection.b, projection.t);
    Prng prng(37);

    Optics optics(projection);
    Camera camera(optics);
    Camera::ImageSpaceHelper ish = camera.getImageSpaceHelper(800, 600);
    Camera::RayHelper rh = camera.getRayHelper(0.0f, 800, 600);

    fprintf(stderr, "origImgSpace=(%g %g)\n", origImgSpace.x(), origImgSpace.y());
    Ray ray = camera.getRay(origImgSpace.x(), origImgSpace.y(), 0.0f, 0.0f, rh, prng);
    fprintf(stderr, "ray.o=(%g %g %g)\n", ray.origin.x(), ray.origin.y(), ray.origin.z());
    fprintf(stderr, "ray.d=(%g %g %g)\n", ray.direction.x(), ray.direction.y(), ray.direction.z());
    vec2 imgSpace = camera.cameraSpaceToImageSpace(ray.direction, ish);
    fprintf(stderr, "imgSpace=(%g %g)\n", imgSpace.x(), imgSpace.y());
}

int main(void)
{
    doit(Projection(800, 600, vec2(123.f, 321.0f), vec2(400.0f)), vec2(0.0f, 0.0f));
    doit(Projection(800, 600, vec2(123.f, 321.0f), vec2(400.0f)), vec2(0.0f, 1.0f));
    doit(Projection(800, 600, vec2(123.f, 321.0f), vec2(400.0f)), vec2(1.0f, 0.0f));
    doit(Projection(800, 600, vec2(123.f, 321.0f), vec2(400.0f)), vec2(1.0f, 1.0f));
    doit(Projection(800, 600, vec2(123.f, 321.0f), vec2(400.0f)), vec2(0.5f, 0.5f));

    return 0;
}
