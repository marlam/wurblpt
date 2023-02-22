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

/* This tool converts 360° videos to conventional videos, stereo or mono.
 * Note that this is not the same as rendering the separately! Think
 * about camera positions and orientations. */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>

using namespace WurblPT;


TGD::ArrayContainer getLeft(const TGD::ArrayContainer& a)
{
    assert(a.dimensionCount() == 2);
    TGD::ArrayContainer r({ a.dimension(0), a.dimension(1) / 2 }, a.componentCount(), a.componentType());
    std::memcpy(r.data(), a.get(a.elementCount() / 2), r.dataSize());
    return r;
}

TGD::ArrayContainer getRight(const TGD::ArrayContainer& a)
{
    assert(a.dimensionCount() == 2);
    TGD::ArrayContainer r({ a.dimension(0), a.dimension(1) / 2 }, a.componentCount(), a.componentType());
    std::memcpy(r.data(), a.data(), r.dataSize());
    return r;
}

TGD::ArrayContainer getStereo(const TGD::ArrayContainer& l, const TGD::ArrayContainer& r)
{
    assert(l.dimensionCount() == 2);
    assert(r.dimensionCount() == 2);
    assert(l.dimension(0) == r.dimension(0));
    assert(l.elementSize() == r.elementSize());
    TGD::ArrayContainer s({ l.dimension(0), l.dimension(1) + r.dimension(1) }, l.componentCount(), l.componentType());
    std::memcpy(s.data(), r.data(), s.dataSize() / 2);
    std::memcpy(s.get(s.elementCount() / 2), l.data(), s.dataSize() / 2);
    return s;
}

TGD::ArrayContainer getConventionalFrom360Mono(const TGD::ArrayContainer& a, size_t width, size_t height, const Transformation& camT, float openingAngle)
{
    assert(a.dimensionCount() == 2);

    Scene scene;
    scene.take(new EnvironmentMapEquiRect(scene.take(createTextureImage(a)),
                EnvironmentMapEquiRect::CompatibilitySurroundVideo));
    scene.updateBVH();

    SensorRGB sensor(width, height);
    Optics optics(Projection(openingAngle, sensor.aspectRatio()));
    Camera camera(optics, camT);

    Parameters params;
    params.maxPathComponents = 1;
    params.randomizeRayOverPixel = false;

    mcpt(sensor, camera, scene, 1, 0.0f, 0.0f, params);
    if (a.componentType() == TGD::uint8)
        return toSRGB(sensor.result());
    else
        return sensor.result();
}

int main(int argc, char* argv[])
{
    if (argc != 7) {
        fprintf(stderr, "Usage: %s width height angle,x,y,z vfov <input> <output>\n", argv[0]);
        fprintf(stderr, "Recommendations:\n"
                "- use half of input width/height\n"
                "- use 0,0,0,0 for default camera transformation\n"
                "- use vfov=50, which is what Bino and VLC do\n");
        return 1;
    }

    TGD::Error err = TGD::ErrorNone;
    unsigned int width = atoi(argv[1]);
    unsigned int height = atoi(argv[2]);
    float angle, axisx, axisy, axisz;
    sscanf(argv[3], "%f,%f,%f,%f", &angle, &axisx, &axisy, &axisz);
    Transformation camT;
    if (angle != 0.0f && (axisx != 0.0f || axisy != 0.0f || axisz != 0.0f))
        camT = Transformation(vec3(0.0f), toQuat(radians(angle), vec3(axisx, axisy, axisz)));
    float vfov = radians(atof(argv[4]));
    TGD::Importer importer(argv[5]);
    TGD::Exporter exporter(argv[6]);

    for (;;) {
        if (!importer.hasMore(&err)) {
            if (err != TGD::ErrorNone) {
                fprintf(stderr, "%s: %s: %s\n", argv[0], importer.fileName().c_str(), TGD::strerror(err));
            }
            break;
        }
        TGD::ArrayContainer a = importer.readArray(&err);
        if (err != TGD::ErrorNone) {
            fprintf(stderr, "%s: %s: %s\n", argv[0], importer.fileName().c_str(), TGD::strerror(err));
            break;
        }

        TGD::ArrayContainer result;
        if (a.dimension(0) == a.dimension(1)) {
            TGD::ArrayContainer l = getConventionalFrom360Mono(getLeft(a),  width, height / 2, camT, vfov);
            TGD::ArrayContainer r = getConventionalFrom360Mono(getRight(a), width, height / 2, camT, vfov);
            result = getStereo(r, l); // XXX why is this reversed?
        } else {
            result = getConventionalFrom360Mono(a, width, height, camT, vfov);
        }

        err = exporter.writeArray(result);
        if (err != TGD::ErrorNone) {
            fprintf(stderr, "%s: %s: %s\n", argv[0], exporter.fileName().c_str(), TGD::strerror(err));
            break;
        }
    }

    return (err == TGD::ErrorNone ? 0 : 1);
}
