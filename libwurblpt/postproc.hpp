/*
 * Copyright (C) 2008
 * Martin Lambers <marlam@marlam.de>
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

#include <algorithm>

#include <tgd/array.hpp>

#include "optics.hpp"
#include "color.hpp"
#include "texture.hpp"
#include "texture_image.hpp"


namespace WurblPT {

/* Convert RGB(float) to SRGB(uint8) */

inline TGD::Array<uint8_t> toSRGB(const TGD::Array<float>& img)
{
    TGD::Array<uint8_t> r(img.dimensions(), 3);
    r.globalTagList() = img.globalTagList();
    for (size_t d = 0; d < img.dimensionCount(); d++)
        r.dimensionTagList(d) = img.dimensionTagList(d);
    r.componentTagList(0).set("INTERPRETATION", "SRGB/R");
    r.componentTagList(1).set("INTERPRETATION", "SRGB/G");
    r.componentTagList(2).set("INTERPRETATION", "SRGB/B");
    for (size_t i = 0; i < r.elementCount(); i++) {
        for (size_t j = 0; j < r.componentCount(); j++) {
            float v = min(img[i][j], 1.0f);
            r[i][j] = float_to_byte(rgb_to_srgb_helper(v));
        }
    }
    return r;
}

/* Dynamic range reduction / tone mapping */

inline float maxLuminance(const TGD::Array<float>& img)
{
    float lum = 0.0f;
    for (size_t i = 0; i < img.elementCount(); i++) {
        vec3 rgb = vec3(img[i]);
        vec3 xyz = rgb_to_xyz(rgb);
        if (xyz.y() > lum)
            lum = xyz.y();
    }
    return lum;
}

inline TGD::Array<float> uniformRationalQuantization(const TGD::Array<float>& img, float maxVal, float brightness /* in [1,inft] */)
{
    /* The resulting luminances will be in [0,100], therefore the RGB values in [0,1],
     * which means you can directly convert the result to sRGB without further processing. */
    TGD::Array<float> r(img.description());
    for (size_t i = 0; i < r.elementCount(); i++) {
        vec3 rgb = vec3(img[i]);
        vec3 xyz = rgb_to_xyz(rgb);
        float old_y = xyz.y() / 100.0f;
        float new_y = brightness * old_y / ((brightness - 1.0f) * old_y + maxVal);
        xyz = adjust_y(xyz, new_y * 100.0f);
        rgb = xyz_to_rgb(xyz);
        r.set(i, { rgb.r(), rgb.g(), rgb.b() });
    }
    return r;
}

inline TGD::Array<float> scaleLuminance(const TGD::Array<float>& img, float factor, float clamp = 1.0f)
{
    TGD::Array<float> r(img.description());
    for (size_t i = 0; i < r.elementCount(); i++) {
        vec3 rgb = vec3(img[i]);
        vec3 xyz = rgb_to_xyz(rgb);
        float new_y = factor * xyz.y();
        if (clamp > 0.0f && new_y > 100.0f * clamp) {
            new_y = 100.0f * clamp;
        }
        xyz = adjust_y(xyz, new_y);
        rgb = xyz_to_rgb(xyz);
        r.set(i, { rgb.r(), rgb.g(), rgb.b() });
    }
    return r;
}

/* Filters */

inline TGD::Array<float> scale(const TGD::Array<float>& img, unsigned int newWidth, unsigned int newHeight)
{
    Texture* tex = createTextureImage(img, LinearizeSRGB_Off);

    TGD::Array<float> scaledImg({ newWidth, newHeight }, img.componentCount());
    scaledImg.globalTagList() = img.globalTagList();
    for (size_t d = 0; d < scaledImg.dimensionCount(); d++)
        scaledImg.dimensionTagList(d) = img.dimensionTagList(d);
    for (size_t c = 0; c < scaledImg.componentCount(); c++)
        scaledImg.componentTagList(c) = img.componentTagList(c);

    for (size_t y = 0; y < scaledImg.dimension(1); y++) {
        float tcy = (y + 0.5f) / scaledImg.dimension(1);
        for (size_t x = 0; x < scaledImg.dimension(0); x++) {
            float tcx = (x + 0.5f) / scaledImg.dimension(0);
            vec4 v = tex->value(vec2(tcx, tcy));
            for (size_t c = 0; c < scaledImg.componentCount(); c++) {
                scaledImg[{x, y}][c] = v[c];
            }
        }
    }

    delete tex;
    return scaledImg;
}

/* Selective despeckling: replace every pixel whose luminance is greatest in its
 * 3x3 neighborhood and at least \a minFactor times greater than the second-highest
 * luminance with the median pixel of the 3x3 neighborhood.
 * (Primitive way to eliminate fireflies without blurring features.)
 * Input and output are RGB image. */
inline TGD::Array<float> despeckle(const TGD::Array<float>& img, float minFactor = 1.25f)
{
    assert(img.dimensionCount() == 2);
    assert(img.componentCount() == 3);

    // Helper class to sort pixels according to luminance
    class RgbLumSorter
    {
    public:
        RgbLumSorter() {}

        bool operator() (const vec4& a, const vec4& b)
        {
            return (a.w() < b.w());
        }
    };
    RgbLumSorter rgbLumSorter;

    TGD::Array<float> out(img.description());
    vec4 rgbLumBuf[9];
    int w = img.dimension(0);
    int h = img.dimension(1);
    for (int y = 0; y < h; y++) {
        unsigned int uy = y;
        for (int x = 0; x < w; x++) {
            unsigned int ux = x;
            for (int r = -1; r <= +1; r++) {
                for (int c = -1; c <= +1; c++) {
                    unsigned int yy = clamp(y + r, 0, h - 1);
                    unsigned int xx = clamp(x + c, 0, w - 1);
                    unsigned int i = (r + 1) * 3 + c + 1;
                    vec3 rgb(img[{ xx, yy }]);
                    float lum = rgb_to_xyz(rgb).y();
                    rgbLumBuf[i] = vec4(rgb, lum);
                }
            }
            vec4 origRgbLum = rgbLumBuf[4];
            std::sort(rgbLumBuf, rgbLumBuf + 9, rgbLumSorter);
            vec3 outRgb = origRgbLum.rgb();
            if (origRgbLum.w() >= rgbLumBuf[8].w()
                    && origRgbLum.w() >= minFactor * rgbLumBuf[7].w()) {
                outRgb = rgbLumBuf[4].rgb();
            }
            out[{ ux, uy }][0] = outRgb.r();
            out[{ ux, uy }][1] = outRgb.g();
            out[{ ux, uy }][2] = outRgb.b();
        }
    }

    return out;
}

/* Lens distortion */

inline TGD::Array<float> applyLensDistortion(const TGD::Array<float>& input,
        const LensDistortion& distortion, const Projection& projection,
        bool forward /* true: distort input; false: undistort input */)
{
    Texture* tex = createTextureImage(input, LinearizeSRGB_Off);

    TGD::Array<float> output(input.dimensions(), input.componentCount());
    output.globalTagList() = input.globalTagList();
    for (size_t d = 0; d < output.dimensionCount(); d++)
        output.dimensionTagList(d) = input.dimensionTagList(d);
    for (size_t c = 0; c < output.componentCount(); c++)
        output.componentTagList(c) = input.componentTagList(c);

    unsigned int width = output.dimension(0);
    unsigned int height = output.dimension(1);
    float invWidth = 1.0f / width;
    float invHeight = 1.0f / height;
    LensDistortion::Helper distortionHelper = distortion.getHelper(projection, width, height);
    for (size_t y = 0; y < output.dimension(1); y++) {
        for (size_t x = 0; x < output.dimension(0); x++) {
            // output image coordinates:
            float p = (x + 0.5f) * invWidth;
            float q = (y + 0.5f) * invHeight;
            // apply lens distortion:
            if (forward) {
                distortion.undistort(p, q, distortionHelper);
            } else {
                distortion.distort(p, q, distortionHelper);
            }
            // get value from input image at modified coordinates
            vec4 v = tex->value(vec2(p, q));
            for (size_t c = 0; c < output.componentCount(); c++) {
                output[{x, y}][c] = v[c];
            }
        }
    }

    delete tex;
    return output;
}

inline TGD::Array<float> distort(const TGD::Array<float>& undistorted,
        const LensDistortion& distortion, const Projection& projection)
{
    return applyLensDistortion(undistorted, distortion, projection, true);
}

inline TGD::Array<float> undistort(const TGD::Array<float>& distorted,
        const LensDistortion& distortion, const Projection& projection)
{
    return applyLensDistortion(distorted, distortion, projection, false);
}

/* Conversion of distance measurements (typically ToF) to coordinates */

inline TGD::Array<float> cameraSpaceDistanceToCoords(
        const TGD::Array<float>& distances /* first component must contain distance */,
        const Projection& projection,
        const LensDistortion& distortion)
{
    Texture* tex = createTextureImage(distances, LinearizeSRGB_Off);

    TGD::Array<float> coords(distances.dimensions(), 3);
    coords.globalTagList() = distances.globalTagList();
    coords.componentTagList(0).set("INTERPRETATION", "x");
    coords.componentTagList(1).set("INTERPRETATION", "y");
    coords.componentTagList(2).set("INTERPRETATION", "z");

    unsigned int width = distances.dimension(0);
    unsigned int height = distances.dimension(1);
    float invWidth = 1.0f / width;
    float invHeight = 1.0f / height;
    LensDistortion::Helper distortionHelper = distortion.getHelper(projection, width, height);
    for (size_t y = 0; y < coords.dimension(1); y++) {
        for (size_t x = 0; x < coords.dimension(0); x++) {
            // undistorted image coordinates:
            float p = (x + 0.5f) * invWidth;
            float q = (y + 0.5f) * invHeight;
            // get distance from distorted image coordinates:
            float dp = p;
            float dq = q;
            distortion.distort(dp, dq, distortionHelper);
            float distance = tex->value(vec2(dp, dq)).r();
            // compute 3D coordinates:
            vec2 uv = (vec2(p, q) - projection.center()) * projection.inverseFocalLength();
            vec3 xyz = normalize(vec3(uv, 1.0f)) * distance;
            coords.set({ x, y }, { xyz.x(), xyz.y(), -xyz.z() });
        }
    }
    return coords;
}

inline TGD::Array<float> cameraSpaceToPMDSpace(
        const TGD::Array<float>& coords /* first 3 components must be x,y,z */)
{
    TGD::Array<float> pmdspace(coords.dimensions(), 3);
    pmdspace.globalTagList() = coords.globalTagList();
    pmdspace.componentTagList(0).set("INTERPRETATION", "x");
    pmdspace.componentTagList(1).set("INTERPRETATION", "y");
    pmdspace.componentTagList(2).set("INTERPRETATION", "z");

    for (size_t e = 0; e < pmdspace.elementCount(); e++) {
        float x = coords[e][0];
        float y = coords[e][1];
        float z = coords[e][2];
        y = -y;
        z = -z;
        pmdspace[e][0] = x;
        pmdspace[e][1] = y;
        pmdspace[e][2] = z;
    }
    return pmdspace;
}

/* Extracting components from arrays */

inline TGD::ArrayContainer extractComponents(const TGD::ArrayContainer& a, const std::vector<size_t>& componentList)
{
    TGD::ArrayContainer r(a.dimensions(), componentList.size(), a.componentType());
    r.globalTagList() = a.globalTagList();
    for (size_t d = 0; d < a.dimensionCount(); d++)
        r.dimensionTagList(d) = a.dimensionTagList(d);
    for (size_t c = 0; c < componentList.size(); c++) {
        assert(componentList[c] < a.componentCount());
        r.componentTagList(c) = a.componentTagList(componentList[c]);
        unsigned char* dst = static_cast<unsigned char*>(r.data());
        const unsigned char* src = static_cast<const unsigned char*>(a.data());
        for (size_t e = 0; e < a.elementCount(); e++) {
            std::memcpy(
                    dst + e * r.elementSize() + c                * r.componentSize(),
                    src + e * a.elementSize() + componentList[c] * a.componentSize(),
                    a.componentSize());
        }
    }
    return r;
}

inline TGD::ArrayContainer extractComponent(const TGD::ArrayContainer& a, size_t component)
{
    return extractComponents(a, { component });
}

}
