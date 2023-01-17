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

/* XYZ and related color spaces helper functions and values */

inline vec3 color_matching_function_approx(float lambda /* in nanometers */)
{
    // Analytic approximation of the CIE 1931 2 deg standard observer CMF.
    // Taken from the paper "Simple Analytic Approximations to the CIE XYZ
    // Color Matching Functions" by Wyman, Sloan, Shirley, JCGT 2(2), 2013.

    /* Note: these analytic approximations are considered good, but there still
     * is a noticable difference in rendering results compared to the tabulated
     * version below.
     * A more complex multi-lobe approximation that provides a better fit is
     * described in the same paper, but does not seem worth the extra
     * computational costs (for now). */

    if (lambda < 360.0f || lambda > 780.0f)
        return vec3(0.0f);
    float tmp;
    // x
    tmp = (lambda - 595.8f) / 33.33f;
    float xBigLobe = 1.065f * exp(-0.5f * tmp * tmp);
    tmp = (lambda - 446.8f) / 19.44f;
    float xSmallLobe = 0.3660f * exp(-0.5f * tmp * tmp);
    float x = xSmallLobe + xBigLobe;
    // y
    tmp = (log(lambda) - log(556.3f)) / 0.075f;
    float y = 1.014f * exp(-0.5f * tmp * tmp);
    // z
    tmp = (log(lambda) - log(449.8f)) / 0.051f;
    float z = 1.839f * exp(-0.5f * tmp * tmp);
    return vec3(x, y, z);
}

inline vec3 color_matching_function(float lambda /* in nanometers */)
{
    // Tables in 5nm resolution for the CIE 1931 2 deg Standard Observer CMF,
    // from lambda=360 to lambda=780. Copied from the code that comes with
    // the paper "Simple Analytic Approximations to the CIE XYZ
    // Color Matching Functions" by Wyman, Sloan, Shirley, JCGT 2(2), 2013.
    static float const xyz1931_5nm[85][3] = {
        { 0.000130f, 0.000004f, 0.000606f },  // 360 nm
        { 0.000232f, 0.000007f, 0.001086f },  // 365 nm
        { 0.000415f, 0.000012f, 0.001946f },  // 370 nm
        { 0.000742f, 0.000022f, 0.003486f },  // 375 nm
        { 0.001368f, 0.000039f, 0.006450f },  // 380 nm
        { 0.002236f, 0.000064f, 0.010550f },  // 385 nm
        { 0.004243f, 0.000120f, 0.020050f },  // 390 nm
        { 0.007650f, 0.000217f, 0.036210f },  // 395 nm
        { 0.014310f, 0.000396f, 0.067850f },  // 400 nm
        { 0.023190f, 0.000640f, 0.110200f },  // 405 nm
        { 0.043510f, 0.001210f, 0.207400f },  // 410 nm
        { 0.077630f, 0.002180f, 0.371300f },  // 415 nm
        { 0.134380f, 0.004000f, 0.645600f },  // 420 nm
        { 0.214770f, 0.007300f, 1.039050f },  // 425 nm
        { 0.283900f, 0.011600f, 1.385600f },  // 430 nm
        { 0.328500f, 0.016840f, 1.622960f },  // 435 nm
        { 0.348280f, 0.023000f, 1.747060f },  // 440 nm
        { 0.348060f, 0.029800f, 1.782600f },  // 445 nm
        { 0.336200f, 0.038000f, 1.772110f },  // 450 nm
        { 0.318700f, 0.048000f, 1.744100f },  // 455 nm
        { 0.290800f, 0.060000f, 1.669200f },  // 460 nm
        { 0.251100f, 0.073900f, 1.528100f },  // 465 nm
        { 0.195360f, 0.090980f, 1.287640f },  // 470 nm
        { 0.142100f, 0.112600f, 1.041900f },  // 475 nm
        { 0.095640f, 0.139020f, 0.812950f },  // 480 nm
        { 0.057950f, 0.169300f, 0.616200f },  // 485 nm
        { 0.032010f, 0.208020f, 0.465180f },  // 490 nm
        { 0.014700f, 0.258600f, 0.353300f },  // 495 nm
        { 0.004900f, 0.323000f, 0.272000f },  // 500 nm
        { 0.002400f, 0.407300f, 0.212300f },  // 505 nm
        { 0.009300f, 0.503000f, 0.158200f },  // 510 nm
        { 0.029100f, 0.608200f, 0.111700f },  // 515 nm
        { 0.063270f, 0.710000f, 0.078250f },  // 520 nm
        { 0.109600f, 0.793200f, 0.057250f },  // 525 nm
        { 0.165500f, 0.862000f, 0.042160f },  // 530 nm
        { 0.225750f, 0.914850f, 0.029840f },  // 535 nm
        { 0.290400f, 0.954000f, 0.020300f },  // 540 nm
        { 0.359700f, 0.980300f, 0.013400f },  // 545 nm
        { 0.433450f, 0.994950f, 0.008750f },  // 550 nm
        { 0.512050f, 1.000000f, 0.005750f },  // 555 nm
        { 0.594500f, 0.995000f, 0.003900f },  // 560 nm
        { 0.678400f, 0.978600f, 0.002750f },  // 565 nm
        { 0.762100f, 0.952000f, 0.002100f },  // 570 nm
        { 0.842500f, 0.915400f, 0.001800f },  // 575 nm
        { 0.916300f, 0.870000f, 0.001650f },  // 580 nm
        { 0.978600f, 0.816300f, 0.001400f },  // 585 nm
        { 1.026300f, 0.757000f, 0.001100f },  // 590 nm
        { 1.056700f, 0.694900f, 0.001000f },  // 595 nm
        { 1.062200f, 0.631000f, 0.000800f },  // 600 nm
        { 1.045600f, 0.566800f, 0.000600f },  // 605 nm
        { 1.002600f, 0.503000f, 0.000340f },  // 610 nm
        { 0.938400f, 0.441200f, 0.000240f },  // 615 nm
        { 0.854450f, 0.381000f, 0.000190f },  // 620 nm
        { 0.751400f, 0.321000f, 0.000100f },  // 625 nm
        { 0.642400f, 0.265000f, 0.000050f },  // 630 nm
        { 0.541900f, 0.217000f, 0.000030f },  // 635 nm
        { 0.447900f, 0.175000f, 0.000020f },  // 640 nm
        { 0.360800f, 0.138200f, 0.000010f },  // 645 nm
        { 0.283500f, 0.107000f, 0.000000f },  // 650 nm
        { 0.218700f, 0.081600f, 0.000000f },  // 655 nm
        { 0.164900f, 0.061000f, 0.000000f },  // 660 nm
        { 0.121200f, 0.044580f, 0.000000f },  // 665 nm
        { 0.087400f, 0.032000f, 0.000000f },  // 670 nm
        { 0.063600f, 0.023200f, 0.000000f },  // 675 nm
        { 0.046770f, 0.017000f, 0.000000f },  // 680 nm
        { 0.032900f, 0.011920f, 0.000000f },  // 685 nm
        { 0.022700f, 0.008210f, 0.000000f },  // 690 nm
        { 0.015840f, 0.005723f, 0.000000f },  // 695 nm
        { 0.011359f, 0.004102f, 0.000000f },  // 700 nm
        { 0.008111f, 0.002929f, 0.000000f },  // 705 nm
        { 0.005790f, 0.002091f, 0.000000f },  // 710 nm
        { 0.004109f, 0.001484f, 0.000000f },  // 715 nm
        { 0.002899f, 0.001047f, 0.000000f },  // 720 nm
        { 0.002049f, 0.000740f, 0.000000f },  // 725 nm
        { 0.001440f, 0.000520f, 0.000000f },  // 730 nm
        { 0.001000f, 0.000361f, 0.000000f },  // 735 nm
        { 0.000690f, 0.000249f, 0.000000f },  // 740 nm
        { 0.000476f, 0.000172f, 0.000000f },  // 745 nm
        { 0.000332f, 0.000120f, 0.000000f },  // 750 nm
        { 0.000235f, 0.000085f, 0.000000f },  // 755 nm
        { 0.000166f, 0.000060f, 0.000000f },  // 760 nm
        { 0.000117f, 0.000042f, 0.000000f },  // 765 nm
        { 0.000083f, 0.000030f, 0.000000f },  // 770 nm
        { 0.000059f, 0.000021f, 0.000000f },  // 775 nm
        { 0.000042f, 0.000015f, 0.000000f }   // 780 nm
    };
    constexpr size_t N = 85;
    constexpr float lambdaMin = 360.0f;
    constexpr float lambdaMax = 780.0f;
    constexpr float lambdaLen = lambdaMax - lambdaMin;

    vec3 xyz(0.0f, 0.0f, 0.0f);
    if (lambda >= lambdaMin && lambda <= lambdaMax) {
        float alpha = (lambda - lambdaMin) * (1.0f / lambdaLen);
        float i = alpha * (N - 1);
        size_t i0 = i;
        if (i - i0 > 1.0f - epsilon)
            i0++;
        xyz = vec3(xyz1931_5nm[i0]);
        if (i0 < N - 1) {
            size_t i1 = i0 + 1;
            float beta = i - i0;
            xyz = (1.0f - beta) * xyz + beta * vec3(xyz1931_5nm[i1]);
        }
    }
    return xyz;
}

inline float d65(float lambda /* in nanometers */)
{
    /* The following data was copied from Mitsuba 2. */
    /**
     * D65 illuminant data from CIE, expressed as relative spectral power distribution,
     * normalized relative to the power at 560nm.
     */
    constexpr float data[95] = {
        46.6383f,  49.3637f,  52.0891f,  51.0323f,  49.9755f,  52.3118f,  54.6482f,  68.7015f,
        82.7549f,  87.1204f,  91.486f,   92.4589f,  93.4318f,  90.057f,   86.6823f,  95.7736f,
        104.865f,  110.936f,  117.008f,  117.41f,   117.812f,  116.336f,  114.861f,  115.392f,
        115.923f,  112.367f,  108.811f,  109.082f,  109.354f,  108.578f,  107.802f,  106.296f,
        104.79f,   106.239f,  107.689f,  106.047f,  104.405f,  104.225f,  104.046f,  102.023f,
        100.0f,    98.1671f,  96.3342f,  96.0611f,  95.788f,   92.2368f,  88.6856f,  89.3459f,
        90.0062f,  89.8026f,  89.5991f,  88.6489f,  87.6987f,  85.4936f,  83.2886f,  83.4939f,
        83.6992f,  81.863f,   80.0268f,  80.1207f,  80.2146f,  81.2462f,  82.2778f,  80.281f,
        78.2842f,  74.0027f,  69.7213f,  70.6652f,  71.6091f,  72.979f,   74.349f,   67.9765f,
        61.604f,   65.7448f,  69.8856f,  72.4863f,  75.087f,   69.3398f,  63.5927f,  55.0054f,
        46.4182f,  56.6118f,  66.8054f,  65.0941f,  63.3828f,  63.8434f,  64.304f,   61.8779f,
        59.4519f,  55.7054f,  51.959f,   54.6998f,  57.4406f,  58.8765f,  60.3125f
    };
    constexpr size_t N = sizeof(data) / sizeof(data[0]);
    constexpr float lambdaMin = 360.0f;
    constexpr float lambdaMax = 830.0f;
    constexpr float lambdaLen = lambdaMax - lambdaMin;

    float r = 0.0f;
    if (lambda >= lambdaMin && lambda <= lambdaMax) {
        float alpha = (lambda - lambdaMin) * (1.0f / lambdaLen);
        float i = alpha * (N - 1);
        size_t i0 = i;
        if (i - i0 > 1.0f - epsilon)
            i0++;
        r = data[i0];
        if (i0 < N - 1) {
            size_t i1 = i0 + 1;
            float beta = i - i0;
            r = (1.0f - beta) * r + beta * data[i1];
        }
    }
    return r;
}

inline vec3 adjust_y(const vec3& xyz, float new_y)
{
    float sum = xyz.x() + xyz.y() + xyz.z();
    if (xyz.y() <= 0.0f || sum <= 0.0f)
        return vec3(0.0f);
    // keep old chromaticity in terms of x, y
    float x = xyz.x() / sum;
    float y = xyz.y() / sum;
    // apply new luminance
    float r = new_y / y;
    return vec3(r * x, new_y, r * (1.0f - x - y));
}

/* Color space conversion: RGB <-> XYZ */

// The matrix entries are the same as used by PBRT3 and Mitsuba2.
// Other values exist, some claiming to be more precise, but we stick
// to these standard values for comparability.
//
// RGB here means linear BT.709, sRGB means nonlinear.

inline vec3 rgb_to_xyz(const vec3& rgb)
{
    return 100.0f * vec3(
            (0.412453f * rgb.r() + 0.357580f * rgb.g() + 0.180423f * rgb.b()),
            (0.212671f * rgb.r() + 0.715160f * rgb.g() + 0.072169f * rgb.b()),
            (0.019334f * rgb.r() + 0.119193f * rgb.g() + 0.950227f * rgb.b()));
}

inline vec3 xyz_to_rgb(const vec3& xyz)
{
    return 0.01f * vec3(
            (+3.240479f * xyz.x() - 1.537150f * xyz.y() - 0.498535f * xyz.z()),
            (-0.969256f * xyz.x() + 1.875991f * xyz.y() + 0.041556f * xyz.z()),
            (+0.055648f * xyz.x() - 0.204023f * xyz.y() + 1.057311f * xyz.z()));
}

/* Color space conversion: RGB <-> sRGB */

inline float rgb_to_srgb_helper(float x)
{
    return (x <= 0.0031308f ? (x * 12.92f) : (1.055f * pow(x, 1.0f / 2.4f) - 0.055f));
}

inline vec3 rgb_to_srgb(const vec3& rgb)
{
    return vec3(rgb_to_srgb_helper(rgb.r()), rgb_to_srgb_helper(rgb.g()), rgb_to_srgb_helper(rgb.b()));
}

inline float srgb_to_rgb_helper(float x)
{
    return (x <= 0.04045f ? (x * (1.0f / 12.92f)) : pow((x + 0.055f) * (1.0f / 1.055f), 2.4f));
}

inline vec3 srgb_to_rgb(const vec3& srgb)
{
    return vec3(srgb_to_rgb_helper(srgb.r()), srgb_to_rgb_helper(srgb.g()), srgb_to_rgb_helper(srgb.b()));
}

/* Integer <-> Float conversions */

inline float byte_to_float(uint8_t x)
{
    return x / 255.0f;
}

inline float uint16_to_float(uint16_t x)
{
    return x / 65535.0f;
}

inline uint8_t float_to_byte(float x)
{
    return round(x * 255.0f);
}

inline uint16_t float_to_uint16(float x)
{
    return round(x * 65535.0f);
}

}
