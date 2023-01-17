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

static powitacq_rgb::Vector3f toVec3fRgb(const vec3& v) { return powitacq_rgb::Vector3f(v.x(), v.y(), v.z()); }
static powitacq::Vector3f toVec3fSpectral(const vec3& v) { return powitacq::Vector3f(v.x(), v.y(), v.z()); }
static vec3 fromVec3fRgb(const powitacq_rgb::Vector3f& v) { return vec3(v.x(), v.y(), v.z()); }

vec3 spectrumToRgb(const powitacq::BRDF& brdf, const powitacq::Spectrum& spectrum)
{
    vec3 xyz(0.0f);
    float N = 0.0f;
    for (size_t i = 0; i < brdf.wavelengths().size(); i++) {
        float lambda = brdf.wavelengths()[i];
        vec3 cmf = color_matching_function(lambda);
        N += d65(lambda) * cmf.y();
        xyz += spectrum[i] * d65(lambda) * cmf;
    }
    xyz *= (brdf.wavelengths()[brdf.wavelengths().size() - 1] - brdf.wavelengths()[0]) / (brdf.wavelengths().size());
    N *= (brdf.wavelengths()[brdf.wavelengths().size() - 1] - brdf.wavelengths()[0]) / (brdf.wavelengths().size());
    fprintf(stderr, "xyzD65ReflectionNormalization = 100.0f / %gf\n", N);
    xyz *= 100.0f / N;

    return xyz_to_rgb(xyz);
}

int main(void)
{
    fprintf(stderr, "Color matching functions: tabulated data vs approximation:\n");
    for (int lambda = 360; lambda <= 780; lambda += 1) {
        vec3 xyz_tab = color_matching_function(lambda);
        vec3 xyz_approx = color_matching_function_approx(lambda);
        vec3 diff = xyz_approx - xyz_tab;
        fprintf(stderr, "lambda=%3d: tab=(%g %g %g) approx=(%g %g %g) diff=(%g %g %g)\n",
                lambda, xyz_tab.x(), xyz_tab.y(), xyz_tab.z(),
                xyz_approx.x(), xyz_approx.y(), xyz_approx.z(),
                diff.x(), diff.y(), diff.z());
    }

    const powitacq_rgb::BRDF brdfRgb("cm_white_rgb.bsdf");
    const powitacq::BRDF brdfSpectral("cm_white_spec.bsdf");

    vec3 wo = normalize(vec3(0.1f, 0.0f, 1.0f));
    vec3 wi = normalize(vec3(0.0f, 0.0f, 1.0f));

    powitacq_rgb::Vector3f attenuationRgb = brdfRgb.eval(toVec3fRgb(wi), toVec3fRgb(wo));
    float pRgb = brdfRgb.pdf(toVec3fRgb(wi), toVec3fRgb(wo));
    powitacq::Spectrum attenuationSpectrum = brdfSpectral.eval(toVec3fSpectral(wi), toVec3fSpectral(wo));
    float pSpectrum = brdfSpectral.pdf(toVec3fSpectral(wi), toVec3fSpectral(wo));

    vec3 rgb = fromVec3fRgb(attenuationRgb);
    vec3 rgbFromSpectrum = spectrumToRgb(brdfSpectral, attenuationSpectrum);

    fprintf(stderr, "RGL Material test:\n");
    fprintf(stderr, "  RGB: (%g, %g, %g), p=%g\n", rgb.r(), rgb.g(), rgb.b(), pRgb);
    fprintf(stderr, "  RGB from Spectrum: (%g, %g, %g), p=%g\n", rgbFromSpectrum.r(), rgbFromSpectrum.g(), rgbFromSpectrum.b(), pSpectrum);

    return 0;
}
