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

/* This creates a normal map with a "Disco Ball" effect when applied to a sphere. */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

class DiscoBallNormalMap : public Texture
{
public:
    static constexpr int width = 1024;
    static constexpr int height = 512;

    int verticalSteps;

    DiscoBallNormalMap(int verticalSteps) :
        verticalSteps(verticalSteps)
    {
    }

    static vec3 anglesToNormal(float theta, float phi)
    {
        return vec3(
                cos(theta) * sin(phi),
                sin(theta),
                cos(theta) * cos(phi)
                );
    }

    virtual vec4 value(const vec2& texcoords, float /* t */) const override
    {
        vec2 tc = fract(texcoords);
        // Reconstruct spherical coordinates from texture coordinates
        float theta = (tc.y() - 0.5f) * pi;     // in [-pi/2,pi/2]
        float phi = tc.x() * 2.0f * pi - pi;    // in [-pi,pi]
        // Reconstruct tangent space from spherical coordinates
        vec3 n = anglesToNormal(theta, phi);
        vec3 t = vec3(cos(phi), 0.0f, -sin(phi));
        TangentSpace ts = TangentSpace(n, t);

        /* Compute the new disco ball normal */

        // New angle theta:
        float yHalfStepLen = pi_2 / (0.5f * verticalSteps);
        int stepY = theta / yHalfStepLen;
        float newTheta = (stepY + sign(theta) * 0.5f) * yHalfStepLen;

        // New angle phi:
        float circleRadiusAtNewTheta = sin(max(0.0f, pi_2 - newTheta));
        float circleCirumferenceAtNewTheta = 2.0f * pi * circleRadiusAtNewTheta;
        int horizontalSteps = max(1.0f, circleCirumferenceAtNewTheta / yHalfStepLen);
        float xHalfStepLen = pi / (0.5f * horizontalSteps);
        float alphaX = (phi + pi) / (2.0f * pi);
        int stepX = alphaX * horizontalSteps;
        float newPhi = -pi + (stepX + 0.5f) * xHalfStepLen;

        // New normal:
        vec3 nrm = anglesToNormal(newTheta, newPhi);

        // Transform the new normal to tangent space
        vec3 tsnrm = ts.toTangentSpace(nrm);
        // Encode the tangent space normal using normal map conventions
        tsnrm = 0.5f * (tsnrm + vec3(1.0f));
        return vec4(tsnrm, 0.0f);
    }

    /* All following functions ensure that the exporter writes the image file in the correct format */

    virtual vec2 texelSize() const
    {
        return vec2(1.0f / width, 1.0f / height);
    }

    virtual unsigned int componentCount() const
    {
        return 3;
    }

    virtual TGD::Type componentType() const
    {
        return TGD::uint8;
    }

    virtual enum LinearizeSRGBType linearizeSRGBType() const
    {
        return LinearizeSRGB_Off;
    }
};

int main(void)
{
    Scene scene;
    Texture* normalTex = scene.take(new DiscoBallNormalMap(23));
    Material* material = new MaterialLambertian(vec3(0.5f));
    material->normalTex = normalTex;
    scene.take(material);
    scene.take(new Sphere(vec3(0.0f), 1.0f, material));
    scene.exportToObj("disco-ball-normalmap");
    return 0;
}
