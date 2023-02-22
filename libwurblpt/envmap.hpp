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

#include <cassert>
#include <vector>
#include <algorithm>

#include "gvm.hpp"
#include "prng.hpp"
#include "texture.hpp"


namespace WurblPT {

/* The Importance Sampling support of the EnvironmentMap class works with any map parameterization (here:
 * equirectangular maps (EnvironmentMapEquiRect) and cube maps (EnvironmentMapCube)).
 * It is based on: M. Lambers. Parameterization-Independent Importance Sampling of Environment Maps. Tech. rep. Computer
 * Graphics Group, University of Siegen, Aug. 2022. doi: 10.48550/arXiv.2208.10815. */

class EnvironmentMap
{
private:
    /* The importance maps */
    int N;
    std::vector<float> M;
    std::vector<int> Ms;
    std::vector<float> Mcs;

    /* Equal-area map projection of a direction d on the unit sphere
     * onto a square map with coordinates in [0,1]x[0,1] */
    static vec2 m(const vec3& d)
    {
        // convert d to latitude and longitude
        float lat = asin(clamp(d.y(), -1.0f, +1.0f));
        float lon = atan2(-d.x(), d.z());
        // map from sphere to disk using Lambert Equal Area projection
        float r = sin(0.5f * (pi_2 - lat));
        float alpha = lon - pi_2;
        // map from disk to square using Shirley's equal-area method
        float u, v;
        if (alpha < -pi_4)
            alpha += 2.0f * pi;
        if (alpha < pi_4) {
            u = r;
            v = alpha * u / pi_4;
        } else if (alpha < pi_2 + pi_4) {
            v = r;
            u = -(alpha - pi_2) * v / pi_4;
        } else if (alpha < pi + pi_4) {
            u = -r;
            v = (alpha - pi) * u / pi_4;
        } else {
            v = -r;
            u = -(alpha - (pi + pi_2)) * v / pi_4;
        }
        return vec2(0.5f * (u + 1.0f), 0.5f * (v + 1.0f));
    }

    /* The inverse of the map projection m() */
    static vec3 invm(const vec2& uv)
    {
        float u = 2.0f * uv.x() - 1.0f;
        float v = 2.0f * uv.y() - 1.0f;
        // map from square to disk
        float r, alpha;
        if (u * u > v * v) {
            r = u;
            alpha = pi_4 * v / u;
        } else {
            r = v;
            if (abs(v) > 0.0f)
                alpha = pi_2 - pi_4 * u / v;
            else
                alpha = 0.0f;
        }
        // map from disk to sphere
        float lat = pi_2 - 2.0f * asin(r);
        float lon = alpha + pi_2;
        // convert to direction
        vec3 d = vec3(
                -cos(lat) * sin(lon),
                sin(lat),
                cos(lat) * cos(lon));
        return normalize(d);
    }

public:
    EnvironmentMap() : N(0)
    {
    }

    virtual ~EnvironmentMap()
    {
    }

    /* Initialization of the importance maps */
    void initializeImportanceSampling(int N)
    {
        this->N = N;
        M.resize(N * N);
        Ms.resize(N * N);
        Mcs.resize(N * N);

        // compute importance for each bin in the map
        float totalImportance = 0.0f;
        for (int y = 0; y < N; y++) {
            float v = (y + 0.5f) / N;
            for (int x = 0; x < N; x++) {
                float u = (x + 0.5f) / N;
                vec3 d = invm(vec2(u, v));
                vec4 v = L(d);
                float importance = v.x() + v.y() + v.z() + v.w();
                totalImportance += importance;
                M[y * N + x] = importance;
            }
        }
        // normalize so that the total importance is 1
        for (int i = 0; i < N * N; i++)
            M[i] /= totalImportance;

        // sort M according to descending importance
        for (int i = 0; i < N * N; i++)
            Ms[i] = i;
        std::sort(Ms.begin(), Ms.end(),
                [this](unsigned int i, unsigned int j) {
                return this->M[i] > this->M[j]; });

        // compute cumulative importance for Ms
        float sum = 0.0f;
        for (int i = 0; i < N * N; i++) {
            sum += M[Ms[i]];
            Mcs[i] = sum;
        }
    }

    /* Check if this map supports importance sampling; only then can p() and d() be called */
    bool supportsImportanceSampling() const
    {
        return N > 0;
    }

    /* Sampling of radiances */
    virtual vec4 L(const vec3& direction, float t = 0.0f) const = 0;

    /* Compute the pdf value for a given direction */
    float p(const vec3& direction) const
    {
        assert(supportsImportanceSampling());
        vec2 uv = m(direction);
        int x = uv.x() * N;
        int y = uv.y() * N;
        if (x >= N)
            x = N - 1;
        if (y >= N)
            y = N - 1;
        float q = M[y * N + x];
        float invBinSizeOnSphere = (N * N) * 0.25f * inv_pi;
        return q * invBinSizeOnSphere;
    }

    /* Generate a random light direction sample according to the pdf */
    vec3 d(Prng& prng) const
    {
        assert(supportsImportanceSampling());
        float r = prng.in01();
        // binary search for the bin that has the requested cumulative importance
        int a = 0;
        int b = N * N - 1;
        while (b > a + 1) {
            int c = (a + b) / 2;
            if (Mcs[c] < r) {
                a = c;
            } else {
                b = c;
            }
        }
        int bin = (Mcs[a] >= r ? a : b);
        // get the real index of the bin
        bin = Ms[bin];
        int x = bin % N;
        int y = bin / N;
        // uniform sampling inside the bin
        float u = (x + prng.in01()) / N;
        float v = (y + prng.in01()) / N;
        return invm(vec2(u, v));
    }
};

class EnvironmentMapEquiRect final : public EnvironmentMap
{
public:
    enum Compatibility {
        CompatibilityMitsuba,
        CompatibilitySurroundVideo
    };

private:
    const Compatibility _compatibility;
    const Texture* _tex;

public:
    EnvironmentMapEquiRect(const Texture* tex, Compatibility compat = CompatibilityMitsuba) :
        _compatibility(compat), _tex(tex)
    {
    }

    virtual vec4 L(const vec3& direction, float t = 0.0f) const override
    {
        float y = asin(clamp(direction.y(), -1.0f, 1.0f));
        float x = atan(-direction.x(), direction.z());
        switch (_compatibility) {
        case CompatibilityMitsuba:
            x -= pi;
            if (x < 0.0f)
                x += 2.0f * pi;
            break;
        case CompatibilitySurroundVideo:
            break;
        }
        x *= 0.5f * inv_pi;
        y = y * inv_pi + 0.5f;
        return _tex->value(vec2(x, y), t);
    }
};

class EnvironmentMapCube final : public EnvironmentMap
{
private:
    const Texture* _cubesides[6]; // posx negx posy negy posz negz

public:
    EnvironmentMapCube(
            const Texture* posx, const Texture* negx,
            const Texture* posy, const Texture* negy,
            const Texture* posz, const Texture* negz) :
        _cubesides { posx, negx, posy, negy, posz, negz }
    {
    }

    virtual vec4 L(const vec3& direction, float t = 0.0f) const override
    {
        float ax = abs(direction.x());
        float ay = abs(direction.y());
        float az = abs(direction.z());
        int cubeside;
        float u, v;
        if (ax > ay && ax > az) {
            u = 0.5f * (direction.z() / -direction.x() + 1.0f);
            v = 0.5f * (direction.y() / ax + 1.0f);
            cubeside = 0 + signbit(direction.x());
        } else if (ay > az) {
            u = 0.5f * (direction.x() / ay + 1.0f);
            v = 0.5f * (direction.z() / -direction.y() + 1.0f);
            cubeside = 2 + signbit(direction.y());
        } else {
            u = 0.5f * (direction.x() / direction.z() + 1.0f);
            v = 0.5f * (direction.y() / az + 1.0f);
            cubeside = 4 + signbit(direction.z());
        }
        return _cubesides[cubeside]->value(vec2(u, v), t);
    }
};

}
