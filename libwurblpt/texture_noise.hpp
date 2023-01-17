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

#include <vector>

#include "texture.hpp"
#include "prng.hpp"


namespace WurblPT {

class TextureValueNoise : public Texture
{
private:
    int _width, _height;
    std::vector<float> _values;

    float value(int x, int y) const
    {
        x = x % _width;
        y = y % _height;
        return _values[y * _width + x];
    }

public:
    TextureValueNoise(int w, int h, Prng& prng) :
        _width(w), _height(h), _values(w * h)
    {
        for (int i = 0; i < w * h; i++)
            _values[i] = prng.in01() * 2.0f - 1.0f;
    }

    virtual vec4 value(const vec2& texcoord, float /* t */) const override
    {
        int ix = floor(_width * texcoord.x());
        int iy = floor(_height * texcoord.y());
        float fx = fract(_width * texcoord.x());
        float fy = fract(_height * texcoord.y());
        float a = mix(value(ix + 0, iy + 0), value(ix + 1, iy + 0), fx);
        float b = mix(value(ix + 0, iy + 1), value(ix + 1, iy + 1), fx);
        float c = mix(a, b, fy);
        return vec4(c);
    }
};

class TextureGradientNoise : public Texture
{
private:
    int _width, _height;
    std::vector<vec2> _values;

    vec2 value(int x, int y) const
    {
        x = x % _width;
        y = y % _height;
        return _values[y * _width + x];
    }

public:
    TextureGradientNoise(int w, int h, Prng& prng) :
        _width(w), _height(h), _values(w * h)
    {
        for (int i = 0; i < w * h; i++)
            _values[i] = Sampler::onUnitDisk(prng.in01());
    }

    virtual vec4 value(const vec2& texcoord, float /* t */) const override
    {
        int ix = floor(_width * texcoord.x());
        int iy = floor(_height * texcoord.y());
        float fx = fract(_width * texcoord.x());
        float fy = fract(_height * texcoord.y());
        float sx = fx * fx * (3.0f - 2.0f * fx); // smoothstep function
        float sy = fy * fy * (3.0f - 2.0f * fy); // smoothstep function
        float a = mix(dot(value(ix + 0, iy + 0), vec2(fx - 0.0f, fy - 0.0f)),
                dot(value(ix + 1, iy + 0), vec2(fx - 1.0f, fy - 0.0f)), sx);
        float b = mix(dot(value(ix + 0, iy + 1), vec2(fx - 0.0f, fy - 1.0f)),
                dot(value(ix + 1, iy + 1), vec2(fx - 1.0f, fy - 1.0f)), sx);
        float c = mix(a, b, sy);
        return vec4(c);
    }
};

// The value() function returns the four nearest distances d1,d2,d3,d4.
class TextureWorleyNoise : public Texture
{
private:
    std::vector<vec2> _points;

    static float toroidalDistance(const vec2& p, const vec2& q)
    {
        // see https://blog.demofox.org/2017/10/01/calculating-the-distance-between-points-in-wrap-around-toroidal-space/
        float dx = abs(q.x() - p.x());
        float dy = abs(q.y() - p.y());
        if (dx > 0.5f)
            dx = 1.0f - dx;
        if (dy > 0.5f)
            dy = 1.0f - dy;
        return sqrt(dx * dx + dy * dy);
    }

public:
    TextureWorleyNoise(int n, Prng& prng) : _points(n)
    {
        assert(n >= 4);
        for (int i = 0; i < n; i++)
            _points[i] = prng.in01x2();
    }

    virtual vec4 value(const vec2& texcoord, float /* t */) const override
    {
        vec2 uv = fract(texcoord);
        float d1 = maxval;
        float d2 = maxval;
        float d3 = maxval;
        float d4 = maxval;
        for (size_t i = 0; i < _points.size(); i++) {
            float d = toroidalDistance(uv, _points[i]);
            if (d < d1) {
                d4 = d3;
                d3 = d2;
                d2 = d1;
                d1 = d;
            } else if (d < d2) {
                d4 = d3;
                d3 = d2;
                d2 = d;
            } else if (d < d3) {
                d4 = d3;
                d3 = d;
            } else if (d < d4) {
                d4 = d;
            }
        }
        return vec4(d1, d2, d3, d4);
    }
};

// This Perlin Noise texture is based on the Ray Tracing book series by Peter Shirley.
class TexturePerlinNoise : public Texture
{
private:
    int _size;
    std::vector<vec3> _randomVectors;
    std::vector<int> _perm_x, _perm_y, _perm_z;

public:
    TexturePerlinNoise(Prng& prng) :
        _size(256), _randomVectors(_size), _perm_x(_size), _perm_y(_size), _perm_z(_size)
    {
        for (int i = 0; i < _size; i++) {
            _randomVectors[i] = Sampler::onUnitSphere(prng.in01x2());
            _perm_x[i] = i;
            _perm_y[i] = i;
            _perm_z[i] = i;
        }
        for (int i = _size - 1; i > 0; i--) {
            int s = prng.in01() * (i + 1);
            std::swap(_perm_x[i], _perm_x[s]);
            int t = prng.in01() * (i + 1);
            std::swap(_perm_y[i], _perm_y[t]);
            int u = prng.in01() * (i + 1);
            std::swap(_perm_z[i], _perm_z[u]);
        }
    }

    virtual vec4 value(const vec2& texcoords, float /* t */) const override
    {
        float x = fract(texcoords.x()) * _size;
        float y = fract(texcoords.y()) * _size;
        float z = 0.5f * _size;

        float u = fract(x);
        float v = fract(y);
        float w = fract(z);

        int i = x;
        int j = y;
        int k = z;

        vec3 c[2][2][2];
        for (int di = 0; di < 2; di++) {
            for (int dj = 0; dj < 2; dj++) {
                for (int dk = 0; dk < 2; dk++) {
                    c[di][dj][dk] = _randomVectors[
                        _perm_x[(i + di) & (_size - 1)]
                            ^ _perm_y[(j + dj) & (_size - 1)]
                            ^ _perm_z[(k + dk) & (_size - 1)]];
                }
            }
        }

        float uu = u * u * (3.0f - 2.0f * u);
        float vv = v * v * (3.0f - 2.0f * v);
        float ww = w * w * (3.0f - 2.0f * w);
        float value = 0.0f;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    vec3 weight(u - i, v - j, w - k);
                    value +=
                        (i * uu + (1.0f - i) * (1.0f - uu))
                        * (j * vv + (1.0f - j) * (1.0f - vv))
                        * (k * ww + (1.0f - k) * (1.0f - ww))
                        * dot(c[i][j][k], weight);
                }
            }
        }

        return vec4(value, value, value, 1.0f);
    }

    vec4 turbulence(const vec2& texcoord, float t, int freqs = 7, float minFreq = 1.0f) const
    {
        float v = 0.0f;
        float a = 1.0f;
        float f = minFreq;
        for (int i = 0; i < freqs; i++) {
            v += a * TexturePerlinNoise::value(texcoord * f, t).r();
            f *= 2.0;
            a /= 2.0;
        }
        return abs(vec4(v, v, v, 0.0f));
    }
};

}
