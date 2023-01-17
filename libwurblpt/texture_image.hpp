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

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include "texture.hpp"
#include "color.hpp"


namespace WurblPT {

template<int IMG_COMPONENT_COUNT, TGD::Type DATA_TYPE, bool LINEARIZE_SRGB>
    requires (IMG_COMPONENT_COUNT == 1 || IMG_COMPONENT_COUNT == 2 || IMG_COMPONENT_COUNT == 3 || IMG_COMPONENT_COUNT == 4)
        && (DATA_TYPE == TGD::uint8 || DATA_TYPE == TGD::uint16 || DATA_TYPE == TGD::float32)
class TextureImage final : public Texture
{
private:
    const TGD::ArrayContainer _img;
    const vec2 _coordFactor;
    const vec2 _coordOffset;
    const vec4 _valFactor;
    const vec4 _valOffset;

    static float from_uint8(uint8_t v)
    {
        return byte_to_float(v);
    }

    static float from_uint8_srgb(uint8_t v)
    {
        return srgb_to_rgb_helper(byte_to_float(v));
    }

    static float from_uint16(uint16_t v)
    {
        return uint16_to_float(v);
    }

    static float from_uint16_srgb(uint16_t v)
    {
        return srgb_to_rgb_helper(uint16_to_float(v));
    }

public:
    TextureImage(const TGD::ArrayContainer& img,
            const vec2& coordFactor = vec2(1.0f), const vec2& coordOffset = vec2(0.0f),
            const vec4& valFactor = vec4(1.0f), const vec4& valOffset = vec4(0.0f)) :
        _img(img),
        _coordFactor(coordFactor), _coordOffset(coordOffset),
        _valFactor(valFactor), _valOffset(valOffset)
    {
        assert(img.componentCount() == IMG_COMPONENT_COUNT);
        assert(img.dimensionCount() == 2);
        assert(img.elementCount() > 0);
        assert(img.componentType() == DATA_TYPE);
    }

    vec4 value(size_t x, size_t y) const
    {
        if (x >= _img.dimension(0))
            x = _img.dimension(0) - 1;
        if (y >= _img.dimension(1))
            y = _img.dimension(1) - 1;

        vec4 color;
        if (DATA_TYPE == TGD::uint8) {
            const uint8_t* data = _img.get<uint8_t>({ x, y });
            if (IMG_COMPONENT_COUNT == 3) {
                if (LINEARIZE_SRGB)
                    color = vec4(from_uint8_srgb(data[0]), from_uint8_srgb(data[1]), from_uint8_srgb(data[2]), 1.0f);
                else
                    color = vec4(from_uint8(data[0]), from_uint8(data[1]), from_uint8(data[2]), 1.0f);
            } else if (IMG_COMPONENT_COUNT == 4) {
                if (LINEARIZE_SRGB)
                    color = vec4(from_uint8_srgb(data[0]), from_uint8_srgb(data[1]), from_uint8_srgb(data[2]), from_uint8(data[3]));
                else
                    color = vec4(from_uint8(data[0]), from_uint8(data[1]), from_uint8(data[2]), from_uint8(data[3]));
            } else if (IMG_COMPONENT_COUNT == 1) {
                float tmp;
                if (LINEARIZE_SRGB)
                    tmp = from_uint8_srgb(data[0]);
                else
                    tmp = from_uint8(data[0]);
                color = vec4(tmp, tmp, tmp, 1.0f);
            } else {
                float tmp;
                if (LINEARIZE_SRGB)
                    tmp = from_uint8_srgb(data[0]);
                else
                    tmp = from_uint8(data[0]);
                color = vec4(tmp, tmp, tmp, from_uint8(data[1]));
            }
            assert(max(color) <= 1.0f);
            assert(min(color) >= 0.0f);
        } else if (DATA_TYPE == TGD::uint16) {
            const uint16_t* data = _img.get<uint16_t>({ x, y });
            if (IMG_COMPONENT_COUNT == 3) {
                if (LINEARIZE_SRGB)
                    color = vec4(from_uint16_srgb(data[0]), from_uint16_srgb(data[1]), from_uint16_srgb(data[2]), 1.0f);
                else
                    color = vec4(from_uint16(data[0]), from_uint16(data[1]), from_uint16(data[2]), 1.0f);
            } else if (IMG_COMPONENT_COUNT == 4) {
                if (LINEARIZE_SRGB)
                    color = vec4(from_uint16_srgb(data[0]), from_uint16_srgb(data[1]), from_uint16_srgb(data[2]), from_uint16(data[3]));
                else
                    color = vec4(from_uint16(data[0]), from_uint16(data[1]), from_uint16(data[2]), from_uint16(data[3]));
            } else if (IMG_COMPONENT_COUNT == 1) {
                float tmp;
                if (LINEARIZE_SRGB)
                    tmp = from_uint16_srgb(data[0]);
                else
                    tmp = from_uint16(data[0]);
                color = vec4(tmp, tmp, tmp, 1.0f);
            } else {
                float tmp;
                if (LINEARIZE_SRGB)
                    tmp = from_uint16_srgb(data[0]);
                else
                    tmp = from_uint16(data[0]);
                color = vec4(tmp, tmp, tmp, from_uint16(data[1]));
            }
            assert(max(color) <= 1.0f);
            assert(min(color) >= 0.0f);
        } else {
            const float* data = _img.get<float>({ x, y });
            if (_img.componentCount() == 3) {
                if (LINEARIZE_SRGB)
                    color = vec4(srgb_to_rgb_helper(data[0]), srgb_to_rgb_helper(data[1]), srgb_to_rgb_helper(data[2]), 1.0f);
                else
                    color = vec4(data[0], data[1], data[2], 1.0f);
            } else if (_img.componentCount() == 4) {
                if (LINEARIZE_SRGB)
                    color = vec4(srgb_to_rgb_helper(data[0]), srgb_to_rgb_helper(data[1]), srgb_to_rgb_helper(data[2]), data[3]);
                else
                    color = vec4(data[0], data[1], data[2], data[3]);
            } else if (_img.componentCount() == 1) {
                float tmp;
                if (LINEARIZE_SRGB)
                    tmp = srgb_to_rgb_helper(data[0]);
                else
                    tmp = data[0];
                color = vec4(tmp, tmp, tmp, 1.0f);
            } else {
                float tmp;
                if (LINEARIZE_SRGB)
                    tmp = srgb_to_rgb_helper(data[0]);
                else
                    tmp = data[0];
                color = vec4(tmp, tmp, tmp, data[1]);
            }
        }
        return color;
    }

    virtual vec4 value(const vec2& texcoords, float /* t */) const override
    {
        assert(all(isfinite(texcoords)));
        vec2 uv = fract(_coordFactor * texcoords + _coordOffset);
#if 0
        // nearest neighbor interpolation
        size_t x = uv.s() * _img.dimension(0);
        size_t y = uv.t() * _img.dimension(1);
        vec4 val = value(x, y);
#else
        // bilinear interpolation
        float uvs = max(0.0f, (uv.s() * _img.dimension(0)) - 0.5f);
        float uvt = max(0.0f, (uv.t() * _img.dimension(1)) - 0.5f);
        size_t x0 = uvs;
        size_t y0 = uvt;
        size_t x1 = x0 + 1; // value() will handle overflow
        size_t y1 = y0 + 1; // value() will handle overflow
        float alpha = uvs - x0;
        assert(alpha >= 0.0f && alpha <= 1.0f);
        float beta = uvt - y0;
        assert(beta >= 0.0f && beta <= 1.0f);
        vec4 v00 = value(x0, y0);
        vec4 v10 = value(x1, y0);
        vec4 v01 = value(x0, y1);
        vec4 v11 = value(x1, y1);
        vec4 a = mix(v00, v10, alpha);
        vec4 b = mix(v01, v11, alpha);
        vec4 val = mix(a, b, beta);
#endif
        return _valFactor * val + _valOffset;
    }

    virtual vec2 texelSize() const override
    {
        return vec2(1.0f / _img.dimension(0), 1.0f / _img.dimension(1));
    }

    virtual unsigned int componentCount() const override
    {
        return IMG_COMPONENT_COUNT;
    }

    virtual TGD::Type componentType() const override
    {
        return DATA_TYPE;
    }

    virtual enum LinearizeSRGBType linearizeSRGBType() const override
    {
        return LINEARIZE_SRGB ? LinearizeSRGB_On : LinearizeSRGB_Off;
    }
};

inline Texture* createTextureImage(const TGD::ArrayContainer& img,
        LinearizeSRGBType linearizeSRGBType = LinearizeSRGB_Auto,
        const vec2& coordFactor = vec2(1.0f),
        const vec2& coordOffset = vec2(0.0f),
        const vec4& valFactor = vec4(1.0f),
        const vec4& valOffset = vec4(0.0f))
{
    assert(img.dimensionCount() == 2);
    assert(img.dimension(0) > 0);
    assert(img.dimension(1) > 0);
    assert(img.componentCount() >= 1);
    assert(img.componentCount() <= 4);
    assert(img.componentType() == TGD::uint8
            || img.componentType() == TGD::uint16
            || img.componentType() == TGD::float32);

    bool linearizeSRGB = false;
    switch (linearizeSRGBType) {
    case LinearizeSRGB_On:
        linearizeSRGB = true;
        break;
    case LinearizeSRGB_Off:
        linearizeSRGB = false;
        break;
    case LinearizeSRGB_Auto:
        linearizeSRGB = (img.componentType() == TGD::uint8 || img.componentType() == TGD::uint16);
        break;
    }

    TGD::Type type = img.componentType();
    Texture* tex = nullptr;
    switch (img.componentCount()) {
    case 1:
        if (type == TGD::uint8) {
            if (linearizeSRGB)
                tex = new TextureImage<1, TGD::uint8, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<1, TGD::uint8, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else if (type == TGD::uint16) {
            if (linearizeSRGB)
                tex = new TextureImage<1, TGD::uint16, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<1, TGD::uint16, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else {
            if (linearizeSRGB)
                tex = new TextureImage<1, TGD::float32, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<1, TGD::float32, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        }
        break;
    case 2:
        if (type == TGD::uint8) {
            if (linearizeSRGB)
                tex = new TextureImage<2, TGD::uint8, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<2, TGD::uint8, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else if (type == TGD::uint16) {
            if (linearizeSRGB)
                tex = new TextureImage<2, TGD::uint16, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<2, TGD::uint16, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else {
            if (linearizeSRGB)
                tex = new TextureImage<2, TGD::float32, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<2, TGD::float32, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        }
        break;
    case 3:
        if (type == TGD::uint8) {
            if (linearizeSRGB)
                tex = new TextureImage<3, TGD::uint8, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<3, TGD::uint8, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else if (type == TGD::uint16) {
            if (linearizeSRGB)
                tex = new TextureImage<3, TGD::uint16, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<3, TGD::uint16, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else {
            if (linearizeSRGB)
                tex = new TextureImage<3, TGD::float32, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<3, TGD::float32, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        }
        break;
    case 4:
        if (type == TGD::uint8) {
            if (linearizeSRGB)
                tex = new TextureImage<4, TGD::uint8, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<4, TGD::uint8, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else if (type == TGD::uint16) {
            if (linearizeSRGB)
                tex = new TextureImage<4, TGD::uint16, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<4, TGD::uint16, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        } else {
            if (linearizeSRGB)
                tex = new TextureImage<4, TGD::float32, true>(img, coordFactor, coordOffset, valFactor, valOffset);
            else
                tex = new TextureImage<4, TGD::float32, false>(img, coordFactor, coordOffset, valFactor, valOffset);
        }
        break;
    }
    return tex;
}

inline Texture* createTextureImage(const std::string& imgFileName,
        LinearizeSRGBType linearizeSRGBType = LinearizeSRGB_Auto,
        const vec2& coordFactor = vec2(1.0f),
        const vec2& coordOffset = vec2(0.0f),
        const vec4& valFactor = vec4(1.0f),
        const vec4& valOffset = vec4(0.0f))
{
    TGD::Error e;
    TGD::ArrayContainer img = TGD::load(imgFileName, TGD::TagList(), &e);
    if (img.elementCount() == 0) {
        fprintf(stderr, "%s: %s\n", imgFileName.c_str(), TGD::strerror(e));
        return nullptr;
    } else if (img.dimensionCount() != 2
            || img.dimension(0) == 0 || img.dimension(1) == 0
            || img.componentCount() < 1 || img.componentCount() > 4
            || (img.componentType() != TGD::uint8
                && img.componentType() != TGD::uint16
                && img.componentType() != TGD::float32)) {
        fprintf(stderr, "%s: not a valid texture image\n", imgFileName.c_str());
        return nullptr;
    } else {
        return createTextureImage(img, linearizeSRGBType, coordFactor, coordOffset, valFactor, valOffset);
    }
}

}
