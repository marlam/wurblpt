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

#include <cstdio>

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include "scene_component.hpp"
#include "color.hpp"
#include "animation.hpp"


namespace WurblPT {

enum LinearizeSRGBType
{
    LinearizeSRGB_On,
    LinearizeSRGB_Off,
    LinearizeSRGB_Auto
};

class Texture : public SceneComponent
{
public:
    virtual ~Texture()
    {
    }

    virtual vec4 value(const vec2& texcoords, float t = 0.0f) const = 0;

    virtual vec2 texelSize() const
    {
        return vec2(0.0f, 0.0f);
    }

    virtual unsigned int componentCount() const
    {
        return 4;
    }

    virtual TGD::Type componentType() const
    {
        return TGD::float32;
    }

    virtual enum LinearizeSRGBType linearizeSRGBType() const
    {
        return LinearizeSRGB_Auto;
    }

    virtual std::string exportToObj(
            std::ostream& /* geometryOut */, std::ostream& /* materialOut */,
            unsigned int& /* globalVertexIndex */,
            bool sceneExportTopLevel,
            AnimationCache& animationCache,
            const std::filesystem::path& basePath, const std::string& baseName,
            std::map<const SceneComponent*, std::string>& sceneExportCache) const override
    {
        if (sceneExportTopLevel)
            return std::string();

        auto it = sceneExportCache.find(this);
        if (it != sceneExportCache.end())
            return it->second;

        char hex[17];
        std::snprintf(hex, sizeof(hex), "%p", this); // TODO use std::format when it becomes available
        std::string myName = baseName + "_tex_" + hex;
        if (componentType() == TGD::float32)
            myName += ".exr";
        else
            myName += ".png";
        std::string fileName = std::filesystem::path(basePath / myName).string();

        unsigned int width = 512;
        unsigned int height = 512;
        if (texelSize().x() > 0.0f)
            width = max(static_cast<unsigned int>(1.0f / texelSize().x()), 1u);
        if (texelSize().y() > 0.0f)
            height = max(static_cast<unsigned int>(1.0f / texelSize().y()), 1u);
        bool unlinearizeSRGB = (linearizeSRGBType() == LinearizeSRGB_On)
            || (linearizeSRGBType() == LinearizeSRGB_Auto && (componentType() == TGD::uint8 || componentType() == TGD::uint16));

        if (componentType() == TGD::uint8) {
            TGD::Array<uint8_t> img({width, height}, componentCount());
            for (unsigned int y = 0; y < height; y++) {
                for (unsigned int x = 0; x < width; x++) {
                    vec2 tc = vec2((x + 0.5f) / width, (y + 0.5f) / height);
                    vec4 v = value(tc, animationCache.t());
                    for (unsigned int c = 0; c < componentCount(); c++) {
                        if (c < 3 && unlinearizeSRGB)
                            v[c] = rgb_to_srgb_helper(v[c]);
                        uint8_t vv = float_to_byte(v[c]);
                        img[{x, y}][c] = vv;
                    }
                }
            }
            TGD::save(img, fileName);
        } else if (componentType() == TGD::uint16) {
            TGD::Array<uint16_t> img({width, height}, componentCount());
            for (unsigned int y = 0; y < height; y++) {
                for (unsigned int x = 0; x < width; x++) {
                    vec2 tc = vec2((x + 0.5f) / width, (y + 0.5f) / height);
                    vec4 v = value(tc, animationCache.t());
                    for (unsigned int c = 0; c < componentCount(); c++) {
                        if (c < 3 && unlinearizeSRGB)
                            v[c] = rgb_to_srgb_helper(v[c]);
                        uint16_t vv = float_to_uint16(v[c]);
                        img[{x, y}][c] = vv;
                    }
                }
            }
            TGD::save(img, fileName);
        } else if (componentType() == TGD::float32) {
            TGD::Array<float> img({width, height}, componentCount());
            for (unsigned int y = 0; y < height; y++) {
                for (unsigned int x = 0; x < width; x++) {
                    vec2 tc = vec2((x + 0.5f) / width, (y + 0.5f) / height);
                    vec4 v = value(tc, animationCache.t());
                    for (unsigned int c = 0; c < componentCount(); c++) {
                        if (c < 3 && unlinearizeSRGB)
                            v[c] = rgb_to_srgb_helper(v[c]);
                        img[{x, y}][c] = v[c];
                    }
                }
            }
            TGD::save(img, fileName);
        }

        sceneExportCache.insert(std::pair<const SceneComponent*, std::string>(this, myName));
        return myName;
    }
};

class TextureConstant final : public Texture
{
private:
    const vec4 _color;

public:
    TextureConstant(const vec4& color) :
        _color(color)
    {
    }

    virtual vec4 value(const vec2& /* texcoords */, float /* t */) const override
    {
        return _color;
    }

    virtual vec2 texelSize() const override
    {
        return vec2(1.0f, 1.0f);
    }
};

class TextureChecker final : public Texture
{
private:
    vec4 _color0, _color1;
    int _horiz, _vert;

public:
    TextureChecker(const vec4& color0, const vec4& color1, int horiz = 9, int vert = 9) :
        _color0(color0), _color1(color1), _horiz(horiz), _vert(vert)
    {
    }

    TextureChecker(const vec3& color0, const vec3& color1, int horiz = 9, int vert = 9) :
        TextureChecker(vec4(color0, average(color0)), vec4(color1, average(color1)), horiz, vert)
    {
    }

    virtual vec4 value(const vec2& texcoords, float /* t */) const override
    {
        int row = texcoords.y() * _vert;
        int col = texcoords.x() * _horiz;
        return (row % 2 == col % 2 ? _color0 : _color1);
    }
};

class TextureTransformer final : public Texture
{
private:
    const Texture* _texture;
    const vec2 _coordFactor;
    const vec2 _coordOffset;
    const vec4 _valFactor;
    const vec4 _valOffset;

public:
    TextureTransformer(const Texture* texture,
            const vec2& coordFactor, const vec2& coordOffset = vec2(0.0f),
            const vec4& valFactor = vec4(1.0f), const vec4& valOffset = vec4(0.0f)) :
        _texture(texture),
        _coordFactor(coordFactor), _coordOffset(coordOffset),
        _valFactor(valFactor), _valOffset(valOffset)
    {
    }

    virtual vec4 value(const vec2& texcoords, float t) const override
    {
        vec4 val = _texture->value(_coordFactor * texcoords + _coordOffset, t);
        return _valFactor * val + _valOffset;
    }

    virtual vec2 texelSize() const override
    {
        return _texture->texelSize() / _coordFactor;
    }

    virtual unsigned int componentCount() const override
    {
        return _texture->componentCount();
    }

    virtual TGD::Type componentType() const override
    {
        return _texture->componentType();
    }
};

}
