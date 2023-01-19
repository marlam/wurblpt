/*
 * Copyright (C) 2020, 2021, 2022
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

#include <string>
#include <map>
#include <filesystem>
#include <fstream>
#include <vector>


namespace WurblPT {

class Hitable;
class AnimationCache;

class SceneComponent
{
public:
    virtual ~SceneComponent()
    {
    }

    /*!\brief Called by \a Scene to create the Hitables for rendering from this scene component */
    virtual std::vector<Hitable*> createHitables() const
    {
        return std::vector<Hitable*>();
    }

    /*!\brief Called by \a Scene and by \a exportToObj() to export this scene component as part
     * of a larger collection of components. */
    virtual std::string exportToObj(
            std::ostream& /* geometryOut */, std::ostream& /* materialOut */,
            unsigned int& /* globalVertexIndex */,
            bool /* sceneExportTopLevel */,
            AnimationCache& /* animationCache */,
            const std::filesystem::path& /* basePath */, const std::string& /* baseName */,
            std::map<const SceneComponent*, std::string>& /* sceneExportCache */) const
    {
        return std::string();
    }

    /*!\brief Export this scene component to OBJ format. Only give the base name of the file,
     * the extension will be determined as necessary (.obj for geometry, .mtl for materials,
     * .png or .exr for textures). */
    bool exportToObj(const std::string& baseFileName, AnimationCache& animationCache) const
    {
        std::filesystem::path path(baseFileName);
        std::filesystem::path basePath = path.parent_path();
        std::string baseName = path.filename().string();

        std::ofstream geometryOut(basePath / (baseName + ".obj"), std::ios::out | std::ios::trunc);
        std::ofstream materialOut(basePath / (baseName + ".mtl"), std::ios::out | std::ios::trunc);

        geometryOut << "mtllib " << baseName << ".mtl" << std::endl;
        std::streampos gpos = geometryOut.tellp();

        unsigned int globalVertexIndex = 1;
        std::map<const SceneComponent*, std::string> sceneExportCache;
        exportToObj(geometryOut, materialOut, globalVertexIndex, false, animationCache, basePath, baseName, sceneExportCache);

        bool geometryWritten = (geometryOut.tellp() > gpos);
        bool materialWritten = (materialOut.tellp() > 0);

        geometryOut.flush();
        materialOut.flush();
        bool success = geometryOut.good() && materialOut.good();
        geometryOut.close();
        materialOut.close();

        if (!geometryWritten)
            std::filesystem::remove(basePath / (baseName + ".obj"));
        if (!materialWritten)
            std::filesystem::remove(basePath / (baseName + ".mtl"));

        return success;
    }
};

}
