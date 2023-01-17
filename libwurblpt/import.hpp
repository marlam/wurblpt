/*
 * Copyright (C) 2017, 2018, 2019, 2020, 2021, 2022
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
#include <string>
#include <map>
#include <tuple>
#include <set>

#include <tgd/io.hpp>

#include "tiny_obj_loader.h"

#include "scene.hpp"
#include "transformation.hpp"
#include "texture_image.hpp"
#include "material_lambertian.hpp"
#include "material_glass.hpp"
#include "material_modphong.hpp"
#include "geometryproc.hpp"
#include "mesh.hpp"
#include "color.hpp"

#ifdef _WIN32
# define WURBLPT_DIR_SEP "\\"
# define WURBLPT_NOT_DIR_SEP "/"
# define WURBLPT_DIR_SEP_CHAR '\\'
# define WURBLPT_NOT_DIR_SEP_CHAR '/'
#else
# define WURBLPT_DIR_SEP "/"
# define WURBLPT_NOT_DIR_SEP "\\"
# define WURBLPT_DIR_SEP_CHAR '/'
# define WURBLPT_NOT_DIR_SEP_CHAR '\\'
#endif


namespace WurblPT {

/* Helper function to create a normal map from a bump map */
inline TGD::ArrayContainer toNormalMap(const TGD::ArrayContainer& bumpMap, float bumpScaling = 8.0f)
{
    assert(bumpMap.componentType() == TGD::uint8);

    TGD::Array<uint8_t> normalMap(bumpMap.dimensions(), 3);
    int w = normalMap.dimension(0);
    int h = normalMap.dimension(1);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t height_r = static_cast<const uint8_t*>(bumpMap.get(y * w + std::min(x + 1, w - 1)))[0];
            uint8_t height_l = static_cast<const uint8_t*>(bumpMap.get(y * w + std::max(x - 1,     0)))[0];
            uint8_t height_t = static_cast<const uint8_t*>(bumpMap.get(std::min(y + 1, h - 1) * w + x))[0];
            uint8_t height_b = static_cast<const uint8_t*>(bumpMap.get(std::max(y - 1,     0) * w + x))[0];

            vec3 tx = vec3(2.0, 0.0, bumpScaling * (byte_to_float(height_r) - byte_to_float(height_l)));
            vec3 ty = vec3(0.0, 2.0, bumpScaling * (byte_to_float(height_t) - byte_to_float(height_b)));
            vec3 n = normalize(cross(tx, ty));

            uint8_t* rgb = normalMap.get<uint8_t>(y * w + x);
            rgb[0] = float_to_byte(0.5f * (n.x() + 1.0f));
            rgb[1] = float_to_byte(0.5f * (n.y() + 1.0f));
            rgb[2] = float_to_byte(0.5f * (n.z() + 1.0f));
        }
    }

    return normalMap;
}

/* Helper function to import a texture image */
inline Texture* importTexture(
        std::map<std::string, Texture*>& textureMap,
        const std::map<std::string, TGD::ArrayContainer>& textureFileMap,
        const std::string& name,
        const float factor[3],
        const float offset[3],
        int* componentCount = nullptr,
        LinearizeSRGBType linearizeSRGBType = LinearizeSRGB_Auto,
        float bumpScaling = -1.0f /* < 0 means this is no bump map */)
{
    std::string linearizeSRGBStr;
    switch (linearizeSRGBType) {
    case LinearizeSRGB_On:
        linearizeSRGBStr = "on";
        break;
    case LinearizeSRGB_Off:
        linearizeSRGBStr = "off";
        break;
    case LinearizeSRGB_Auto:
        linearizeSRGBStr = "auto";
        break;
    }
    std::string cacheName = name
        + std::string("_factor=") + std::to_string(factor[0]) + ',' + std::to_string(factor[1])
        + std::string("_offset=") + std::to_string(offset[0]) + ',' + std::to_string(offset[1])
        + std::string("_linsrgb=") + linearizeSRGBStr
        + std::string("_bumpscal=") + std::to_string(bumpScaling);
    auto it = textureMap.find(cacheName);
    if (it != textureMap.end()) {
        fprintf(stderr, "    texture %s: found in cache\n", cacheName.c_str());
        return it->second;
    } else {
        fprintf(stderr, "    texture %s: creating\n", cacheName.c_str());
    }
    auto fit = textureFileMap.find(name);
    assert(fit != textureFileMap.end());
    TGD::ArrayContainer img = fit->second;
    Texture* tex;
    if (img.elementCount() == 0) {
        tex = new TextureConstant(vec4(0.5f));
        fprintf(stderr, "    texture %s: replaced with dummy texture\n", cacheName.c_str());
    } else {
        if (componentCount)
            *componentCount = img.componentCount();
        if (bumpScaling > 0.0f) {
            if (img.componentType() != TGD::uint8) {
                fprintf(stderr, "    texture %s: cannot handle bump/normal that is not uint8\n", cacheName.c_str());
                tex = new TextureConstant(vec4(0.5f, 0.5f, 1.0f, 0.0f));
                fprintf(stderr, "    texture %s: replaced with dummy texture\n", cacheName.c_str());
            } else {
                // check that this really is a bump map and not already a normal map
                bool imageIsBumpMap = true;
                if (img.componentCount() >= 3) {
                    // sometimes gray level bump maps are stored as RGB images (for example
                    // in the San Miguel scene from McGuire's graphics data), so check that here
                    for (size_t e = 0; e < img.elementCount(); e++) {
                        const uint8_t* elem = img.get<uint8_t>(e);
                        if (elem[0] != elem[1] || elem[0] != elem[2] || elem[1] != elem[2]) {
                            imageIsBumpMap = false;
                            break;
                        }
                    }
                }
                if (imageIsBumpMap) {
                    // bump map: convert to normal map first
                    img = toNormalMap(img, bumpScaling);
                }
                tex = createTextureImage(img, linearizeSRGBType,
                        vec2(factor[0], factor[1]), vec2(offset[0], offset[1]));
            }
        } else {
            tex = createTextureImage(img, linearizeSRGBType,
                    vec2(factor[0], factor[1]), vec2(offset[0], offset[1]));
        }
    }
    textureMap.insert(std::pair<std::string, Texture*>(cacheName, tex));
    return tex;
}

/* Helper function to split a multiline TinyObjLoader message */
inline std::vector<std::string> tinyObjMsgToLines(const std::string& s)
{
    std::vector<std::string> lines;
    size_t i = 0;
    for (;;) {
        size_t j = s.find_first_of('\n', i);
        if (j < std::string::npos) {
            lines.push_back(s.substr(i, j - i));
            i = j + 1;
        } else {
            break;
        }
    }
    return lines;
}

/*! \brief Flag: Disable light sources. */
constexpr unsigned int ImportBitDisableLightSources = (1 << 0);
/*! \brief Flag: Do not mark light emitting materials as hot spots. */
constexpr unsigned int ImportBitDisableHotSpots     = (1 << 1);
/*! \brief Flag: Import all materials as two-sided, e.g. for San Miguel. */
constexpr unsigned int ImportBitTwoSidedMaterials   = (1 << 2);
/*! \brief Flag: Tf=0 (or d=1) means opaque, e.g. for San Miguel. */
constexpr unsigned int ImportBitInvertedTf          = (1 << 3);
/*! \brief Flag: Use glass material for transparency instead of ModPhong. */
constexpr unsigned int ImportBitWithGlass           = (1 << 4);

/*! \brief Import a triangle-based scene from an OBJ file.
 * \param scene             The scene to import into
 * \param filename          Name of the OBJ file
 * \param transformation    Optional transformation to be applied
 * \param flags             Combination of the importer flags.
 */
inline bool importIntoScene(Scene& scene, const std::string& filename,
        const Transformation& transformation = Transformation(),
        unsigned int importBits = 0)
{
    fprintf(stderr, "%s: importing...\n", filename.c_str());
    tinyobj::ObjReaderConfig conf;
    conf.triangulate = true;
    conf.vertex_color = false;
    tinyobj::ObjReader reader;
    reader.ParseFromFile(filename, conf);
    if (reader.Warning().size() > 0) {
        std::vector<std::string> lines = tinyObjMsgToLines(reader.Warning());
        for (size_t i = 0; i < lines.size(); i++)
            fprintf(stderr, "  warning: %s\n", lines[i].c_str());
    }
    if (!reader.Valid()) {
        if (reader.Error().size() > 0) {
            std::vector<std::string> lines = tinyObjMsgToLines(reader.Error());
            for (size_t i = 0; i < lines.size(); i++)
                fprintf(stderr, "  error: %s\n", lines[i].c_str());
        } else {
            fprintf(stderr, "  unknown error\n");
        }
        fprintf(stderr, "%s: import failure\n", filename.c_str());
        return false;
    }
    const std::vector<tinyobj::material_t>& objMaterials = reader.GetMaterials();
    std::string basedir;
    size_t dirSepFound = filename.find_last_of(WURBLPT_DIR_SEP WURBLPT_NOT_DIR_SEP);
    if (dirSepFound != std::string::npos)
        basedir = filename.substr(0, dirSepFound);
    else
        basedir = ".";

    /* Load all texture files in parallel */

    // Find all texture files
    fprintf(stderr, "  loading texture files...\n");
    std::set<std::string> textureFileSet;
    for (size_t i = 0; i < objMaterials.size(); i++) {
        const tinyobj::material_t& M = objMaterials[i];
        if (M.normal_texname.size() > 0)
            textureFileSet.insert(M.normal_texname);
        if (M.bump_texname.size() > 0)
            textureFileSet.insert(M.bump_texname);
        if (M.diffuse_texname.size() > 0)
            textureFileSet.insert(M.diffuse_texname);
        if (M.specular_texname.size() > 0)
            textureFileSet.insert(M.specular_texname);
        if (M.specular_highlight_texname.size() > 0)
            textureFileSet.insert(M.specular_highlight_texname);
        if (M.alpha_texname.size() > 0)
            textureFileSet.insert(M.alpha_texname);
        if (M.emissive_texname.size() > 0)
            textureFileSet.insert(M.emissive_texname);
    }
    // Copy the set into a vector so that we can do a parallel loop over it
    std::vector<std::string> textureFileVector;
    for (auto it = textureFileSet.cbegin(); it != textureFileSet.cend(); ++it)
        textureFileVector.push_back(*it);
    std::map<std::string, TGD::ArrayContainer> textureFileMap;
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < textureFileVector.size(); i++) {
        const std::string& name = textureFileVector[i];
        std::string fileName = basedir + WURBLPT_DIR_SEP + name;
        for (size_t i = 0; i < fileName.size(); i++)
            if (fileName[i] == WURBLPT_NOT_DIR_SEP_CHAR)
                fileName[i] = WURBLPT_DIR_SEP_CHAR;
        TGD::Error err;
        TGD::ArrayContainer img = TGD::load(fileName, TGD::TagList(), &err);
        fprintf(stderr, "    %s: %s\n", name.c_str(), (err != TGD::ErrorNone ? TGD::strerror(err) : "ok"));
        #pragma omp critical
        textureFileMap.insert(std::pair<std::string, TGD::ArrayContainer>(name, img));
    }

    /* Create the materials */

    std::vector<Material*> materials;
    std::vector<std::string> materialNames;
    std::vector<bool> materialIsLight;
    std::vector<bool> materialWantsTangents;
    std::map<std::string, Texture*> textureMap;
    for (size_t i = 0; i < objMaterials.size(); i++) {
        // get parameters
        const tinyobj::material_t& M = objMaterials[i];
        fprintf(stderr, "  material '%s'...\n", M.name.c_str());
        vec3 dif = vec3(M.diffuse);
        vec3 spc = vec3(M.specular);
        vec3 emi = vec3(M.emission);
        vec3 tra = vec3(M.transmittance);
        float shi = M.shininess;
        float opa = M.dissolve;
        float ior = M.ior;
        // fixups and checks
        if (importBits & ImportBitInvertedTf) {
            tra = vec3(1.0f) - tra;
        }
        if (opa >= 1.0f && max(tra) < 1.0f) {
            opa = average(tra);
            tra = vec3(1.0f) - tra;
        }
        if (opa < 1.0f && max(tra) <= 0.0f) {
            tra = (1.0f - opa) * dif;
        }
        if (any(greaterThan(dif + spc, vec3(1.0f)))) {
            fprintf(stderr, "    warning: not energy conserving\n");
        }
        // get normal map, supported by all materials
        Texture* nrmTex = nullptr;
        if (M.normal_texname.size() > 0)
            nrmTex = importTexture(textureMap, textureFileMap, M.normal_texname,
                    M.normal_texopt.scale, M.normal_texopt.origin_offset, nullptr, LinearizeSRGB_Off);
        else if (M.bump_texname.size() > 0)
            nrmTex = importTexture(textureMap, textureFileMap, M.bump_texname,
                    M.bump_texopt.scale, M.bump_texopt.origin_offset, nullptr, LinearizeSRGB_Off, M.bump_texopt.bump_multiplier);
        // get diffuse texture details, to be used by all materials
        int imgComp;
        Texture* difTex = nullptr;
        if (M.diffuse_texname.size() > 0)
            difTex = importTexture(textureMap, textureFileMap, M.diffuse_texname,
                    M.diffuse_texopt.scale, M.diffuse_texopt.origin_offset, &imgComp);
        bool difTexHasAlpha = (difTex && (imgComp == 2 || imgComp == 4));
        if (!difTexHasAlpha
                && max(spc) <= 0.0f && M.specular_texname.size() == 0
                && ((importBits & ImportBitDisableLightSources) || (max(emi) <= 0.0f && M.emissive_texname.size() == 0))
                && opa >= 1.0f && M.alpha_texname.size() == 0) {
            // generate a simple Lambertian material, which is cheaper to evaluate
            fprintf(stderr, "    using Lambertian material model\n");
            MaterialLambertian* mat = new MaterialLambertian(dif, difTex);
            mat->normalTex = nrmTex;
            materials.push_back(mat);
            materialNames.push_back(M.name);
            materialIsLight.push_back(false);
            materialWantsTangents.push_back(mat->normalTex);
        } else if ((importBits & ImportBitWithGlass)
                && !difTex && M.specular_texname.size() == 0
                && max(emi) <= 0.0f && M.emissive_texname.size() == 0
                && opa < 1.0f && M.alpha_texname.size() == 0) {
            fprintf(stderr, "    using Glass material model\n");
            vec3 absorption = MaterialGlass::transparentColorToAbsorption(dif);
            MaterialGlass* mat = new MaterialGlass(absorption, ior);
            mat->normalTex = nrmTex;
            materials.push_back(mat);
            materialNames.push_back(M.name);
            materialIsLight.push_back(false);
            materialWantsTangents.push_back(mat->normalTex);
        } else {
            // generate ModPhong material
            fprintf(stderr, "    using ModPhong material model\n");
            MaterialModPhong *mat = new MaterialModPhong;
            mat->normalTex = nrmTex;
            mat->haveNIR = false;
            int imgComp;
            mat->diffuse = vec4(dif, 0.0f);
            mat->diffuseTex = difTex;
            mat->diffuseTexHasAlpha = difTexHasAlpha;
            mat->specular = vec4(spc, 0.0f);
            if (M.specular_texname.size() > 0)
                mat->specularTex = importTexture(textureMap, textureFileMap, M.specular_texname,
                        M.specular_texopt.scale, M.specular_texopt.origin_offset, &imgComp);
            mat->specularTexHasAlpha = (mat->specularTex && (imgComp == 2 || imgComp == 4));
            mat->shininess = shi;
            if (M.specular_highlight_texname.size() > 0)
                mat->shininessTex = importTexture(textureMap, textureFileMap, M.specular_highlight_texname,
                        M.specular_highlight_texopt.scale, M.specular_highlight_texopt.origin_offset, nullptr, LinearizeSRGB_Off);
            mat->opacity = opa;
            if (M.alpha_texname.size() > 0)
                mat->opacityTex = importTexture(textureMap, textureFileMap, M.alpha_texname,
                        M.alpha_texopt.scale, M.alpha_texopt.origin_offset, nullptr, LinearizeSRGB_Off);
            mat->indexOfRefraction = ior;
            mat->transmissive = vec4(tra, 0.0f);
            if (!(importBits & ImportBitDisableLightSources))
                mat->emissive = vec4(emi, 0.0f);
            if (!(importBits & ImportBitDisableLightSources) && M.emissive_texname.size() > 0)
                mat->emissiveTex = importTexture(textureMap, textureFileMap, M.emissive_texname,
                        M.emissive_texopt.scale, M.emissive_texopt.origin_offset);
            materials.push_back(mat);
            materialNames.push_back(M.name);
            materialIsLight.push_back(dot(mat->emissive, mat->emissive) > 0.0f || mat->emissiveTex);
            materialWantsTangents.push_back(mat->normalTex);
        }
    }
    for (auto it = textureMap.cbegin(); it != textureMap.cend(); it++)
        scene.take(it->second);
    for (size_t i = 0; i < materials.size(); i++)
        scene.take(materials[i], materialNames[i]);
    if (importBits & ImportBitTwoSidedMaterials) {
        for (size_t i = 0; i < materials.size(); i++) {
            MaterialTwoSided* mat = new MaterialTwoSided(materials[i], materials[i]);
            materials[i] = mat;
            scene.take(mat, materialNames[i]);
        }
    }

    /* Import the shapes */

    // Collect all geometry with the same material into one single MeshInstance.
    // So we loop over the materials, and for each material over all geometry.
    const tinyobj::attrib_t& attrib = reader.GetAttrib();
    const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
    fprintf(stderr, "  importing %zu shapes\n", shapes.size());
    Material* nullMaterial = scene.take(new MaterialLambertian(vec4(0.5f)));
    #pragma omp parallel for schedule(dynamic)
    for (int matId = -1; matId < static_cast<int>(materials.size()); matId++) {
        if (matId < 0) {
            fprintf(stderr, "  importing all geometry that uses no material\n");
        } else {
            fprintf(stderr, "  importing all geometry that uses material '%s'\n", objMaterials[matId].name.c_str());
        }
        for (size_t s = 0; s < shapes.size(); s++) {
            std::map<std::tuple<int, int, int>, unsigned int> indexTupleMap;
            std::vector<vec3> positions;
            std::vector<vec3> normals;
            std::vector<vec2> texcoords;
            std::vector<unsigned int> indices;
            bool haveNormals = true;
            bool haveTexCoords = true;
            const tinyobj::mesh_t& mesh = shapes[s].mesh;
            for (size_t i = 0; i < mesh.indices.size(); i++) {
                size_t triangleIndex = i / 3;
                if (mesh.material_ids[triangleIndex] == matId) {
                    const tinyobj::index_t& index = mesh.indices[i];
                    int vi = index.vertex_index;
                    int ni = index.normal_index;
                    int ti = index.texcoord_index;
                    std::tuple<int, int, int> indexTuple = std::make_tuple(vi, ni, ti);
                    auto it = indexTupleMap.find(indexTuple);
                    if (it == indexTupleMap.end()) {
                        unsigned int newIndex = indexTupleMap.size();
                        assert(vi >= 0);
                        assert(vi < static_cast<int>(attrib.vertices.size() / 3));
                        positions.push_back(vec3(attrib.vertices.data() + 3 * vi));
                        assert(all(isfinite(positions.back())));
                        if (ni < 0)
                            haveNormals = false;
                        if (ti < 0)
                            haveTexCoords = false;
                        if (haveNormals) {
                            assert(ni < static_cast<int>(attrib.normals.size() / 3));
                            vec3 n = vec3(attrib.normals.data() + 3 * ni);
                            if (!all(isfinite(n)) || dot(n, n) < epsilon) {
                                // yes this happens... e.g. in the Amazon Bistro scene
                                fprintf(stderr, "      warning: invalid normals in shape %zu '%s'\n", s, shapes[s].name.c_str());
                                haveNormals = false;
                            } else {
                                // do not trust normals to be of unit length: normalize!
                                normals.push_back(normalize(n));
                            }
                        }
                        if (haveTexCoords) {
                            assert(ti < static_cast<int>(attrib.texcoords.size() / 2));
                            texcoords.push_back(vec2(attrib.texcoords.data() + 2 * ti));
                            assert(all(isfinite(texcoords.back())));
                        }
                        assert(newIndex == positions.size() - 1);
                        assert(!haveNormals || newIndex == normals.size() - 1);
                        assert(!haveTexCoords || newIndex == texcoords.size() - 1);
                        indices.push_back(newIndex);
                        indexTupleMap.insert(std::make_pair(indexTuple, newIndex));
                    } else {
                        indices.push_back(it->second);
                    }
                }
            }
            if (indices.size() == 0) {
                continue;
            }
            if (!haveNormals) {
                fprintf(stderr, "      computing normals for shape %zu '%s'\n", s, shapes[s].name.c_str());
                normals = computeNormals(positions, indices);
            }
            if (!haveTexCoords) {
                texcoords.clear();
            }
            Material* mat;
            bool matIsLight;
            bool matWantsTangents;
            if (matId < 0) {
                mat = nullMaterial;
                matIsLight = false;
                matWantsTangents = false;
            } else {
                mat = materials[matId];
                matIsLight = materialIsLight[matId];
                matWantsTangents = materialWantsTangents[matId];
            }
            Mesh* wurblMesh = new Mesh(positions, normals, texcoords, indices, transformation, matWantsTangents);
            MeshInstance* wurblMeshInstance = new MeshInstance(wurblMesh, mat);
            #pragma omp critical
            {
                scene.take(wurblMesh);
                scene.take(wurblMeshInstance, (matIsLight && !(importBits & ImportBitDisableHotSpots)) ? HotSpot : ColdSpot);
            }
        }
    }

    fprintf(stderr, "%s: import done\n", filename.c_str());
    return true;
}

/*! \brief Import triangle geometry from an OBJ file.
 * \param filename      Name of the OBJ file
 * \param shapeName     If not empty, import only this shape
 * \param T             Transform geometry
 */
inline Mesh* importGeometry(const std::string& filename,
        const std::string& shapeName = std::string(),
        const Transformation& T = Transformation())
{
    tinyobj::ObjReaderConfig conf;
    conf.triangulate = true;
    conf.vertex_color = false;
    tinyobj::ObjReader reader;
    reader.ParseFromFile(filename, conf);
    if (reader.Warning().size() > 0) {
        std::vector<std::string> lines = tinyObjMsgToLines(reader.Warning());
        for (size_t i = 0; i < lines.size(); i++)
            fprintf(stderr, "  warning: %s\n", lines[i].c_str());
    }
    if (!reader.Valid()) {
        if (reader.Error().size() > 0) {
            std::vector<std::string> lines = tinyObjMsgToLines(reader.Error());
            for (size_t i = 0; i < lines.size(); i++)
                fprintf(stderr, "  error: %s\n", lines[i].c_str());
        } else {
            fprintf(stderr, "  unknown error\n");
        }
        fprintf(stderr, "%s: import failure\n", filename.c_str());
        return nullptr;
    }

    // Read all geometry (or at least the shape with the given name)
    // and ignore all materials.
    const tinyobj::attrib_t& attrib = reader.GetAttrib();
    const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
    std::map<std::tuple<int, int, int>, unsigned int> indexTupleMap;
    std::vector<vec3> positions;
    std::vector<vec3> normals;
    std::vector<vec2> texcoords;
    std::vector<unsigned int> indices;
    bool haveNormals = true;
    bool haveTexCoords = true;
    for (size_t s = 0; s < shapes.size(); s++) {
        if (shapeName.size() > 0 && shapeName != shapes[s].name)
            continue;
        const tinyobj::mesh_t& mesh = shapes[s].mesh;
        for (size_t i = 0; i < mesh.indices.size(); i++) {
            const tinyobj::index_t& index = mesh.indices[i];
            int vi = index.vertex_index;
            int ni = index.normal_index;
            int ti = index.texcoord_index;
            std::tuple<int, int, int> indexTuple = std::make_tuple(vi, ni, ti);
            auto it = indexTupleMap.find(indexTuple);
            if (it == indexTupleMap.end()) {
                unsigned int newIndex = indexTupleMap.size();
                assert(vi >= 0);
                positions.push_back(vec3(attrib.vertices.data() + 3 * vi));
                if (ni < 0)
                    haveNormals = false;
                if (ti < 0)
                    haveTexCoords = false;
                if (haveNormals) {
                    // do not trust normals to be of unit length: normalize!
                    normals.push_back(normalize(vec3(attrib.normals.data() + 3 * ni)));
                }
                if (haveTexCoords) {
                    texcoords.push_back(vec2(attrib.texcoords.data() + 2 * ti));
                }
                indices.push_back(newIndex);
                indexTupleMap.insert(std::make_pair(indexTuple, newIndex));
            } else {
                indices.push_back(it->second);
            }
        }
    }
    if (!haveNormals) {
        normals = computeNormals(positions, indices);
    }
    if (!haveTexCoords) {
        texcoords.clear();
    }
    return new Mesh(positions, normals, texcoords, indices, T);
}

}
