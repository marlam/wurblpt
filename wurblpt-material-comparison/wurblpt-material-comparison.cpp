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

void createScene(Scene& scene)
{
    // the floor
    Texture* floorTexture = scene.take(new TextureChecker(vec4(0.6f), vec4(0.4f), 200, 200));
    Material* floorMaterial = scene.take(new MaterialLambertian(vec4(0.0f), floorTexture));
    Transformation floorTransformation(vec3(0.0f), toQuat(radians(-90.0f), vec3(1.0f, 0.0f, 0.0f)), vec3(20.0f));
    Mesh* quadData = scene.take(generateQuad(floorTransformation));
    scene.take(new MeshInstance(quadData, floorMaterial));

    // the first object
    //Material* firstObjectMaterial = scene.take(new MaterialLambertian(vec3(1.0f, 0.0f, 0.0f)));
    Material* firstObjectMaterial = scene.take(new MaterialMirror(vec3(1.0f, 1.0f, 1.0f)));
    //Material* firstObjectMaterial = scene.take(new MaterialRGLSpectral("cm_white_spec.bsdf"));
    scene.take(new Sphere(vec3(-1.5f, 1.0f, 0.0f), 1.0f, firstObjectMaterial));

    // the second object
    //Material* secondObjectMaterial = scene.take(new MaterialLambertian(vec3(0.0f, 1.0f, 0.0f)));
    //Material* secondObjectMaterial = scene.take(new MaterialRGL("cm_white_rgb.bsdf"));
    Material* secondObjectMaterial = scene.take(new MaterialModPhong(vec3(0.4f, 0.1f, 0.1f), vec3(0.6f, 0.6f, 0.6f), 500.0f));
    scene.take(new Sphere(vec3(+1.5f, 1.0f, 0.0f), 1.0f, secondObjectMaterial));

    // the environment map
    Texture* tex = scene.take(new TextureConstant(vec4(1.0f)));
    //Texture* tex = scene.take(createTextureImage("aerodynamics_workshop_8k.hdr"));
    //Texture* tex = scene.take(createTextureImage("kloofendal_48d_partly_cloudy_4k.hdr"));
    EnvironmentMap* envmap = scene.take(new EnvironmentMapEquiRect(tex));
    //envmap->initializeImportanceSampling(1024);
}

int main(void)
{
    bool preview = true;

    unsigned int width        = (preview ? 960 : 1920);
    unsigned int height       = (preview ? 540 : 1080);
    unsigned int samples_sqrt = (preview ?   8 :   50);

    Scene scene;
    createScene(scene);

    SensorRGB sensor(width, height);
    Optics optics(Projection(radians(40.0f), sensor.aspectRatio()));

    Camera camera(optics, Transformation::fromLookAt(vec3(0.0f, 5.0f, 5.0f), vec3(0.0f, 1.0f, 0.0f)));

    scene.updateBVH();

    mcpt(sensor, camera, scene, samples_sqrt);
    const TGD::Array<float>& hdrImg = sensor.result();
    TGD::save(hdrImg, "image.tgd");

    return 0;
}
