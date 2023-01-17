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

/* This demonstrates a dynamic scene recorded with a Time-of-Flight distance sensor.
 * Ground Truth is provided to analyse various effects. */

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

class QuadAnimation : public AnimationKeyframes
{
public:
    QuadAnimation() : AnimationKeyframes(
            0.0f, Transformation(vec3(-1.0f, 0.5f, -1.5f)),
            5.0f, Transformation(vec3(+1.0f, 0.5f, -1.5f)))
    {
    }
};

class ObjectAnimation : public AnimationKeyframes
{
public:
    ObjectAnimation() : AnimationKeyframes()
    {
        addKeyframe(0.0f,  Transformation(vec3(0.0f, -0.3f, -1.0f), toQuat(radians(  0.0f), vec3(0.0f, 1.0f, 0.5f))));
        addKeyframe(1.25f, Transformation(vec3(0.0f, -0.3f, -1.0f), toQuat(radians( 60.0f), vec3(0.0f, 1.0f, 0.5f))));
        addKeyframe(2.5f,  Transformation(vec3(0.0f, -0.3f, -1.0f), toQuat(radians(120.0f), vec3(0.0f, 1.0f, 0.5f))));
        addKeyframe(3.75f, Transformation(vec3(0.0f, -0.3f, -1.0f), toQuat(radians(180.0f), vec3(0.0f, 1.0f, 0.5f))));
        addKeyframe(5.0f,  Transformation(vec3(0.0f, -0.3f, -1.0f), toQuat(radians(240.0f), vec3(0.0f, 1.0f, 0.5f))));
    }
};

void createScene(Scene& scene)
{
    Material* bgMaterial = scene.take(new MaterialLambertian(vec4(1.0f)));
    //Material* bgMaterial = scene.take(new MaterialLambertian(vec4(0.9f)));
    Transformation bgTransformation(vec3(0.0f, 0.0f, -2.0f), quat::null(), vec3(5.0f));
    scene.take(new MeshInstance(scene.take(generateQuad(bgTransformation)), bgMaterial));

    Material* quadMaterial = scene.take(new MaterialModPhong(vec3(0.7f), vec3(0.3f), 100.0f));
    //Material* quadMaterial = scene.take(new MaterialLambertian(vec4(0.7f)));
    Transformation quadTransformation(vec3(0.0f), quat::null(), vec3(0.2f));
    int quadAnimationIndex = scene.take(new QuadAnimation);
    scene.take(new MeshInstance(scene.take(generateQuad(quadTransformation)), quadMaterial, quadAnimationIndex));

    Material* objectMaterial = scene.take(new MaterialModPhong(vec3(0.5f), vec3(0.5f), 100.0f));
    //Material* objectMaterial = scene.take(new MaterialLambertian(vec4(0.5f)));
    Transformation objectTransformation(vec3(0.0f), quat::null(), vec3(0.33f));
    int objectAnimationIndex = scene.take(new ObjectAnimation);
    scene.take(new MeshInstance(scene.take(generateIcosahedron(objectTransformation)), objectMaterial, objectAnimationIndex));

    Material* lightFrontSide = scene.take(new LightTof(40.0f / (4.0f * pi), radians(120.0f)));
    Material* lightBackSide = scene.take(new MaterialLambertian(vec4(0.0f)));
    Material* lightMaterial = scene.take(new MaterialTwoSided(lightFrontSide, lightBackSide));
    Transformation lightTransformation(
            vec3(0.0f),
            toQuat(radians(180.0f), vec3(1.0f, 0.0f, 0.0f)),
            vec3(0.10f, 0.05f, 1.0f));
    scene.take(new MeshInstance(scene.take(generateQuad(lightTransformation)), lightMaterial), HotSpot);
}

int main(void)
{
    bool preview = true;

    unsigned int width        = 352;
    unsigned int height       = 288;
    unsigned int samples_sqrt = (preview ? 10 : 30);
    float shotNoiseFactor     = (preview ? 0.0f : 1.0f);

    Scene scene;
    createScene(scene);

    SensorTofAmcw sensor(width, height);
    sensor.pauseTime = 36000.0f;

    Optics optics(Projection(radians(70.0f), sensor.aspectRatio()));

    Camera camera(optics);

    Parameters params;
    params.maxPathComponents = 2; // only direct illumination!
    params.rrThreshold = 0.0f;

    float start = 0.0f;
    float end = 5.0f;
    float frameDuration = 1.0f / sensor.fps();
    unsigned int frameCount = (end - start) / frameDuration;
    for (unsigned int frame = 0; frame < frameCount; frame++) {
        char frameStr[16];
        std::snprintf(frameStr, sizeof(frameStr), "%04u", frame);
        std::string framePrefix = std::string("frame") + frameStr;
        TGD::Array<float> pmdPhaseImages[sensor.phaseImageCount];
        for (unsigned int phaseImage = 0; phaseImage < sensor.phaseImageCount; phaseImage++) {
            char phaseStr[16];
            std::snprintf(phaseStr, sizeof(phaseStr), "%u", phaseImage);
            std::string phasePrefix = std::string("phase") + phaseStr;
            float t0 = start + frame * frameDuration + phaseImage * sensor.phaseImageDuration();
            float t1 = t0 + sensor.exposureTime / 1e6f;
            scene.updateBVH(t0, t1);
            sensor.setPhaseIndex(phaseImage);
            mcpt(sensor, camera, scene, samples_sqrt, t0, t1, params);
            TGD::save(sensor.energy(), framePrefix + "-" + phasePrefix + "-energies.tgd");
            pmdPhaseImages[phaseImage] = sensor.phase(shotNoiseFactor);
            TGD::save(pmdPhaseImages[phaseImage], framePrefix + "-" + phasePrefix + "-images.tgd");
        }
        TGD::Array<float> pmdResult = sensor.result(pmdPhaseImages);
        TGD::save(pmdResult, framePrefix + "-results.tgd");
        TGD::Array<float> pmdCoords = cameraSpaceToPMDSpace(cameraSpaceDistanceToCoords(pmdResult, optics.projection, optics.distortion));
        TGD::save(pmdCoords, framePrefix + "-coordinates.tgd");

        float gtTime = start + frame * frameDuration;
        float gtTimePrev = start + (frame - 1.0f) * frameDuration;
        float gtTimeNext = start + (frame + 1.0f) * frameDuration;
        scene.updateBVH(gtTime, gtTime);
        GroundTruth gt = getGroundTruth(sensor, camera, scene, gtTime, gtTimePrev, gtTimeNext);
        TGD::save(gt.cameraSpaceDistances,    framePrefix + "-groundtruth-distances.tgd");
        TGD::save(gt.cameraSpacePositions,    framePrefix + "-groundtruth-coordinates.tgd");
        TGD::save(gt.worldSpaceOffsetToPrev,  framePrefix + "-groundtruth-wsflow-backward.tgd");
        TGD::save(gt.worldSpaceOffsetToNext,  framePrefix + "-groundtruth-wsflow-forward.tgd");
        TGD::save(gt.cameraSpaceOffsetToPrev, framePrefix + "-groundtruth-csflow-backward.tgd");
        TGD::save(gt.cameraSpaceOffsetToNext, framePrefix + "-groundtruth-csflow-forward.tgd");
        TGD::save(gt.pixelSpaceOffsetToPrev,  framePrefix + "-groundtruth-psflow-backward.tgd");
        TGD::save(gt.pixelSpaceOffsetToNext,  framePrefix + "-groundtruth-psflow-forward.tgd");
    }

    return 0;
}
