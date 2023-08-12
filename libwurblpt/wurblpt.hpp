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

#ifdef __GNUC__
# include <cpuid.h>
#endif

#include <omp.h>

#if __has_include(<unistd.h>)
# include <unistd.h>
#endif

#include <ctime>

#include <tgd/array.hpp>

#include "scene.hpp"
#include "camera.hpp"
#include "sampler.hpp"
#include "mpi.hpp"

// for convenience:
#include "animation_keyframes.hpp"
#include "texture_image.hpp"
#include "texture_noise.hpp"
#include "envmap.hpp"
#include "fresnel.hpp"
#include "hitable_triangle.hpp"
#include "material_lambertian.hpp"
#include "material_mirror.hpp"
#include "material_glass.hpp"
#include "material_modphong.hpp"
#include "material_ggx.hpp"
#include "material_phase_function_isotropic.hpp"
#include "material_rgl.hpp"
#include "light_diffuse.hpp"
#include "light_spot.hpp"
#include "light_tof.hpp"
#include "generator.hpp"
#include "sphere.hpp"
#include "mesh.hpp"
#include "medium.hpp"
#include "import.hpp"
#include "postproc.hpp"
#include "sensor.hpp"
#include "sensor_rgb.hpp"
#include "sensor_tof_amcw.hpp"


namespace WurblPT {

/* Path Tracing parameters */

class Parameters
{
public:
    unsigned int maxPathComponents; // 1 == from sensor to light source, 2 == direct illumination of surfaces,
                                    // 3 == one indirect bounce, and so on
    float rrThreshold;              // If the attenuation factor of a path falls beneath this value, Russian roulette
                                    // starts to be used
    bool randomizeRayOverPixel;     // should be true for antialiasing and flying pixels (for ToF)
    float minHitDistance;           // minimum required distance between hits (to avoid hitting the same surface again)

    Parameters() :
        maxPathComponents(128),
        rrThreshold(1.0f),
        randomizeRayOverPixel(true),
        minHitDistance(0.00001f)
    {
    }
};

/* The core path tracing function */

// Power heuristic for Multiple Importance Sampling / Next Event Estimation
inline float powerHeuristicWeight(float f, float g)
{
    f *= f;
    g *= g;
    return (f + g > 0.0f ? f / (f + g) : 0.0f);
}

inline void tracePath(
        const Sensor& sensor,
        float t0, float t1,
        float* sampleAccumulator,
        const Ray& startRay,
        const Scene& scene,
        size_t hotSpotsSize,
        float invHotSpotsSize,
        const Parameters& params,
        AnimationCache& animationCache,
        Prng& prng)
{
    assert(animationCache.t() == startRay.time);

    vec4 attenuation(1.0f);
    float pathLength = 0.0f;
    vec4 opticalPathLength(0.0f);
    Ray ray = startRay;

    for (unsigned int pathComponent = 0;; pathComponent++) {
        vec4 radianceToAccumulate;

        // Let the ray hit the scene
        HitRecord hr = scene.bvh().hit(ray, RayIntersectionHelper(ray),
                params.minHitDistance, maxval, params.minHitDistance,
                animationCache, prng);

        // No hit: sample environment map if available and end the path
        if (!hr.haveHit) {
            if (scene.environmentMap()) {
                radianceToAccumulate = attenuation * scene.environmentMap()->L(ray.direction, ray.time);
                sensor.accumulateRadiance(ray, pathComponent,
                        std::numeric_limits<float>::max(),
                        vec4(std::numeric_limits<float>::max()),
                        std::numeric_limits<float>::max(),
                        radianceToAccumulate, hr, t0, t1, sampleAccumulator);
            }
            break;
        }

        // Have a hit: update the path lengths
        pathLength += hr.a;
        opticalPathLength += hr.a * ray.refractiveIndex;

        // Stop if this is the last path component
        if (!(pathComponent + 1 < params.maxPathComponents))
            break;

        // Scatter the current ray to get the next path component
        ScatterRecord sr = hr.hitable->material()->scatter(ray, hr, prng);

        // Accumulated emitted radiance
        radianceToAccumulate = attenuation * hr.hitable->material()->emitted(ray, hr);
        sensor.accumulateRadiance(ray, pathComponent, pathLength, opticalPathLength,
                (pathComponent == 0 ? 0.0f : hr.a),
                radianceToAccumulate, hr, t0, t1, sampleAccumulator);

        // Stop if there is no further scattering
        if (sr.type == ScatterNone)
            break;

        // Compute attenuation for next path component
        vec4 nextAttenuation = attenuation * sr.attenuation;
        if (sr.type == ScatterRandom) {
            if (sr.pdf > 0.0f)
                nextAttenuation /= sr.pdf;
            else
                nextAttenuation = vec4(0.0f);
        }

        // Sample direct illumination (multiple importance sampling, next event estimation)
        if (sr.type == ScatterRandom && hotSpotsSize > 0) {
            // update the attenuation for the next path component (MIS power heuristic)
            float hotSpotsPdf = 0.0f;
            for (size_t i = 0; i < hotSpotsSize; i++)
                hotSpotsPdf += scene.hotSpots()[i]->pdfValue(hr.position, sr.direction, animationCache, prng);
            hotSpotsPdf *= invHotSpotsSize;
            nextAttenuation *= powerHeuristicWeight(sr.pdf, hotSpotsPdf);
            // choose a hot spot randomly
            size_t hotSpotIndex = prng.in01() * hotSpotsSize;
            hotSpotIndex = min(hotSpotIndex, hotSpotsSize - 1);
            // get direction to it
            vec3 directDir = scene.hotSpots()[hotSpotIndex]->direction(hr.position, animationCache, prng);
            // get the pdf value for this direction
            float directPdf = 0.0f;
            for (size_t i = 0; i < hotSpotsSize; i++)
                directPdf += scene.hotSpots()[i]->pdfValue(hr.position, directDir, animationCache, prng);
            directPdf *= invHotSpotsSize;
            // avoid corner cases where the pdf is 0
            if (directPdf > 0.0f) {
                // get information about a ray going from our current hit point in this direction
                ScatterRecord directSR = hr.hitable->material()->scatterToDirection(ray, hr, directDir);
                // check if the direction is possible
                if (directSR.pdf > 0.0f) {
                    // shoot a ray from our current hit point in this direction
                    Ray directRay(hr.position, directDir, ray.time, directSR.refractiveIndex);
                    HitRecord directHR = scene.bvh().hit(directRay, RayIntersectionHelper(directRay),
                            params.minHitDistance, maxval, params.minHitDistance,
                            animationCache, prng);
                    // check if we hit the hot spot we chose
                    if (directHR.haveHit && directHR.hitable == scene.hotSpots()[hotSpotIndex]) {
                        // add the contribution of the hot spot using the power heuristic weight
                        float weight = powerHeuristicWeight(directPdf, directSR.pdf);
                        radianceToAccumulate = attenuation * directSR.attenuation / directPdf * weight * directHR.hitable->material()->emitted(directRay, directHR);
                        float pathLengthToAccumulate = pathLength + directHR.a;
                        vec4 opticalPathLengthToAccumulate = opticalPathLength + directHR.a * directRay.refractiveIndex;
                        sensor.accumulateRadiance(ray, pathComponent + 1,
                                pathLengthToAccumulate, opticalPathLengthToAccumulate,
                                directHR.a, radianceToAccumulate,
                                directHR, t0, t1, sampleAccumulator);
                    }
                }
            }
        } else if (sr.type == ScatterRandom && scene.environmentMap()
                && scene.environmentMap()->supportsImportanceSampling()) {
            // update the throughput for the next path segment
            float lightsP = scene.environmentMap()->p(sr.direction);
            nextAttenuation *= powerHeuristicWeight(sr.pdf, lightsP);
            // get direction
            vec3 lightDir = scene.environmentMap()->d(prng);
            // get the pdf value for this direction
            float lightDirP = scene.environmentMap()->p(lightDir);
            // get information about a ray going from our current hit point in this direction
            ScatterRecord lightSR = hr.hitable->material()->scatterToDirection(ray, hr, lightDir);
            // check if the direction is possible
            if (lightSR.pdf > 0.0f) {
                // shoot a ray from our current hit point in this direction
                Ray lightRay(hr.position, lightDir, ray.time, lightSR.refractiveIndex);
                HitRecord lightHR = scene.bvh().hit(lightRay, RayIntersectionHelper(lightRay),
                        params.minHitDistance, maxval, params.minHitDistance,
                        animationCache, prng);
                // check if we hit the environment map
                if (!lightHR.haveHit) {
                    // add the contribution of the environment using the power heuristic weight
                    float weight = powerHeuristicWeight(lightDirP, lightSR.pdf);
                    radianceToAccumulate = attenuation * lightSR.attenuation / lightDirP * weight
                        * scene.environmentMap()->L(lightDir);
                    sensor.accumulateRadiance(ray, pathComponent + 1,
                            std::numeric_limits<float>::max(),
                            vec4(std::numeric_limits<float>::max()),
                            std::numeric_limits<float>::max(),
                            radianceToAccumulate, lightHR, t0, t1, sampleAccumulator);
                }
            }
        }

        // Update attenuation and ray
        attenuation = nextAttenuation;
        ray = Ray(hr.position, sr.direction, ray.time, sr.refractiveIndex);

        // The following is the russian roulette strategy used by pbrtv3 and mitsuba 0.5.x,
        // except that pbrtv3 starts at pathComponent >= 4.
        if (max(attenuation) < params.rrThreshold && pathComponent >= 5) {
            // Russian roulette: q is the probability to cancel the ray, thus continuing
            // rays need to be weighted with rrWeight=1/(1-q). See PBR3 Sec 13.7.
            // We used to have q = max(0.05f, 1.0f - max(attenuation)), but that can be near 1
            // so that rrWeight becomes very large.
            // Mitsuba2 uses an upper bound of 0.95f for q. Additionally, we make sure that
            // q is never < 0 which might have happened with an attenuation slightly larger
            // than 1.
            float q = clamp(1.0f - max(attenuation), 0.0f, 0.95f);
            if (prng.in01() < q)
                break;
            float rrWeight = 1.0f / (1.0f - q);
            attenuation *= rrWeight;
        }
    }
}

/* The main Path Tracer function */

inline void mcpt(
        MPICoordinator& mpiCoordinator,
        Sensor& sensor,
        const Camera& camera,
        const Scene& scene,
        unsigned int samplesSqrt,
        float t0 = 0.0f, float t1 = 0.0f,
        const Parameters& params = Parameters())
{
    // Make sure the BVH is up to date for [t0,t1]
    assert(!scene.bvhNeedsUpdate(t0, t1));

    bool tty = false;
#if __has_include(<unistd.h>)
    tty = isatty(fileno(stderr));
#endif

#ifndef _MSC_VER
    struct timespec cpuTimeStart;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpuTimeStart);
#endif

    TGD::ArrayContainer* pixelArray = sensor.pixelArray();
    unsigned int width = sensor.width();
    unsigned int height = sensor.height();
    unsigned int samples = samplesSqrt * samplesSqrt;
    const Camera::RayHelper cameraRayHelper = camera.getRayHelper(t0, width, height);
    float invSamples = 1.0f / samples;
    float invSamplesSqrt = 1.0f / samplesSqrt;
    vec2 invSize = vec2(1.0f / width, 1.0f / height);
    size_t hotSpotsSize = scene.hotSpots().size();
    float invHotSpotsSize = 1.0f / scene.hotSpots().size();
    std::vector<AnimationCache> perThreadAnimationCaches(omp_get_max_threads(), AnimationCache(scene.animations()));
    unsigned int threadCount = 0; // will be filled in the loop over a pixel block

    fprintf(stderr, "Number of hitables that are hot spots: %zu\n", hotSpotsSize);
    fprintf(stderr, "Rendering %ux%u pixels with %u samples for %.3fs-%.3fs.\n", width, height, samples, t0, t1);

    // Loop over pixel blocks provided by the mpiCoordinator
    mpiCoordinator.init(width, height, static_cast<float*>(pixelArray->data()), pixelArray->componentCount());
    for (;;) {
        unsigned int blockStart;
        unsigned int blockSize;
        mpiCoordinator.getBlock(&blockStart, &blockSize);
        if (blockSize == 0)
            break;
        fprintf(stderr, "%s: rendering block of size %u starting at %u\n", mpiCoordinator.processId(), blockSize, blockStart);
        float invBlockSize = 1.0f / blockSize;

        // Trivially parallel loop over pixels in the block
        float progress = 0.0f;
        float last_reported_progress = progress;
        if (tty)
            fprintf(stderr, "%s: progress:   0.0%%", mpiCoordinator.processId());
        else
            fprintf(stderr, "%s: progress:   0.0%%\n", mpiCoordinator.processId());
        #pragma omp parallel for schedule(dynamic)
        for (unsigned int blockPixel = 0; blockPixel < blockSize; blockPixel++) {
            unsigned int pixel = blockStart + blockPixel;
            threadCount = omp_get_num_threads();
            unsigned int threadIndex = omp_get_thread_num();
            unsigned int y = pixel / width;
            unsigned int x = pixel % width;
            Prng prng(pixel);
            // Initialize sample accumulator
            float sampleAccumulator[Sensor::maxPixelComponents];
            for (int k = 0; k < Sensor::maxPixelComponents; k++)
                sampleAccumulator[k] = 0.0f;
            // Loop over samples
            for (unsigned int sampleIndex = 0; sampleIndex < samples; sampleIndex++) {
                vec2 uv = vec2(x, y);
                if (params.randomizeRayOverPixel) {
                    /* Simple stratified sampling / jittering; see e.g.
                     * https://psgraphics.blogspot.com/2018/10/flavors-of-sampling-in-ray-tracing.html */
                    unsigned int j = sampleIndex / samplesSqrt;
                    unsigned int i = sampleIndex % samplesSqrt;
                    uv += vec2(i + prng.in01(), j + prng.in01()) * invSamplesSqrt;
                } else {
                    uv += vec2(0.5f, 0.5f);
                }
                uv *= invSize;
                Ray r = camera.getRay(uv.x(), uv.y(), t0, t1, cameraRayHelper, prng);
                perThreadAnimationCaches[threadIndex].init(r.time);
                tracePath(sensor, t0, t1, sampleAccumulator,
                        r, scene, hotSpotsSize, invHotSpotsSize,
                        params, perThreadAnimationCaches[threadIndex],
                        prng);
            }
            // Write to results (lock-free)
            sensor.finishPixel(pixel, invSamples, sampleAccumulator);
            // Report progress
            if (omp_get_thread_num() == 0) {
                progress = 100.0f * blockPixel * invBlockSize;
                float progress_step = (blockSize == width * height ? 0.1f : 5.0f);
                if (progress - last_reported_progress > progress_step && progress < (100.0f - 0.5f * progress_step)) {
                    if (tty)
                        fprintf(stderr, "\b\b\b\b\b\b% 5.1f%%", progress);
                    else
                        fprintf(stderr, "%s: progress: % 5.1f%%\n", mpiCoordinator.processId(), progress);
                    last_reported_progress = progress;
                }
            }
        }

        mpiCoordinator.submitBlock(blockStart, blockSize);
        if (tty)
            fprintf(stderr, "\b\b\b\b\b\b100.0%%\n");
        else
            fprintf(stderr, "%s: progress: 100.0%%\n", mpiCoordinator.processId());
    }

    fprintf(stderr, "%s is done\n", mpiCoordinator.processId());
    mpiCoordinator.finish();

    double cpuTimeSeconds = std::numeric_limits<double>::quiet_NaN();
#ifndef _MSC_VER
    struct timespec cpuTimeStop;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpuTimeStop);
    double cpuTimeStartSeconds = cpuTimeStart.tv_sec + cpuTimeStart.tv_nsec * 1e-9;
    double cpuTimeStopSeconds = cpuTimeStop.tv_sec + cpuTimeStop.tv_nsec * 1e-9;
    cpuTimeSeconds = cpuTimeStopSeconds - cpuTimeStartSeconds;
#endif

    std::string cpuModel = "unknown";
#ifdef __GNUC__
    if (__get_cpuid_max(0x80000004, NULL)) {
        uint32_t brand[12];
        __get_cpuid(0x80000002, brand+0x0, brand+0x1, brand+0x2, brand+0x3);
        __get_cpuid(0x80000003, brand+0x4, brand+0x5, brand+0x6, brand+0x7);
        __get_cpuid(0x80000004, brand+0x8, brand+0x9, brand+0xa, brand+0xb);
        cpuModel = std::string(reinterpret_cast<const char*>(brand), sizeof(brand));
    }
#endif

#define WURBLPT_XSTR(x) WURBLPT_STR(x)
#define WURBLPT_STR(x) #x
#if defined(__clang__)
    const char* compiler = "clang++ " WURBLPT_XSTR(__clang_major__) "." WURBLPT_XSTR(__clang_minor__) "." WURBLPT_XSTR(__clang_patchlevel__);
#elif defined(__GNUC__) && defined(__GNUC_MINOR__)
    const char* compiler = "g++ " WURBLPT_XSTR(__GNUC__) "." WURBLPT_XSTR(__GNUC_MINOR__) "." WURBLPT_XSTR(__GNUC_PATCHLEVEL__);
#else
    const char* compiler = "unknown";
#endif
#undef WURBLPT_STR
#undef WURBLPT_XSTR

    if (mpiCoordinator.mainProcess()) {
        pixelArray->globalTagList().set("WURBLPT/COMPILER", compiler);
        pixelArray->globalTagList().set("WURBLPT/SAMPLES_PER_PIXEL", std::to_string(samples));
        pixelArray->globalTagList().set("WURBLPT/MAX_PATH_COMPONENTS", std::to_string(params.maxPathComponents));
        pixelArray->globalTagList().set("WURBLPT/RUSSIAN_ROULETTE_THRESHOLD", std::to_string(params.rrThreshold));
        if (mpiCoordinator.worldSize() == 1) {
            pixelArray->globalTagList().set("WURBLPT/CPU_MODEL", cpuModel.c_str());
            pixelArray->globalTagList().set("WURBLPT/CPU_THREADS", std::to_string(threadCount));
            pixelArray->globalTagList().set("WURBLPT/CPU_SECONDS", std::to_string(cpuTimeSeconds));
        }
    }
}

// Convenience wrapper for programs that don't use MPI
inline void mcpt(
        Sensor& sensor,
        const Camera& camera,
        const Scene& scene,
        unsigned int samplesSqrt,
        float t0 = 0.0f, float t1 = 0.0f,
        const Parameters& params = Parameters())
{
    MPICoordinator mpiCoordinator;
    mcpt(mpiCoordinator, sensor, camera, scene, samplesSqrt, t0, t1, params);
}

/* Ground Truth generation */

class GroundTruth
{
public:
    static constexpr unsigned int WorldSpacePositions         = (1 << 0);
    static constexpr unsigned int WorldSpaceGeometryNormals   = (1 << 1);
    static constexpr unsigned int WorldSpaceGeometryTangents  = (1 << 2);
    static constexpr unsigned int WorldSpaceMaterialNormals   = (1 << 3);
    static constexpr unsigned int WorldSpaceMaterialTangents  = (1 << 4);
    static constexpr unsigned int CameraSpacePositions        = (1 << 5);
    static constexpr unsigned int CameraSpaceGeometryNormals  = (1 << 6);
    static constexpr unsigned int CameraSpaceGeometryTangents = (1 << 7);
    static constexpr unsigned int CameraSpaceMaterialNormals  = (1 << 8);
    static constexpr unsigned int CameraSpaceMaterialTangents = (1 << 9);
    static constexpr unsigned int CameraSpaceDepths           = (1 << 10);
    static constexpr unsigned int CameraSpaceDistances        = (1 << 11);
    static constexpr unsigned int TexCoords                   = (1 << 12);
    static constexpr unsigned int WorldSpaceOffsetToPrev      = (1 << 13);
    static constexpr unsigned int WorldSpaceOffsetToNext      = (1 << 14);
    static constexpr unsigned int CameraSpaceOffsetToPrev     = (1 << 15);
    static constexpr unsigned int CameraSpaceOffsetToNext     = (1 << 16);
    static constexpr unsigned int PixelSpaceOffsetToPrev      = (1 << 17);
    static constexpr unsigned int PixelSpaceOffsetToNext      = (1 << 18);
    static constexpr unsigned int Materials                   = (1 << 19);
    static constexpr unsigned int All                         = (1 << 20) - 1;

    unsigned int bits;
    TGD::Array<float> worldSpacePositions;
    TGD::Array<float> worldSpaceGeometryNormals;
    TGD::Array<float> worldSpaceGeometryTangents;
    TGD::Array<float> worldSpaceMaterialNormals;
    TGD::Array<float> worldSpaceMaterialTangents;
    TGD::Array<float> cameraSpacePositions;
    TGD::Array<float> cameraSpaceGeometryNormals;
    TGD::Array<float> cameraSpaceGeometryTangents;
    TGD::Array<float> cameraSpaceMaterialNormals;
    TGD::Array<float> cameraSpaceMaterialTangents;
    TGD::Array<float> cameraSpaceDepths; // == -cameraSpacePosition.z
    TGD::Array<float> cameraSpaceDistances; // == length(cameraSpacePosition)
    TGD::Array<float> texCoords;
    TGD::Array<float> worldSpaceOffsetToPrev;
    TGD::Array<float> worldSpaceOffsetToNext;
    TGD::Array<float> cameraSpaceOffsetToPrev;
    TGD::Array<float> cameraSpaceOffsetToNext;
    TGD::Array<float> pixelSpaceOffsetToPrev;
    TGD::Array<float> pixelSpaceOffsetToNext;
    TGD::Array<int> materials;

    GroundTruth() : bits(0)
    {
    }

    GroundTruth(unsigned int width, unsigned int height, unsigned int bits = All) :
        bits(bits)
    {
        // World Space Geometry
        if (bits & WorldSpacePositions) {
            worldSpacePositions = TGD::Array<float>({ width, height }, 3);
            worldSpacePositions.componentTagList(0).set("INTERPRETATION", "x");
            worldSpacePositions.componentTagList(1).set("INTERPRETATION", "y");
            worldSpacePositions.componentTagList(2).set("INTERPRETATION", "z");
        }
        if (bits & WorldSpaceGeometryNormals) {
            worldSpaceGeometryNormals = TGD::Array<float>({ width, height }, 3);
            worldSpaceGeometryNormals.componentTagList(0).set("INTERPRETATION", "nx");
            worldSpaceGeometryNormals.componentTagList(1).set("INTERPRETATION", "ny");
            worldSpaceGeometryNormals.componentTagList(2).set("INTERPRETATION", "nz");
        }
        if (bits & WorldSpaceGeometryTangents) {
            worldSpaceGeometryTangents = TGD::Array<float>({ width, height }, 3);
            worldSpaceGeometryTangents.componentTagList(0).set("INTERPRETATION", "tx");
            worldSpaceGeometryTangents.componentTagList(1).set("INTERPRETATION", "ty");
            worldSpaceGeometryTangents.componentTagList(2).set("INTERPRETATION", "tz");
        }
        if (bits & WorldSpaceMaterialNormals) {
            worldSpaceMaterialNormals = TGD::Array<float>({ width, height }, 3);
            worldSpaceMaterialNormals.componentTagList(0).set("INTERPRETATION", "nx");
            worldSpaceMaterialNormals.componentTagList(1).set("INTERPRETATION", "ny");
            worldSpaceMaterialNormals.componentTagList(2).set("INTERPRETATION", "nz");
        }
        if (bits & WorldSpaceMaterialTangents) {
            worldSpaceMaterialTangents = TGD::Array<float>({ width, height }, 3);
            worldSpaceMaterialTangents.componentTagList(0).set("INTERPRETATION", "tx");
            worldSpaceMaterialTangents.componentTagList(1).set("INTERPRETATION", "ty");
            worldSpaceMaterialTangents.componentTagList(2).set("INTERPRETATION", "tz");
        }
        // Camera Space Geometry
        if (bits & CameraSpacePositions) {
            cameraSpacePositions = TGD::Array<float>({ width, height }, 3);
            cameraSpacePositions.componentTagList(0).set("INTERPRETATION", "x");
            cameraSpacePositions.componentTagList(1).set("INTERPRETATION", "y");
            cameraSpacePositions.componentTagList(2).set("INTERPRETATION", "z");
        }
        if (bits & CameraSpaceGeometryNormals) {
            cameraSpaceGeometryNormals = TGD::Array<float>({ width, height }, 3);
            cameraSpaceGeometryNormals.componentTagList(0).set("INTERPRETATION", "nx");
            cameraSpaceGeometryNormals.componentTagList(1).set("INTERPRETATION", "ny");
            cameraSpaceGeometryNormals.componentTagList(2).set("INTERPRETATION", "nz");
        }
        if (bits & CameraSpaceGeometryTangents) {
            cameraSpaceGeometryTangents = TGD::Array<float>({ width, height }, 3);
            cameraSpaceGeometryTangents.componentTagList(0).set("INTERPRETATION", "tx");
            cameraSpaceGeometryTangents.componentTagList(1).set("INTERPRETATION", "ty");
            cameraSpaceGeometryTangents.componentTagList(2).set("INTERPRETATION", "tz");
        }
        if (bits & CameraSpaceMaterialNormals) {
            cameraSpaceMaterialNormals = TGD::Array<float>({ width, height }, 3);
            cameraSpaceMaterialNormals.componentTagList(0).set("INTERPRETATION", "nx");
            cameraSpaceMaterialNormals.componentTagList(1).set("INTERPRETATION", "ny");
            cameraSpaceMaterialNormals.componentTagList(2).set("INTERPRETATION", "nz");
        }
        if (bits & CameraSpaceMaterialTangents) {
            cameraSpaceMaterialTangents = TGD::Array<float>({ width, height }, 3);
            cameraSpaceMaterialTangents.componentTagList(0).set("INTERPRETATION", "tx");
            cameraSpaceMaterialTangents.componentTagList(1).set("INTERPRETATION", "ty");
            cameraSpaceMaterialTangents.componentTagList(2).set("INTERPRETATION", "tz");
        }
        if (bits & CameraSpaceDepths) {
            cameraSpaceDepths = TGD::Array<float>({ width, height }, 1);
            cameraSpaceDepths.componentTagList(0).set("INTERPRETATION", "depth");
        }
        if (bits & CameraSpaceDistances) {
            cameraSpaceDistances = TGD::Array<float>({ width, height }, 1);
            cameraSpaceDistances.componentTagList(0).set("INTERPRETATION", "distance");
        }
        // Texture Coordinates
        if (bits & TexCoords) {
            texCoords = TGD::Array<float>({ width, height }, 2);
            texCoords.componentTagList(0).set("INTERPRETATION", "u");
            texCoords.componentTagList(1).set("INTERPRETATION", "v");
        }
        // Backward and forward flow information
        if (bits & WorldSpaceOffsetToPrev) {
            worldSpaceOffsetToPrev = TGD::Array<float>({ width, height }, 3);
            worldSpaceOffsetToPrev.componentTagList(0).set("INTERPRETATION", "dx");
            worldSpaceOffsetToPrev.componentTagList(1).set("INTERPRETATION", "dy");
            worldSpaceOffsetToPrev.componentTagList(2).set("INTERPRETATION", "dz");
        }
        if (bits & WorldSpaceOffsetToNext) {
            worldSpaceOffsetToNext = TGD::Array<float>({ width, height }, 3);
            worldSpaceOffsetToNext.componentTagList(0).set("INTERPRETATION", "dx");
            worldSpaceOffsetToNext.componentTagList(1).set("INTERPRETATION", "dy");
            worldSpaceOffsetToNext.componentTagList(2).set("INTERPRETATION", "dz");
        }
        if (bits & CameraSpaceOffsetToPrev) {
            cameraSpaceOffsetToPrev = TGD::Array<float>({ width, height }, 3);
            cameraSpaceOffsetToPrev.componentTagList(0).set("INTERPRETATION", "dx");
            cameraSpaceOffsetToPrev.componentTagList(1).set("INTERPRETATION", "dy");
            cameraSpaceOffsetToPrev.componentTagList(2).set("INTERPRETATION", "dz");
        }
        if (bits & CameraSpaceOffsetToNext) {
            cameraSpaceOffsetToNext = TGD::Array<float>({ width, height }, 3);
            cameraSpaceOffsetToNext.componentTagList(0).set("INTERPRETATION", "dx");
            cameraSpaceOffsetToNext.componentTagList(1).set("INTERPRETATION", "dy");
            cameraSpaceOffsetToNext.componentTagList(2).set("INTERPRETATION", "dz");
        }
        if (bits & PixelSpaceOffsetToPrev) {
            pixelSpaceOffsetToPrev = TGD::Array<float>({ width, height }, 2);
            pixelSpaceOffsetToPrev.componentTagList(0).set("INTERPRETATION", "dx");
            pixelSpaceOffsetToPrev.componentTagList(1).set("INTERPRETATION", "dy");
        }
        if (bits & PixelSpaceOffsetToNext) {
            pixelSpaceOffsetToNext = TGD::Array<float>({ width, height }, 2);
            pixelSpaceOffsetToNext.componentTagList(0).set("INTERPRETATION", "dx");
            pixelSpaceOffsetToNext.componentTagList(1).set("INTERPRETATION", "dy");
        }
        // Other
        if (bits & Materials) {
            materials = TGD::Array<int>({ width, height }, 1);
            materials.componentTagList(0).set("INTERPRETATION", "material");
        }
    }
};

inline GroundTruth getGroundTruth(const Sensor& sensor, const Camera& camera, const Scene& scene,
        float t0, float tPrev, float tNext,
        unsigned int groundTruthBits = GroundTruth::All,
        const Parameters& params = Parameters())
{
    assert(tPrev <= t0);
    assert(t0 <= tNext);
    // Make sure the BVH is up to date for t0
    assert(!scene.bvhNeedsUpdate(t0, t0));

    unsigned int width = sensor.width();
    unsigned int height = sensor.height();
    unsigned int pixels = width * height;
    const Camera::RayHelper cameraRayHelper = camera.getRayHelper(t0, width, height);
    const Camera::ImageSpaceHelper imageSpaceHelper = camera.getImageSpaceHelper(width, height);

    AnimationCache animationCacheT0(scene.animations(), t0);
    Transformation cameraTransformationT0 = camera.at(t0);
    Transformation inverseCameraTransformationT0 = inverse(cameraTransformationT0);
    AnimationCache animationCacheTPrev(scene.animations(), tPrev);
    Transformation cameraTransformationTPrev = camera.at(tPrev);
    Transformation inverseCameraTransformationTPrev = inverse(cameraTransformationTPrev);
    AnimationCache animationCacheTNext(scene.animations(), tNext);
    Transformation cameraTransformationTNext = camera.at(tNext);
    Transformation inverseCameraTransformationTNext = inverse(cameraTransformationTNext);

    GroundTruth gt(width, height, groundTruthBits);

    fprintf(stderr, "Getting ground truth for %ux%u pixels at %.3fs... ", width, height, t0);
    float invWidth = 1.0f / width;
    float invHeight = 1.0f / height;
    // Trivially parallel loop over pixels
    #pragma omp parallel for schedule(dynamic)
    for (unsigned int pixel = 0; pixel < pixels; pixel++) {
        unsigned int y = pixel / width;
        unsigned int x = pixel % width;
        Prng prng(pixel);
        vec2 pixelCoord = vec2((x + 0.5f) * invWidth, (y + 0.5f) * invHeight);
        Ray ray = camera.getRay(pixelCoord.x(), pixelCoord.y(), t0, t0, cameraRayHelper,
                prng, false /* disable randomness */);
        const HitRecord hr = scene.bvh().hit(ray, RayIntersectionHelper(ray),
                params.minHitDistance, maxval, params.minHitDistance,
                animationCacheT0, prng);
        vec3 wsPos(0.0f), wsGNrm(0.0f), wsGTan(0.0f), wsMNrm(0.0f), wsMTan(0.0f);
        vec3 csPos(0.0f), csGNrm(0.0f), csGTan(0.0f), csMNrm(0.0f), csMTan(0.0f);
        float csDepth = 0.0f, csDist = 0.0f;
        vec2 txCor(0.0f);
        vec3 wsOP(0.0f), wsON(0.0f), csOP(0.0f), csON(0.0f);
        vec2 psOP(0.0f), psON(0.0f);
        int matInd = -1;
        if (hr.haveHit) {
            // World Space Geometry
            wsPos = hr.position;
            wsGNrm = hr.normal;
            wsGTan = hr.tangent;
            TangentSpace ts = hr.hitable->material()->tangentSpaceAt(hr, t0);
            wsMNrm = ts.normal;
            wsMTan = ts.tangent;
            // Camera Space Geometry
            csPos = inverseCameraTransformationT0.rotation * wsPos + inverseCameraTransformationT0.translation;
            csGNrm = inverseCameraTransformationT0.rotation * wsGNrm;
            csGTan = inverseCameraTransformationT0.rotation * wsGTan;
            csMNrm = inverseCameraTransformationT0.rotation * wsMNrm;
            csMTan = inverseCameraTransformationT0.rotation * wsMTan;
            csDepth = -csPos.z();
            csDist = length(csPos);
            // Texture Coordinates
            txCor = hr.texcoords;
            // Backward and forward flow information
            vec3 wsPosPrev = wsPos;
            vec3 wsPosNext = wsPos;
            if (hr.hitable->animationIndex() >= 0) {
                vec3 posOrig = inverse(animationCacheT0.get(hr.hitable->animationIndex())) * wsPos;
                wsPosPrev = animationCacheTPrev.get(hr.hitable->animationIndex()) * posOrig;
                wsPosNext = animationCacheTNext.get(hr.hitable->animationIndex()) * posOrig;
            }
            wsOP = wsPosPrev - wsPos;
            wsON = wsPosNext - wsPos;
            vec3 csPosPrev = inverseCameraTransformationTPrev.rotation * wsPosPrev + inverseCameraTransformationTPrev.translation;
            vec3 csPosNext = inverseCameraTransformationTNext.rotation * wsPosNext + inverseCameraTransformationTNext.translation;
            csOP = csPosPrev - csPos;
            csON = csPosNext - csPos;
            vec2 psPos = pixelCoord * vec2(width, height);
            assert(length(psPos - camera.cameraSpaceToImageSpace(csPos, imageSpaceHelper) * vec2(width, height)) < 0.001f);
            vec2 psPosPrev = camera.cameraSpaceToImageSpace(csPosPrev, imageSpaceHelper) * vec2(width, height);
            vec2 psPosNext = camera.cameraSpaceToImageSpace(csPosNext, imageSpaceHelper) * vec2(width, height);
            psOP = psPosPrev - psPos;
            psON = psPosNext - psPos;
            // Other
            matInd = scene.materialIndex(hr.hitable->material());
        }
        if (gt.bits &  GroundTruth::WorldSpacePositions)
            gt.worldSpacePositions.set({ x, y }, { wsPos.x(), wsPos.y(), wsPos.z() });
        if (gt.bits &  GroundTruth::WorldSpaceGeometryNormals)
            gt.worldSpaceGeometryNormals.set({ x, y }, { wsGNrm.x(), wsGNrm.y(), wsGNrm.z() });
        if (gt.bits &  GroundTruth::WorldSpaceGeometryTangents)
            gt.worldSpaceGeometryTangents.set({ x, y }, { wsGTan.x(), wsGTan.y(), wsGTan.z() });
        if (gt.bits &  GroundTruth::WorldSpaceMaterialNormals)
            gt.worldSpaceMaterialNormals.set({ x, y }, { wsMNrm.x(), wsMNrm.y(), wsMNrm.z() });
        if (gt.bits &  GroundTruth::WorldSpaceMaterialTangents)
            gt.worldSpaceMaterialTangents.set({ x, y }, { wsMTan.x(), wsMTan.y(), wsMTan.z() });
        if (gt.bits &  GroundTruth::CameraSpacePositions)
            gt.cameraSpacePositions.set({ x, y }, { csPos.x(), csPos.y(), csPos.z() });
        if (gt.bits &  GroundTruth::CameraSpaceGeometryNormals)
            gt.cameraSpaceGeometryNormals.set({ x, y }, { csGNrm.x(), csGNrm.y(), csGNrm.z() });
        if (gt.bits &  GroundTruth::CameraSpaceGeometryTangents)
            gt.cameraSpaceGeometryTangents.set({ x, y }, { csGTan.x(), csGTan.y(), csGTan.z() });
        if (gt.bits &  GroundTruth::CameraSpaceMaterialNormals)
            gt.cameraSpaceMaterialNormals.set({ x, y }, { csMNrm.x(), csMNrm.y(), csMNrm.z() });
        if (gt.bits &  GroundTruth::CameraSpaceMaterialTangents)
            gt.cameraSpaceMaterialTangents.set({ x, y }, { csMTan.x(), csMTan.y(), csMTan.z() });
        if (gt.bits &  GroundTruth::CameraSpaceDepths)
            gt.cameraSpaceDepths.set({ x, y }, { csDepth });
        if (gt.bits &  GroundTruth::CameraSpaceDistances)
            gt.cameraSpaceDistances.set({ x, y }, { csDist });
        if (gt.bits &  GroundTruth::TexCoords)
            gt.texCoords.set({ x, y }, { txCor.s(), txCor.t() });
        if (gt.bits &  GroundTruth::WorldSpaceOffsetToPrev)
            gt.worldSpaceOffsetToPrev.set({ x, y }, { wsOP.x(), wsOP.y(), wsOP.z() });
        if (gt.bits &  GroundTruth::WorldSpaceOffsetToNext)
            gt.worldSpaceOffsetToNext.set({ x, y }, { wsON.x(), wsON.y(), wsON.z() });
        if (gt.bits &  GroundTruth::CameraSpaceOffsetToPrev)
            gt.cameraSpaceOffsetToPrev.set({ x, y }, { csOP.x(), csOP.y(), csOP.z() });
        if (gt.bits &  GroundTruth::CameraSpaceOffsetToNext)
            gt.cameraSpaceOffsetToNext.set({ x, y }, { csON.x(), csON.y(), csON.z() });
        if (gt.bits &  GroundTruth::PixelSpaceOffsetToPrev)
            gt.pixelSpaceOffsetToPrev.set({ x, y }, { psOP.x(), psOP.y() });
        if (gt.bits &  GroundTruth::PixelSpaceOffsetToNext)
            gt.pixelSpaceOffsetToNext.set({ x, y }, { psON.x(), psON.y() });
        if (gt.bits &  GroundTruth::Materials)
            gt.materials.set({ x, y }, { matInd });
    }
    fprintf(stderr, "done\n");

    return gt;
}

inline GroundTruth getGroundTruth(const Sensor& sensor, const Camera& camera, const Scene& scene,
        float t0 = 0.0f,
        unsigned int groundTruthBits = GroundTruth::All,
        const Parameters& params = Parameters())
{
    return getGroundTruth(sensor, camera, scene, t0, t0, t0, groundTruthBits, params);
}

}
