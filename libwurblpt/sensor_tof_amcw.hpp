/*
 * Copyright (C) 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020, 2021, 2022
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

/* This simulates a Time-of-Flight depth sensor based on the Amplitude-Modulated
 * Continuous Wave (AMCW) principle.
 *
 * The simulation model is described in:
 * M. Lambers, S. Hoberg, and A. Kolb. "Simulation of Time-of-Flight Sensors for
 * Evaluation of Chip Layout Variants". In: IEEE Sensors Journal 15.7 (July 2015),
 * pp. 4019–4026. doi: 10.1109/JSEN.2015.2409816.
 *
 * This version uses global illumination.
 *
 * A simulation of the chips electronics including noise effects is missing; a good
 * starting point to implement this is:
 * Matthias J. Almer, "Time-of-Flight 3D-Camera Performance Modeling", Master's Thesis,
 * Graz University of Technology
 */

#pragma once

#include <random>

#include "sensor.hpp"
#include "material.hpp"


namespace WurblPT {

class SensorTofAmcw final : public Sensor
{
public:
    /* Fixed parameters */
    constexpr static float dutyCycle = 0.5f;

    /* Modifyable parameters. I did not bother with getters/setters, so these are simply public. */
    unsigned int phaseImageCount;// typically 4; currently needs to be a multiple of 4
    float wavelength;           // in nm
    double modulationFrequency; // in Hz
    float exposureTime;         // for each phase image, in microseconds
    float readoutTime;          // for each phase image, in microsedonds
    float pauseTime;            // after all phase image acquisitions, in microseconds
    float pixelArea;            // in micrometers squared; pixel side length is sometimes called pixel pitch
    float contrast;             // achievable pixel contrast in [0,1]
    float quantumEfficiency;    //
    int maxElectrons;           // max number of electrons per pixel before saturation

private:
    /* Internal members */
    int _phaseImageIndex;
    TGD::Array<float> _frame;

public:
    /* Public interface for the application */
    SensorTofAmcw(unsigned int width, unsigned int height) :
        phaseImageCount(4),
        wavelength(880.0f),
        modulationFrequency(10e6),
        exposureTime(1000.0f),
        readoutTime(1000.0f),
        pauseTime(42000.0f),
        pixelArea(12.0f * 12.0f),
        contrast(0.75f),
        quantumEfficiency(0.8f),
        maxElectrons(100000),
        _phaseImageIndex(0),
        _frame({ width, height }, 3)
    {
        _frame = TGD::Array<float>({ width, height }, 3);
        _frame.componentTagList(0).set("INTERPRETATION", "a");
        _frame.componentTagList(1).set("INTERPRETATION", "b");
        _frame.componentTagList(2).set("INTERPRETATION", "total");
    }

    void setPhaseIndex(int i)
    {
        _phaseImageIndex = i;
    }

    float tau(unsigned int phaseImageIndex) const
    {
        return phaseImageIndex * (2.0f * pi) / phaseImageCount;
    }

    float fracModfreqC() const
    {
        return modulationFrequency / speedOfLight; // double precision computation is necessary
    }

    float fracCModfreq() const
    {
        return speedOfLight / modulationFrequency; // double precision computation is necessary
    }

    void setPauseTimeForFPS(float fps)
    {
        // all values in microseconds
        float totalFrameDuration = 1e6f / fps;
        float totalPhaseImageDurations = phaseImageCount * (exposureTime + readoutTime);
        pauseTime = totalFrameDuration - totalPhaseImageDurations;
    }

    float frameDuration() const
    {
        return (phaseImageCount * (exposureTime + readoutTime) + pauseTime) / 1e6f;
    }

    float fps() const
    {
        return 1.0f / frameDuration();
    }

    float phaseImageDuration() const // in seconds
    {
        return (exposureTime + readoutTime) / 1e6f;
    }

    /* The following functions can be called once the path tracer has stored some results into our internal _frame */

    const TGD::Array<float>& energy() const
    {
        return _frame;
    }

    TGD::Array<float> phase(float shotNoiseFactor, unsigned long long prngSeed = 42) const
    {
        std::mt19937_64 generator(prngSeed);
        std::normal_distribution<float> gaussianDistribution(0.0f, 1.0f);
        TGD::Array<float> phaseImage({ _frame.dimension(0), _frame.dimension(1) }, 4);
        phaseImage.globalTagList() = _frame.globalTagList();
        phaseImage.componentTagList(0).set("INTERPRETATION", "a-b");
        phaseImage.componentTagList(1).set("INTERPRETATION", "a+b");
        phaseImage.componentTagList(2).set("INTERPRETATION", "a");
        phaseImage.componentTagList(3).set("INTERPRETATION", "b");
        vec2 maxElectronsVec(maxElectrons, maxElectrons);
        for (size_t i = 0; i < phaseImage.elementCount(); i++) {
            vec2 energy = vec2(_frame.get<float>(i));
            // ([1e-9m] * [1e-21 J]) / [1e-25 Jm] = [1e-30]/[1e-25] = 1e-5 electrons
            vec2 electrons = quantumEfficiency * wavelength * energy / hc / 10000.0f;
            vec2 shotNoise = sqrt(electrons) * vec2(gaussianDistribution(generator), gaussianDistribution(generator)); // approximation of Poisson noise
            electrons += shotNoiseFactor * shotNoise;
            // transform electrons to digNums [0, 1]
            vec2 digNums = clamp(electrons, vec2(0.0f), maxElectronsVec) / maxElectronsVec;
            phaseImage.set<float>(i, { digNums.x() - digNums.y(), digNums.x() + digNums.y(), digNums.x(), digNums.y() });
        }
        return phaseImage;
    }

    TGD::Array<float> result(const TGD::Array<float>* phases) const
    {
        assert(phaseImageCount % 4 == 0);
        int tau0Index   = 0;
        int tau90Index  = 1 * phaseImageCount / 4;
        int tau180Index = 2 * phaseImageCount / 4;
        int tau270Index = 3 * phaseImageCount / 4;
        TGD::Array<float> r(phases[0].dimensions(), 4);
        r.globalTagList() = phases[0].globalTagList();
        std::string totalCpuTime;
        totalCpuTime  = phases[tau0Index  ].globalTagList().value("WurblPT/CPU_SECONDS", "unknown") + '+';
        totalCpuTime += phases[tau90Index ].globalTagList().value("WurblPT/CPU_SECONDS", "unknown") + '+';
        totalCpuTime += phases[tau180Index].globalTagList().value("WurblPT/CPU_SECONDS", "unknown") + '+';
        totalCpuTime += phases[tau270Index].globalTagList().value("WurblPT/CPU_SECONDS", "unknown");
        r.globalTagList().set("WurblPT/CPU_SECONDS", totalCpuTime);
        r.componentTagList(0).set("INTERPRETATION", "distance");
        r.componentTagList(1).set("INTERPRETATION", "amplitude");
        r.componentTagList(2).set("INTERPRETATION", "intensity");
        r.componentTagList(3).set("INTERPRETATION", "phase_shift");
        for (size_t i = 0; i < r.elementCount(); i++) {
            float D[4] = { // A-B from the phase images
                phases[tau0Index  ].get<float>(i, 0),
                phases[tau90Index ].get<float>(i, 0),
                phases[tau180Index].get<float>(i, 0),
                phases[tau270Index].get<float>(i, 0)
            };
            float phaseShift;
            float distance;
            if (abs(D[0] - D[2]) <= 0.0f && abs(D[1] - D[3]) <= 0.0f) {
                phaseShift = 0.0f;
                distance = 0.0f;
            } else {
                phaseShift = atan(D[3] - D[1], D[0] - D[2]);
                if (phaseShift < 0.0f)
                    phaseShift += 2.0 * pi;
                distance = fracCModfreq() * phaseShift * 0.25f * inv_pi;
            }
            float amplitude = sqrt((D[0] - D[2]) * (D[0] - D[2]) + (D[1] - D[3]) * (D[1] - D[3])) * pi_2;
            float intensity = 0.5f * (D[0] + D[1] + D[2] + D[3]);
            r.set<float>(i, { distance, amplitude, intensity, phaseShift });
        }
        return r;
    }

    /* Public interface used by the path tracer; the application does not need to call these */

    virtual unsigned int width() const override
    {
        return _frame.dimension(0);
    }

    virtual unsigned int height() const override
    {
        return _frame.dimension(1);
    }

    virtual void accumulateRadiance(
            const Ray& /* r */,
            unsigned int /* pathComponent */,
            float /* geometricPathLength */,
            const vec4& opticalPathLength,
            const vec4& radiance,
            const HitRecord& hitRecord,
            float /* t0 */, float /* t1 */,
            float* sampleAccumulator) const override
    {
        float irradiance = radiance.w() * 1000.0f; // [mW/m^2]
        float power = irradiance * pixelArea; // [(mw/m^2) * (1e-6m)^2] = [1e-3 * 1e-6 * 1e-6 W] = [1e-15 W] = Femtowatt
        float energy = power * dutyCycle * exposureTime; // [1e-15 W * 1e-6s] = [1e-21 J] = Zeptojoule
        float halfEnergy = 0.5f * energy;
        float t = 0.0f;
        if (hitRecord.haveHit && hitRecord.hitable->material()->isTofLight(hitRecord)) {
            float phaseShift = 2.0f * pi * opticalPathLength.w() * fracModfreqC();
            t = contrast * cos(tau(_phaseImageIndex) + phaseShift);
        }
        float energyA = halfEnergy * (1.0f + t);
        float energyB = halfEnergy * (1.0f - t);
        sampleAccumulator[0] += energyA;
        sampleAccumulator[1] += energyB;
        sampleAccumulator[2] += energy;
    }

    virtual void finishPixel(unsigned int pixelIndex, float invSamples, const float* sampleAccumulator) override
    {
        _frame[pixelIndex][0] = invSamples * sampleAccumulator[0];
        _frame[pixelIndex][1] = invSamples * sampleAccumulator[1];
        _frame[pixelIndex][2] = invSamples * sampleAccumulator[2];
    }

    virtual TGD::ArrayContainer* pixelArray() override
    {
        return &_frame;
    }
};

}
