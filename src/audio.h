// ============================================================================
//  audio.h  -  Procedural real-time Formula 1 sound synthesis system
//              Generates authentic V6 hybrid engine harmonics, turbo whine,
//              tyre screech, aerodynamic wind rush, and pit stop wheel guns
//              using the standard Windows Multimedia (waveOut) API.
// ============================================================================
#pragma once
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <mmsystem.h>
#include <atomic>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstdio>

struct AudioSystem {
    static constexpr int SAMPLE_RATE = 44100;
    static constexpr int NUM_CHANNELS = 2;
    static constexpr int BUFFER_SAMPLES = 1024;
    static constexpr int NUM_BUFFERS = 4;

    HWAVEOUT hWave = nullptr;
    WAVEHDR headers[NUM_BUFFERS] = {};
    int16_t bufferData[NUM_BUFFERS][BUFFER_SAMPLES * NUM_CHANNELS] = {};
    HANDLE hEvent = nullptr;

    std::thread audioThread;
    std::atomic<bool> running{false};
    std::atomic<bool> enabled{true};
    std::atomic<float> masterVolume{0.75f};

    // Game state inputs (updated from main thread)
    std::atomic<float> inRpm{0.25f};
    std::atomic<float> inThrottle{0.0f};
    std::atomic<float> inSpeed{0.0f};
    std::atomic<float> inSkid{0.0f};
    std::atomic<float> inShake{0.0f};
    std::atomic<int>   inPitStage{0};
    std::atomic<bool>  inPaused{false};

    // Internal synthesis phase accumulators (audio thread only)
    double enginePhase = 0.0;
    double turboPhase = 0.0;
    double tyrePhase = 0.0;
    double tyreMod = 0.0;
    double windPhase = 0.0;
    double gunPhase = 0.0;
    double gunImpactPhase = 0.0;
    double jackPhase = 0.0;

    // Filter states
    float noiseLfsr = 0.5f;
    float windFilter = 0.0f;
    float skidFilter = 0.0f;
    float bumpFilter = 0.0f;

    // Smoothed values
    float curRpm = 0.25f;
    float curThrottle = 0.0f;
    float curSpeed = 0.0f;
    float curSkid = 0.0f;
    float curBump = 0.0f;

    void init() {
        if (hWave) return;

        hEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (!hEvent) {
            fprintf(stderr, "[Audio] Failed to create audio event handle\n");
            return;
        }

        WAVEFORMATEX wfx = {};
        wfx.wFormatTag = WAVE_FORMAT_PCM;
        wfx.nChannels = NUM_CHANNELS;
        wfx.nSamplesPerSec = SAMPLE_RATE;
        wfx.wBitsPerSample = 16;
        wfx.nBlockAlign = (wfx.nChannels * wfx.wBitsPerSample) / 8;
        wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;
        wfx.cbSize = 0;

        MMRESULT res = waveOutOpen(&hWave, WAVE_MAPPER, &wfx, (DWORD_PTR)hEvent, 0, CALLBACK_EVENT);
        if (res != MMSYSERR_NOERROR) {
            fprintf(stderr, "[Audio] waveOutOpen failed (code %u) - running without audio\n", (unsigned)res);
            CloseHandle(hEvent);
            hEvent = nullptr;
            return;
        }

        for (int i = 0; i < NUM_BUFFERS; i++) {
            headers[i].lpData = (LPSTR)bufferData[i];
            headers[i].dwBufferLength = BUFFER_SAMPLES * NUM_CHANNELS * sizeof(int16_t);
            headers[i].dwFlags = 0;
            waveOutPrepareHeader(hWave, &headers[i], sizeof(WAVEHDR));
        }

        running = true;
        audioThread = std::thread(&AudioSystem::audioLoop, this);
        printf("[Audio] Procedural F1 Sound Engine initialized (44.1 kHz, 16-bit stereo)\n");
    }

    void shutdown() {
        if (!running) return;
        running = false;
        if (hEvent) SetEvent(hEvent);
        if (audioThread.joinable()) audioThread.join();

        if (hWave) {
            waveOutReset(hWave);
            for (int i = 0; i < NUM_BUFFERS; i++) {
                waveOutUnprepareHeader(hWave, &headers[i], sizeof(WAVEHDR));
            }
            waveOutClose(hWave);
            hWave = nullptr;
        }
        if (hEvent) {
            CloseHandle(hEvent);
            hEvent = nullptr;
        }
    }

    ~AudioSystem() {
        shutdown();
    }

    void update(float speed, float throttle, float brk, float latAcc, int ctrl, int stage, float shake, bool paused) {
        if (!running) return;
        // Compute engine RPM: 0.25 (idle) .. 1.0 (limiter)
        float absSpd = std::fabs(speed);
        float span = 11.5f;
        float rpm = std::clamp((std::fmod(absSpd, span) / span) * 0.5f + 0.5f + throttle * 0.05f, 0.0f, 1.0f);
        if (absSpd < 0.5f) rpm = 0.25f + throttle * 0.5f;
        if (ctrl == 0) rpm = 0.25f; // CTRL_GRID: idle on grid before race starts

        inRpm.store(rpm, std::memory_order_relaxed);
        inThrottle.store(throttle, std::memory_order_relaxed);
        inSpeed.store(speed, std::memory_order_relaxed);

        // Skid detection: lateral slip or hard braking lockup at speed
        float skid = 0.0f;
        if (absSpd > 5.0f) {
            float latG = std::fabs(latAcc);
            if (latG > 9.0f) skid += std::clamp((latG - 9.0f) / 14.0f, 0.0f, 1.0f);
            if (brk > 0.4f && absSpd > 12.0f) skid += std::clamp(brk * 0.8f, 0.0f, 1.0f);
        }
        inSkid.store(std::clamp(skid, 0.0f, 1.0f), std::memory_order_relaxed);
        inShake.store(shake, std::memory_order_relaxed);
        inPitStage.store(stage, std::memory_order_relaxed);
        inPaused.store(paused, std::memory_order_relaxed);
    }

    void toggleMute() {
        enabled = !enabled;
    }

    bool isMuted() const {
        return !enabled;
    }

private:
    float nextNoise() {
        // Fast linear congruential noise generator
        static uint32_t seed = 0x12345678;
        seed = seed * 1664525u + 1013904223u;
        return ((float)(seed & 0xFFFF) / 32767.5f) - 1.0f;
    }

    void audioLoop() {
        // Prime all buffers initially
        for (int i = 0; i < NUM_BUFFERS; i++) {
            fillBuffer(bufferData[i], BUFFER_SAMPLES);
            waveOutWrite(hWave, &headers[i], sizeof(WAVEHDR));
        }

        int curBuf = 0;
        while (running) {
            WaitForSingleObject(hEvent, 20);
            if (!running) break;

            while (headers[curBuf].dwFlags & WHDR_DONE) {
                fillBuffer(bufferData[curBuf], BUFFER_SAMPLES);
                headers[curBuf].dwFlags &= ~WHDR_DONE;
                waveOutWrite(hWave, &headers[curBuf], sizeof(WAVEHDR));
                curBuf = (curBuf + 1) % NUM_BUFFERS;
            }
        }
    }

    void fillBuffer(int16_t* buffer, int numSamples) {
        if (!enabled || inPaused) {
            std::fill(buffer, buffer + numSamples * NUM_CHANNELS, 0);
            return;
        }

        const float dt = 1.0f / (float)SAMPLE_RATE;
        const float targetR = inRpm.load(std::memory_order_relaxed);
        const float targetT = inThrottle.load(std::memory_order_relaxed);
        const float targetS = inSpeed.load(std::memory_order_relaxed);
        const float targetSk = inSkid.load(std::memory_order_relaxed);
        const float targetSh = inShake.load(std::memory_order_relaxed);
        const int   stage = inPitStage.load(std::memory_order_relaxed);
        const float vol = masterVolume.load(std::memory_order_relaxed);

        for (int i = 0; i < numSamples; i++) {
            // Smooth input parameters per sample
            curRpm += (targetR - curRpm) * 0.003f;
            curThrottle += (targetT - curThrottle) * 0.004f;
            curSpeed += (targetS - curSpeed) * 0.003f;
            curSkid += (targetSk - curSkid) * 0.005f;
            if (targetSh > curBump) curBump = targetSh;
            curBump = std::max(0.0f, curBump - 0.0003f);

            // ================================================================
            // 1. FORMULA 1 V6 TURBO HYBRID ENGINE SYNTHESIS
            // ================================================================
            // Crankshaft rotation frequency (65 Hz at 4000 RPM idle to 250 Hz at 15000 RPM)
            float revHz = 65.0f + curRpm * 185.0f;
            // 6-cylinder 4-stroke firing frequency (3 power strokes per revolution)
            float fireHz = revHz * 3.0f;

            enginePhase += fireHz * dt * 2.0 * 3.14159265358979323846;
            if (enginePhase > 2.0 * 3.14159265358979323846) {
                enginePhase -= 2.0 * 3.14159265358979323846;
            }

            // Synthesize rich harmonic series typical of high-rev racing exhausts
            float p = (float)enginePhase;
            float h1 = std::sin(p);
            float h2 = std::sin(p * 2.0f);
            float h3 = std::sin(p * 3.0f);
            float h4 = std::sin(p * 4.0f);
            float h5 = std::sin(p * 5.0f);
            float h6 = std::sin(p * 6.0f);
            float sub = std::sin(p * 0.5f);

            // Throttle modifies harmonic richness and raspy distortion
            float engineWave = 0.32f * h1 + 0.28f * h2 + 0.22f * h3 + 0.14f * h4 + 0.09f * h5 + 0.06f * h6 + 0.12f * sub;
            // Soft-saturation waveshaping creates metallic racing exhaust roar
            float drive = 1.3f + curThrottle * 1.7f;
            engineWave = std::tanh(engineWave * drive) * (0.35f + curThrottle * 0.35f + curRpm * 0.3f);

            // Off-throttle overrun burble
            if (curThrottle < 0.15f && curRpm > 0.45f) {
                float rumble = std::sin(p * 0.33f) * nextNoise() * 0.08f;
                engineWave += rumble;
            }

            // Turbocharger spool whine (high frequency whistle scaling with throttle)
            float turboHz = 2400.0f + curThrottle * 1900.0f + curRpm * 800.0f;
            turboPhase += turboHz * dt * 2.0 * 3.14159265358979323846;
            if (turboPhase > 2.0 * 3.14159265358979323846) turboPhase -= 2.0 * 3.14159265358979323846;
            float turboWave = std::sin((float)turboPhase) * (0.02f + curThrottle * 0.06f);

            // ================================================================
            // 2. TYRE SCREECH / SKID SYNTHESIS
            // ================================================================
            float tyreWave = 0.0f;
            if (curSkid > 0.02f) {
                tyreMod += 18.0 * dt * 2.0 * 3.14159265358979323846;
                float squealHz = 980.0f + 260.0f * std::sin((float)tyreMod);
                tyrePhase += squealHz * dt * 2.0 * 3.14159265358979323846;
                if (tyrePhase > 2.0 * 3.14159265358979323846) tyrePhase -= 2.0 * 3.14159265358979323846;

                float noise = nextNoise();
                skidFilter += (noise - skidFilter) * 0.35f;
                tyreWave = (std::sin((float)tyrePhase) * 0.5f + skidFilter * 0.5f) * curSkid * 0.35f;
            }

            // ================================================================
            // 3. AERODYNAMIC WIND RUSH
            // ================================================================
            float rawWind = nextNoise();
            windFilter += (rawWind - windFilter) * 0.08f;
            float windSpeed = std::clamp(std::fabs(curSpeed) / 85.0f, 0.0f, 1.0f);
            float windWave = windFilter * windSpeed * 0.18f;

            // ================================================================
            // 4. PIT STOP PROCEDURAL SOUND EFFECTS
            // ================================================================
            float pitWave = 0.0f;
            // Wheel gun active during tyre change (stage 6: PS_CHANGING)
            if (stage == 6) {
                // High-speed air gun whine (~1700 Hz) + rapid impact hammer (~62 Hz)
                gunPhase += 1680.0 * dt * 2.0 * 3.14159265358979323846;
                gunImpactPhase += 62.0 * dt * 2.0 * 3.14159265358979323846;
                float impact = std::max(0.0f, std::sin((float)gunImpactPhase));
                impact = impact * impact * impact; // sharp click peaks
                float airWhine = std::sin((float)gunPhase) * 0.25f + nextNoise() * 0.15f;
                pitWave = (airWhine + impact * 0.55f) * 0.42f;
            }
            // Trolley jack hiss / clunk during lifting (stage 4) and lowering (stage 8)
            else if (stage == 4 || stage == 8) {
                jackPhase += 420.0 * dt * 2.0 * 3.14159265358979323846;
                float hiss = nextNoise() * 0.12f + std::sin((float)jackPhase) * 0.06f;
                pitWave = hiss * 0.35f;
            }

            // ================================================================
            // 5. KERB / BARRIER COLLISION THUMP
            // ================================================================
            float bumpWave = 0.0f;
            if (curBump > 0.01f) {
                bumpFilter += (nextNoise() - bumpFilter) * 0.04f; // Low-frequency thud
                bumpWave = bumpFilter * curBump * 0.6f;
            }

            // ================================================================
            // MIX & MASTER TO STEREO OUTPUT
            // ================================================================
            float totalMono = engineWave * 0.55f + turboWave + tyreWave + windWave + pitWave + bumpWave;
            // Soft master limiting to prevent clipping
            float masterOut = std::tanh(totalMono * 1.15f) * vol;

            // Subtle stereo spatial distribution (pit gun on right, wind wide)
            float leftSample  = masterOut * 0.95f - pitWave * 0.1f + windWave * 0.1f;
            float rightSample = masterOut * 0.95f + pitWave * 0.1f - windWave * 0.1f;

            buffer[i * 2 + 0] = (int16_t)std::clamp(leftSample * 32767.0f, -32768.0f, 32767.0f);
            buffer[i * 2 + 1] = (int16_t)std::clamp(rightSample * 32767.0f, -32768.0f, 32767.0f);
        }
    }
};
