#pragma once

// -----------------------------------------------------------
// DSP Processor - handles all audio signal processing
// - 3-band EQ (bass/mid/treble shelving)
// - Crossover split-ear mode
// - Bass boost
// - Goertzel frequency analysis
// -----------------------------------------------------------

#include <stdint.h>
#include <math.h>
#include "biquad.h"
#include "goertzel.h"
#include "fast_math.h"
#include "../config/app_config.h"

class DSPProcessor {
public:
    DSPProcessor();

    // Initialize/update for given sample rate
    void init(uint32_t sampleRate);
    void setSampleRate(uint32_t sampleRate);
    void updateSampleRate(uint32_t sampleRate) { setSampleRate(sampleRate); }
    void resetAllFilters();

    // EQ settings
    void setEQ(float bassDB, float midDB, float trebleDB);
    void setEQ(int8_t bassDB, int8_t midDB, int8_t trebleDB, uint32_t sampleRate) {
        if (sampleRate != m_sampleRate) setSampleRate(sampleRate);
        setEQ((float)bassDB, (float)midDB, (float)trebleDB);
    }
    float getBassDB() const { return m_eqBassDB; }
    float getMidDB() const { return m_eqMidDB; }
    float getTrebleDB() const { return m_eqTrebleDB; }

    // Control flags
    void setBassBoost(bool enable) { m_bassBoostEnabled = enable; }
    void setChannelFlip(bool enable) { m_channelFlipEnabled = enable; }
    void setBypass(bool enable) { m_bypassEnabled = enable; }
    void setAnalysisEnabled(bool enable) { m_analysisEnabled = enable; }

    bool isBassBoostEnabled() const { return m_bassBoostEnabled; }
    bool isChannelFlipEnabled() const { return m_channelFlipEnabled; }
    bool isBypassEnabled() const { return m_bypassEnabled; }
    bool isAnalysisEnabled() const { return m_analysisEnabled; }

    // Process a single stereo sample (in-place)
    void processStereo(float &L, float &R);

    // Get Goertzel levels (legacy - keeping for beat detection)
    float getDB(int band) const { return m_goertzel.getDB(band); }
    float getLin(int band) const { return m_goertzel.getLin(band); }
    void zeroLevels() { m_goertzel.zeroLevels(); m_peakMeter.zero(); }
    
    // Named accessors for Goertzel bands (used for beat detection)
    float getGoertzel30dB() const { return m_goertzel.getDB(0); }
    float getGoertzel60dB() const { return m_goertzel.getDB(1); }
    float getGoertzel100dB() const { return m_goertzel.getDB(2); }
    float getGoertzel30Lin() const { return m_goertzel.getLin(0); }
    float getGoertzel60Lin() const { return m_goertzel.getLin(1); }
    float getGoertzel100Lin() const { return m_goertzel.getLin(2); }

    // Fast peak meter levels (for BLE level display) - much more responsive
    float getPeakDB(int band) const { return m_peakMeter.getDB(band); }
    float getPeakLin(int band) const { return m_peakMeter.getLin(band); }

    // Get control byte for BLE
    uint8_t getControlByte() const;
    void applyControlByte(uint8_t v);

private:
    void updateFilters();
    void updateLPAlpha();

    // -----------------------------------------------------------
    // Soft Clipper (replaces compressor for better sound quality)
    // - No envelope tracking = instant response, zero latency effect
    // - Tanh-style soft clipping = gentle saturation instead of hard clip
    // - Much lower CPU usage than compressor
    // - Preserves transients and dynamics better
    // -----------------------------------------------------------
    struct SoftClipper {
        float threshold = 0.85f;     // Start soft-clipping above this (linear)
        float ceiling = 1.0f;        // Maximum output level
        
        void init(float /* sampleRate */) {
            // No state to initialize - pure memoryless function
        }
        
        // Soft clip a single sample using cubic soft-clip curve
        // Below threshold: linear pass-through
        // Above threshold: smooth saturation curve approaching ceiling
        inline float softClip(float x) {
            float sign = x >= 0.0f ? 1.0f : -1.0f;
            float absX = x * sign;
            
            if (absX <= threshold) {
                return x;  // Below threshold: pass through
            }
            
            // Cubic soft-clip: smooth transition from threshold to ceiling
            // y = threshold + (ceiling - threshold) * tanh((x - threshold) / (ceiling - threshold))
            // Simplified approximation using rational function (faster than tanh)
            float excess = absX - threshold;
            float headroom = ceiling - threshold;
            float t = excess / headroom;  // Normalized excess [0, inf)
            
            // Approximation: t / (1 + t) maps [0,inf) -> [0,1)
            // This gives smooth saturation curve
            float compressed = headroom * t / (1.0f + t);
            
            return sign * (threshold + compressed);
        }
        
        // Process stereo pair
        void process(float &L, float &R) {
            L = softClip(L);
            R = softClip(R);
        }
    };
    
    SoftClipper m_clipper;

    // -----------------------------------------------------------
    // 3-Band Peak Meter for 30Hz, 60Hz, 100Hz display
    // - 3 LP filters at different cutoffs to capture each sub-bass band
    // - Instant attack, smooth release for punchy visual response
    // - Ultra-lightweight: just 3 cascaded 1-pole LP filters
    // -----------------------------------------------------------
    struct PeakMeter {
        static constexpr int NUM_BANDS = 3;  // 30Hz, 60Hz, 100Hz
        
        // Release envelope
        float releaseCoef = 0.0f;
        float releaseCoefInv = 0.0f;
        
        // LP filter for 30Hz band (~45Hz cutoff)
        float lp30State = 0.0f;
        float lp30Coef = 0.0f;
        float lp30CoefInv = 0.0f;
        float lp30Envelope = 0.0f;
        float lp30PeakLin = 0.0f;
        
        // LP filter for 60Hz band (~80Hz cutoff)
        float lp60State = 0.0f;
        float lp60Coef = 0.0f;
        float lp60CoefInv = 0.0f;
        float lp60Envelope = 0.0f;
        float lp60PeakLin = 0.0f;
        
        // LP filter for 100Hz band (~120Hz cutoff)
        float lp100State = 0.0f;
        float lp100Coef = 0.0f;
        float lp100CoefInv = 0.0f;
        float lp100Envelope = 0.0f;
        float lp100PeakLin = 0.0f;
        
        // Pre-computed constant for dB conversion
        static constexpr float DB_SCALE = 8.685889638f;  // 20/ln(10)
        
        void init(float sampleRate) {
            if (sampleRate <= 0) sampleRate = 44100.0f;
            
            // Release time ~50ms for smooth response
            float invSampleRate = fast_recipsf2(sampleRate);
            float releaseMs = 50.0f;
            float releaseSamples = releaseMs * 0.001f * sampleRate;
            releaseCoef = expf(-fast_recipsf2(releaseSamples));
            releaseCoefInv = 1.0f - releaseCoef;
            
            float dt = invSampleRate;
            
            // LP for 30Hz band (~45Hz cutoff)
            float fc30 = 45.0f;
            float rc30 = fast_recipsf2(2.0f * DSP_PI_F * fc30);
            lp30Coef = dt * fast_recipsf2(rc30 + dt);
            lp30CoefInv = 1.0f - lp30Coef;
            
            // LP for 60Hz band (~80Hz cutoff)
            float fc60 = 80.0f;
            float rc60 = fast_recipsf2(2.0f * DSP_PI_F * fc60);
            lp60Coef = dt * fast_recipsf2(rc60 + dt);
            lp60CoefInv = 1.0f - lp60Coef;
            
            // LP for 100Hz band (~120Hz cutoff)
            float fc100 = 120.0f;
            float rc100 = fast_recipsf2(2.0f * DSP_PI_F * fc100);
            lp100Coef = dt * fast_recipsf2(rc100 + dt);
            lp100CoefInv = 1.0f - lp100Coef;
            
            zero();
        }
        
        // Process with 3 LP filters for 30Hz, 60Hz, 100Hz bands
        inline void process(float x) {
            // LP for 30Hz band (~45Hz cutoff)
            lp30State = lp30CoefInv * lp30State + lp30Coef * x;
            float lp30Abs = lp30State >= 0.0f ? lp30State : -lp30State;
            if (lp30Abs > lp30Envelope) {
                lp30Envelope = lp30Abs;
            } else {
                lp30Envelope = releaseCoef * lp30Envelope + releaseCoefInv * lp30Abs;
            }
            lp30PeakLin = lp30Envelope;
            
            // LP for 60Hz band (~80Hz cutoff)
            lp60State = lp60CoefInv * lp60State + lp60Coef * x;
            float lp60Abs = lp60State >= 0.0f ? lp60State : -lp60State;
            if (lp60Abs > lp60Envelope) {
                lp60Envelope = lp60Abs;
            } else {
                lp60Envelope = releaseCoef * lp60Envelope + releaseCoefInv * lp60Abs;
            }
            lp60PeakLin = lp60Envelope;
            
            // LP for 100Hz band (~120Hz cutoff)
            lp100State = lp100CoefInv * lp100State + lp100Coef * x;
            float lp100Abs = lp100State >= 0.0f ? lp100State : -lp100State;
            if (lp100Abs > lp100Envelope) {
                lp100Envelope = lp100Abs;
            } else {
                lp100Envelope = releaseCoef * lp100Envelope + releaseCoefInv * lp100Abs;
            }
            lp100PeakLin = lp100Envelope;
        }
        
        // Get dB for band: 0=30Hz, 1=60Hz, 2=100Hz
        float getDB(int band) const {
            float lin;
            if (band == 0) lin = lp30PeakLin;       // 30Hz (LP ~45Hz)
            else if (band == 1) lin = lp60PeakLin;  // 60Hz (LP ~80Hz)
            else lin = lp100PeakLin;                 // 100Hz (LP ~120Hz)
            
            if (lin < 1e-6f) return -60.0f;
            float dB = DB_SCALE * fast_logf(lin);
            if (dB < -60.0f) return -60.0f;
            if (dB > 0.0f) return 0.0f;
            return dB;
        }
        
        float getLin(int band) const {
            if (band == 0) return lp30PeakLin;
            if (band == 1) return lp60PeakLin;
            return lp100PeakLin;
        }
        
        void zero() {
            lp30State = 0.0f;
            lp30Envelope = 0.0f;
            lp30PeakLin = 0.0f;
            lp60State = 0.0f;
            lp60Envelope = 0.0f;
            lp60PeakLin = 0.0f;
            lp100State = 0.0f;
            lp100Envelope = 0.0f;
            lp100PeakLin = 0.0f;
        }
    };
    
    PeakMeter m_peakMeter;

    // Sample rate
    uint32_t m_sampleRate;

    // EQ filters
    Biquad m_eqBassL, m_eqBassR;
    Biquad m_eqMidL, m_eqMidR;
    Biquad m_eqTrebleL, m_eqTrebleR;

    // Bass boost shelf filters (separate for L/R stereo)
    Biquad m_bassShelfL, m_bassShelfR;

    // Crossover filters
    Biquad m_crossoverLPL, m_crossoverHPR;

    // Goertzel frequency analysis
    GoertzelBank m_goertzel;

    // EQ gains
    float m_eqBassDB;
    float m_eqMidDB;
    float m_eqTrebleDB;
    bool m_eqActive;

    // 1-pole low-pass state
    float m_lpAlpha;
    float m_lpState;

    // Control flags
    bool m_bassBoostEnabled;
    bool m_channelFlipEnabled;
    bool m_bypassEnabled;
    bool m_analysisEnabled;
};

// -----------------------------------------------------------
// Implementation
// -----------------------------------------------------------

inline DSPProcessor::DSPProcessor() 
    : m_sampleRate(APP_I2S_DEFAULT_SR)
    , m_eqBassDB(0.0f)
    , m_eqMidDB(0.0f)
    , m_eqTrebleDB(0.0f)
    , m_eqActive(false)
    , m_lpAlpha(0.0f)
    , m_lpState(0.0f)
    , m_bassBoostEnabled(false)
    , m_channelFlipEnabled(false)
    , m_bypassEnabled(false)
    , m_analysisEnabled(true)
{
}

inline void DSPProcessor::init(uint32_t sampleRate) {
    m_sampleRate = sampleRate > 0 ? sampleRate : APP_I2S_DEFAULT_SR;
    updateFilters();
    updateLPAlpha();
    m_goertzel.init(m_sampleRate);
    m_clipper.init((float)m_sampleRate);
    m_peakMeter.init((float)m_sampleRate);
}

inline void DSPProcessor::setSampleRate(uint32_t sampleRate) {
    if (sampleRate == m_sampleRate && sampleRate != 0) return;
    m_sampleRate = sampleRate > 0 ? sampleRate : APP_I2S_DEFAULT_SR;
    updateFilters();
    updateLPAlpha();
    resetAllFilters();  // Clear all filter states to prevent noise on codec switch
    m_goertzel.init(m_sampleRate);
    m_clipper.init((float)m_sampleRate);
    m_peakMeter.init((float)m_sampleRate);
    m_lpState = 0.0f;  // Reset filter state
}

inline void DSPProcessor::setEQ(float bassDB, float midDB, float trebleDB) {
    // Scale input range from ±12dB (phone) to actual audio range
    // Bass scaled more conservatively to prevent clipping
    // Mid/treble can be more aggressive as they clip less
    m_eqBassDB = bassDB * 0.5f;
    m_eqMidDB = midDB * 0.7f;
    m_eqTrebleDB = trebleDB * 0.7f;
    updateFilters();
}

inline void DSPProcessor::updateFilters() {
    if (m_sampleRate == 0) return;
    float fs = (float)m_sampleRate;

    m_eqBassL.makeLowShelf(fs, 150.0f, m_eqBassDB);
    m_eqBassR.makeLowShelf(fs, 150.0f, m_eqBassDB);
    m_eqMidL.makePeakingEQ(fs, 1000.0f, 1.0f, m_eqMidDB);
    m_eqMidR.makePeakingEQ(fs, 1000.0f, 1.0f, m_eqMidDB);
    m_eqTrebleL.makeHighShelf(fs, 6000.0f, m_eqTrebleDB);
    m_eqTrebleR.makeHighShelf(fs, 6000.0f, m_eqTrebleDB);

    // Bass boost shelf (+2 dB at 150 Hz)
    m_bassShelfL.makeLowShelf(fs, 150.0f, 2.0f);
    m_bassShelfR.makeLowShelf(fs, 150.0f, 2.0f);

    // Crossover filters
    m_crossoverLPL.makeLowPass(fs, APP_CROSSOVER_LP_FREQ);
    m_crossoverHPR.makeHighPass(fs, APP_CROSSOVER_HP_FREQ);

    // Cache EQ active state
    m_eqActive = (fabsf(m_eqBassDB) >= 0.1f) ||
                 (fabsf(m_eqMidDB) >= 0.1f) ||
                 (fabsf(m_eqTrebleDB) >= 0.1f);
}

inline void DSPProcessor::resetAllFilters() {
    // Reset all biquad filter states to prevent noise when sample rate changes
    m_eqBassL.reset();
    m_eqBassR.reset();
    m_eqMidL.reset();
    m_eqMidR.reset();
    m_eqTrebleL.reset();
    m_eqTrebleR.reset();
    m_bassShelfL.reset();
    m_bassShelfR.reset();
    m_crossoverLPL.reset();
    m_crossoverHPR.reset();
    m_peakMeter.zero();
    m_lpState = 0.0f;
}

inline void DSPProcessor::updateLPAlpha() {
    const float fc = 300.0f;

    if (m_sampleRate == 0) {
        m_lpAlpha = 0.0f;
        return;
    }

    float dt = fast_recipsf2((float)m_sampleRate);
    float rc = fast_recipsf2(2.0f * DSP_PI_F * fc);
    float alpha = fast_div(dt, rc + dt);

    if (alpha < 0.001f) alpha = 0.001f;
    if (alpha > 0.999f) alpha = 0.999f;

    m_lpAlpha = alpha;
}

inline void DSPProcessor::processStereo(float &L, float &R) {
    // Create mono mix for analysis (before any processing)
    float mono = (L + R) * 0.5f;

    // EQ always applies (regardless of bypass)
    if (m_eqActive) {
        L = m_eqBassL.process(L);
        R = m_eqBassR.process(R);
        L = m_eqMidL.process(L);
        R = m_eqMidR.process(R);
        L = m_eqTrebleL.process(L);
        R = m_eqTrebleR.process(R);
    }

    // Audio analysis (using original audio before DSP)
    if (m_analysisEnabled) {
        m_goertzel.processSample(mono);  // For beat detection
        m_peakMeter.process(mono);       // For level display (fast response)
    }

    // Crossover split-ear mode only when NOT bypassed
    // Stereo preserved: LP applied to L channel, HP applied to R channel
    if (!m_bypassEnabled) {
        // Apply LP filter to LEFT channel (keeps stereo L content, low frequencies only)
        float lpOut = m_crossoverLPL.process(L);
        // Apply HP filter to RIGHT channel (keeps stereo R content, high frequencies only)
        float hpOut = m_crossoverHPR.process(R);

        // Apply bass boost shelf to LP channel
        if (m_bassBoostEnabled) {
            lpOut = m_bassShelfL.process(lpOut) * DSP_BASS_GAIN_BOOST;
        }

        // Gain compensation: each ear only gets part of the spectrum
        constexpr float CROSSOVER_GAIN = 1.41f;
        lpOut *= CROSSOVER_GAIN;
        hpOut *= CROSSOVER_GAIN;

        // Channel flip controls which ear gets LP vs HP
        if (!m_channelFlipEnabled) {
            L = lpOut;  // Left ear: low frequencies from L channel
            R = hpOut;  // Right ear: high frequencies from R channel
        } else {
            L = hpOut;  // Left ear: high frequencies from R channel
            R = lpOut;  // Right ear: low frequencies from L channel
        }
    } else {
        // Bypass mode: full-range stereo with optional bass boost
        if (m_bassBoostEnabled) {
            // Apply bass shelf to each channel independently (keeps stereo)
            L = m_bassShelfL.process(L) * DSP_BASS_GAIN_BOOST;
            R = m_bassShelfR.process(R) * DSP_BASS_GAIN_BOOST;
        }
    }

    // Apply soft clipper to prevent harsh digital clipping
    m_clipper.process(L, R);
}

inline uint8_t DSPProcessor::getControlByte() const {
    uint8_t v = 0;
    if (m_bassBoostEnabled) v |= 0x01;
    if (m_channelFlipEnabled) v |= 0x02;
    if (m_bypassEnabled) v |= 0x04;
    return v;
}

inline void DSPProcessor::applyControlByte(uint8_t v) {
    m_bassBoostEnabled = (v & 0x01) != 0;
    m_channelFlipEnabled = (v & 0x02) != 0;
    m_bypassEnabled = (v & 0x04) != 0;
}
