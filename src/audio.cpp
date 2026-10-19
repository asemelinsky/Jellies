#include "audio.h"

#include <cmath>
#include <cstdint>
#include <vector>
#include "raylib.h"

namespace audio {
namespace {

constexpr int kRate = 44100;
constexpr int kVariants = 3;
constexpr int kAliases = 6;
constexpr float kMasterVolume = 0.35f;
constexpr float kMinGap = 0.045f;     // seconds between sounds of one type
constexpr int kMaxPerSecond = 12;

struct Bank {
    Sound sources[kVariants] = {};
    Sound aliases[kVariants][kAliases] = {};
    int next[kVariants] = {};
    float sinceLast = 1.0f;
    bool loaded = false;
};

bool gReady = false;
bool gMuted = false;
float gClock = 0.0f;
Bank gSplat, gSquelch;
std::vector<float> gRecent;  // start times of sounds played in the last second
uint32_t gRng = 0x9E3779B9u;

// xorshift32, kept separate from raylib's RNG so sounds don't disturb spawning
float Rand01(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return (s & 0xFFFFFF) / float(0xFFFFFF);
}
float RandNoise(uint32_t& s) { return Rand01(s) * 2.0f - 1.0f; }

Wave MakeWave(std::vector<float> samples) {
    // Short fade-out against clicks, then normalise to a fixed peak
    const int fade = kRate / 100;
    const int n = (int)samples.size();
    for (int i = 0; i < fade && i < n; i++) samples[n - 1 - i] *= i / float(fade);
    float peak = 1e-6f;
    for (float s : samples) peak = std::fmax(peak, std::fabs(s));
    const float gain = 0.9f / peak;

    Wave w = {};
    w.frameCount = (unsigned int)n;
    w.sampleRate = kRate;
    w.sampleSize = 16;
    w.channels = 1;
    short* data = (short*)RL_MALLOC(n * sizeof(short));
    for (int i = 0; i < n; i++) data[i] = (short)std::lround(samples[i] * gain * 32767.0f);
    w.data = data;
    return w;
}

// Soft wet slap: filtered noise burst plus a low thump sweeping down
std::vector<float> SynthSplat(uint32_t seed) {
    const float dur = 0.18f;
    const int n = (int)(dur * kRate);
    std::vector<float> out(n);
    uint32_t s = seed;
    float lp = 0.0f, lp2 = 0.0f, phase = 0.0f;
    const float thumpBase = 55.0f + 15.0f * Rand01(s);
    const float noiseDecay = 22.0f + 6.0f * Rand01(s);
    for (int i = 0; i < n; i++) {
        float t = i / float(kRate);
        float attack = std::fmin(1.0f, t / 0.002f);

        // Noise through a two-pole lowpass whose cutoff falls from ~3 kHz to ~400 Hz
        float fc = 400.0f + 2600.0f * std::exp(-t * 30.0f);
        float a = 1.0f - std::exp(-2.0f * PI * fc / kRate);
        lp += a * (RandNoise(s) - lp);
        lp2 += a * (lp - lp2);
        float noise = lp2 * attack * std::exp(-t * noiseDecay);

        float f = thumpBase + 85.0f * std::exp(-t * 18.0f);
        phase += 2.0f * PI * f / kRate;
        float thump = std::sin(phase) * std::fmin(1.0f, t / 0.003f) * std::exp(-t * 22.0f);

        out[i] = 1.6f * noise + 0.8f * thump;
    }
    return out;
}

// Sticky "chvak": wobbling low tone plus band-passed noise under a bell envelope
std::vector<float> SynthSquelch(uint32_t seed) {
    const float dur = 0.25f;
    const int n = (int)(dur * kRate);
    std::vector<float> out(n);
    uint32_t s = seed;
    float phase = 0.0f;
    float low = 0.0f, band = 0.0f;  // Chamberlin state-variable filter
    const float wobbleRate = 18.0f + 10.0f * Rand01(s);
    const float wobblePhase = 2.0f * PI * Rand01(s);
    const float startF = 290.0f + 60.0f * Rand01(s);
    for (int i = 0; i < n; i++) {
        float t = i / float(kRate);
        float u = t / dur;
        float env = std::pow(u, 0.4f) * std::pow(1.0f - u, 1.6f) * 3.2f;

        float f = 140.0f + (startF - 140.0f) * std::pow(1.0f - u, 1.3f);
        f *= 1.0f + 0.12f * std::sin(2.0f * PI * wobbleRate * t + wobblePhase);
        phase += 2.0f * PI * f / kRate;
        float tone = std::sin(phase) + 0.35f * std::sin(2.0f * phase + 0.5f) + 0.15f * std::sin(3.0f * phase);

        float fc = std::fmin(2.0f * f + 250.0f, 2500.0f);
        float k = 2.0f * std::sin(PI * fc / kRate);
        float q = 0.45f;
        float high = RandNoise(s) - low - q * band;
        band += k * high;
        low += k * band;

        float wet = 0.7f + 0.3f * std::sin(2.0f * PI * 37.0f * t);
        out[i] = env * wet * (0.7f * tone + 0.35f * band);
    }
    return out;
}

void LoadBank(Bank& bank, std::vector<float> (*synth)(uint32_t), uint32_t seedBase) {
    for (int v = 0; v < kVariants; v++) {
        Wave w = MakeWave(synth(seedBase + 7919u * (v + 1)));
        bank.sources[v] = LoadSoundFromWave(w);
        UnloadWave(w);
        for (int a = 0; a < kAliases; a++) bank.aliases[v][a] = LoadSoundAlias(bank.sources[v]);
        bank.next[v] = 0;
    }
    bank.sinceLast = 1.0f;
    bank.loaded = true;
}

void UnloadBank(Bank& bank) {
    if (!bank.loaded) return;
    for (int v = 0; v < kVariants; v++) {
        for (int a = 0; a < kAliases; a++) {
            StopSound(bank.aliases[v][a]);
            UnloadSoundAlias(bank.aliases[v][a]);
        }
    }
    for (int v = 0; v < kVariants; v++) UnloadSound(bank.sources[v]);
    bank = Bank{};
}

void Play(Bank& bank, float strength, float typeVolume) {
    if (!gReady || gMuted || !bank.loaded) return;
    if (strength > 1.0f) strength = 1.0f;
    if (strength <= 0.01f) return;
    if (bank.sinceLast < kMinGap) return;
    const int recent = (int)gRecent.size();
    if (recent > kMaxPerSecond) return;

    float vol = kMasterVolume * typeVolume * (0.25f + 0.75f * strength) / (1.0f + 0.35f * recent);
    int v = (int)(Rand01(gRng) * kVariants);
    if (v >= kVariants) v = kVariants - 1;
    Sound& snd = bank.aliases[v][bank.next[v]];
    bank.next[v] = (bank.next[v] + 1) % kAliases;
    SetSoundVolume(snd, vol);
    SetSoundPitch(snd, 0.85f + 0.35f * Rand01(gRng));
    PlaySound(snd);

    bank.sinceLast = 0.0f;
    gRecent.push_back(gClock);
}

}  // namespace

bool Init() {
    if (gReady) return true;
    InitAudioDevice();
    if (!IsAudioDeviceReady()) return false;
    LoadBank(gSplat, SynthSplat, 1234u);
    LoadBank(gSquelch, SynthSquelch, 98765u);
    gRecent.clear();
    gClock = 0.0f;
    gReady = true;
    return true;
}

void Shutdown() {
    if (!gReady) return;
    UnloadBank(gSplat);
    UnloadBank(gSquelch);
    CloseAudioDevice();
    gReady = false;
}

void Update(float dt) {
    gClock += dt;
    gSplat.sinceLast += dt;
    gSquelch.sinceLast += dt;
    size_t keep = 0;
    for (float t : gRecent)
        if (gClock - t < 1.0f) gRecent[keep++] = t;
    gRecent.resize(keep);
}

void PlaySplat(float strength) { Play(gSplat, strength, 1.0f); }

void PlaySquelch(float strength) { Play(gSquelch, strength, 0.8f); }

void SetMuted(bool muted) { gMuted = muted; }

bool IsMuted() { return gMuted; }

}  // namespace audio
