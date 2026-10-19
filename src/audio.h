// Procedurally synthesised, quiet jelly sounds with rate limiting.
#pragma once

namespace audio {

bool Init();      // opens the audio device; safe to call when no device exists
void Shutdown();
void Update(float dt);  // advances cooldowns

// strength in [0,1]; quieter and throttled when many play close together
void PlaySplat(float strength);    // jelly hits the platform
void PlaySquelch(float strength);  // jelly-jelly contact, grabbing

void SetMuted(bool muted);
bool IsMuted();

}  // namespace audio
