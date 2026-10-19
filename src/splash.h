// Jelly droplets flying out of strong impacts and glossy wet stains on the
// platform (impact puddles, droplet spots, sliding trails) that slowly fade.
#pragma once
#include "physics.h"

namespace splash {

void Init();
void Shutdown();
void Clear();                               // on scene reset
void Update(const World& world, float dt);  // after World::Step: spawn from impacts, simulate droplets, age stains

// Inside BeginMode3D in the opaque pass right after the platform. Stains lie
// on the platform top, which nothing opaque covers from above, so the depth
// test is switched off internally (no z-fighting) and restored afterwards.
void DrawStains(const Camera3D& cam, Vector3 lightDir, Vector3 lightColor, Vector3 ambient);
// Inside BeginMode3D in the transparent pass (after jellies), depth test on.
void DrawDroplets(const Camera3D& cam, Vector3 lightDir, Vector3 lightColor, Vector3 ambient);

}  // namespace splash
