// Candy eyes for gummy bears and worms: skinned to the jelly particles like
// the render mesh, with pupils that lag behind the body, random blinking,
// a squint after hard landings and wide eyes while being carried.
#pragma once
#include <vector>
#include "physics.h"

namespace eyes {

void Init(const std::vector<ShapeTemplate>& templates);  // needs a GL context
void Shutdown();
void Update(const World& world, float dt);
// Call inside BeginMode3D after the jellies are drawn (depth test on)
void Draw(const World& world, const Camera3D& cam, Vector3 lightDir, Vector3 lightColor, Vector3 ambient);
void ForceBlink(int bodyId);

}  // namespace eyes
