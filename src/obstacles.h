// Obstacles on the platform: a trampoline and a ramp.
#pragma once
#include "physics.h"
#include "render.h"

namespace obstacles {

void Init(World& world);   // creates meshes and registers the collider
void Shutdown(World& world);
void SetEnabled(bool on);  // when off they neither collide nor draw
bool Enabled();
void Update(const World& world, float dt);  // animation (trampoline membrane)
void AppendSceneObjects(std::vector<SceneObject>& out);

}  // namespace obstacles
