// «Кошик» game mode: throw jellies into a basket to score points.
#pragma once
#include "physics.h"
#include "render.h"

namespace basket {

void Init(World& world);   // creates meshes and registers the collider
void Shutdown(World& world);
void SetEnabled(bool on);  // toggles the game mode (basket, score, timer)
bool Enabled();
void Reset();              // scene reset: score and round restart
void Update(World& world, float dt);  // scoring, basket movement
void AppendSceneObjects(std::vector<SceneObject>& out);
void SetCamera(const Camera3D& cam);  // for score pop-ups in screen space
void DrawHUD();            // 2D overlay (score, time, record); after the UI panel

}  // namespace basket
