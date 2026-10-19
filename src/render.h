// Rendering: sky with an automatic sun/moon cycle, platform and sea with soft
// PCF shadows, coloured translucent jelly shadows, and translucent jellies
// with screen-space refraction, Fresnel and fake subsurface glow.
#pragma once
#include "physics.h"

enum ShadowQuality { SHADOW_LOW, SHADOW_MEDIUM, SHADOW_HIGH, SHADOW_ULTRA, SHADOW_QUALITY_COUNT };
const char* ShadowQualityName(int q);  // UTF-8 Ukrainian label

struct SkyState {
    Vector3 sunDir;         // unit vector towards the sun
    Vector3 lightDir;       // unit vector towards the active light (sun by day, moon at night)
    Vector3 lightColor;     // colour * intensity of the active light
    Vector3 ambientSky;     // hemispherical ambient, upper half
    Vector3 ambientGround;  // hemispherical ambient, lower half
    Vector3 zenith;         // sky colour overhead
    Vector3 horizon;        // sky colour at the horizon
    float night;            // 0 = day, 1 = full night
};

// Static scene geometry (obstacles, basket, ...) drawn with shadows like the
// platform. Meshes are owned by whoever appends them; vertices in mesh space.
struct SceneObject {
    const Mesh* mesh = nullptr;
    Matrix transform = MatrixIdentity();
    Vector3 color = {0.8f, 0.8f, 0.8f};
    int pattern = 0;     // 0 = plain, 1 = wicker weave, 2 = diagonal stripes
    float gloss = 0.3f;  // 0 = matte, 1 = shiny
    bool castShadow = true;
    bool doubleSided = false;
};

// dayPhase in [0,1): 0 = sunrise, ~0.375 = noon, 0.75 = sunset, then night
SkyState ComputeSky(float dayPhase);

class Renderer {
public:
    bool Init(int width, int height);
    void Shutdown();
    void Resize(int width, int height);

    void SetShadowQuality(int q);
    int GetShadowQuality() const { return shadowQuality_; }

    void CreateBodyMesh(Body& b, const ShapeTemplate& st);
    void DestroyBodyMesh(Body& b);
    // Skin every body's render mesh from its particles and upload to the GPU
    void UpdateBodyMeshes(World& world);

    // Draws the whole 3D scene to the current framebuffer (call inside
    // BeginDrawing/EndDrawing, before the UI). hovered/grabbed are body
    // indices or -1.
    void Render(const Camera3D& cam, const World& world, const SkyState& sky, int hovered, int grabbed, float time);

    // Filled by the game every frame before Render()
    std::vector<SceneObject> sceneObjects;

private:
    struct Impl;
    Impl* impl_ = nullptr;
    int shadowQuality_ = SHADOW_HIGH;
};
