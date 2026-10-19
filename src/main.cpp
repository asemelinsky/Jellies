// «Желейки» — soft-body jelly sandbox.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>

#include "audio.h"
#include "eyes.h"
#include "physics.h"
#include "render.h"
#include "shapes.h"
#include "ui.h"

namespace {

const Vector3 kRainbow[7] = {
    {0.95f, 0.10f, 0.14f},  // червоний
    {1.00f, 0.48f, 0.06f},  // помаранчевий
    {1.00f, 0.84f, 0.08f},  // жовтий
    {0.18f, 0.82f, 0.22f},  // зелений
    {0.22f, 0.74f, 1.00f},  // блакитний
    {0.14f, 0.28f, 0.95f},  // синій
    {0.58f, 0.18f, 0.90f},  // фіолетовий
};

const float kSpawnInterval = 10.0f;
const float kDayLength = 120.0f;  // seconds for a full sun cycle at time speed 1
const float kMaxThrowSpeed = 14.0f;
const float kKillY = -26.0f;

struct OrbitCamera {
    float yaw = 0.6f, pitch = 0.5f, dist = 19.0f;
    Vector3 target = {0, 0.6f, 0};
    Camera3D cam = {};

    void Update() {
        cam.target = target;
        cam.position = {target.x + dist * std::cos(pitch) * std::sin(yaw), target.y + dist * std::sin(pitch),
                        target.z + dist * std::cos(pitch) * std::cos(yaw)};
        cam.up = {0, 1, 0};
        cam.fovy = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;
    }
    Vector3 Forward() const { return Norm(cam.target - cam.position); }
};

struct App {
    std::vector<ShapeTemplate> templates;
    World world{&templates};
    Renderer renderer;
    OrbitCamera orbit;

    bool paused = false;
    bool showPanel = true;
    bool vsync = true;
    float timeScale = 1.0f;
    float softness = 0.5f;
    float spawnTimer = 0.0f;
    float dayPhase = 0.07f;
    float simTime = 0.0f;
    float physicsMs = 0.0f;

    // Grabbing
    int hovered = -1;
    float grabPlaneDist = 0;
    std::deque<std::pair<double, Vector3>> grabHistory;

    void Spawn();
    void Reset();
    void HandleInput();
    void UpdateGrab();
    void Simulate(float dt);
    void DrawUI();
};

void App::Spawn() {
    const int shape = GetRandomValue(0, SHAPE_COUNT - 1);
    const float scale = RandF(0.6f, 1.5f);
    const Vector3 color = kRainbow[GetRandomValue(0, 6)];
    const float L = world.params.platformHalf;
    const float g = std::max(world.params.gravity, 1.0f);

    Vector3 pos, vel;
    const int mode = GetRandomValue(0, 2);  // 0 = from above, 1 = from the left, 2 = from the right
    if (mode == 0) {
        pos = {RandF(-L * 0.6f, L * 0.6f), RandF(9.0f, 13.0f), RandF(-L * 0.6f, L * 0.6f)};
        vel = {0, -1.5f, 0};
    } else {
        // Left/right as seen from the current camera
        Vector3 fwd = orbit.Forward();
        fwd.y = 0;
        fwd = Norm(fwd);
        Vector3 right = Cross(fwd, Vector3{0, 1, 0});
        float side = mode == 1 ? -1.0f : 1.0f;
        pos = right * (side * (L + 6.0f)) + fwd * RandF(-3.0f, 3.0f);
        pos.y = RandF(3.0f, 5.5f);
        Vector3 land = {RandF(-L * 0.45f, L * 0.45f), 0.6f, RandF(-L * 0.45f, L * 0.45f)};
        // Ballistic arc that lands on the platform after T seconds
        const float T = RandF(1.0f, 1.3f);
        vel = (land - pos) * (1.0f / T);
        vel.y += 0.5f * g * T;
    }
    int b = world.AddBody(shape, scale, color, pos, QRandom(), vel);
    renderer.CreateBodyMesh(world.bodies[b], templates[shape]);
}

void App::Reset() {
    for (Body& b : world.bodies) renderer.DestroyBodyMesh(b);
    world.Clear();
    hovered = -1;
    grabHistory.clear();
    spawnTimer = 0;
    Spawn();
}

void App::HandleInput() {
    if (IsKeyPressed(KEY_SPACE)) {
        int n = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 10 : 1;
        for (int i = 0; i < n; i++) Spawn();
    }
    if (IsKeyPressed(KEY_P)) paused = !paused;
    if (IsKeyPressed(KEY_R)) Reset();
    if (IsKeyPressed(KEY_H)) showPanel = !showPanel;
    if (IsKeyPressed(KEY_M)) audio::SetMuted(!audio::IsMuted());
    if (IsKeyPressed(KEY_F11) || (IsKeyPressed(KEY_ENTER) && (IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT))))
        ToggleBorderlessWindowed();
    if (IsKeyPressed(KEY_ESCAPE) && IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE)) ToggleBorderlessWindowed();
    if (IsKeyPressed(KEY_V)) {
        vsync = !vsync;
        if (vsync) {
            SetWindowState(FLAG_VSYNC_HINT);
            SetTargetFPS(0);
        } else {
            ClearWindowState(FLAG_VSYNC_HINT);
            SetTargetFPS(0);
        }
    }

    const bool uiMouse = ui::WantsMouse();

    // Camera: right mouse rotates, wheel zooms
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !uiMouse) {
        Vector2 d = GetMouseDelta();
        orbit.yaw -= d.x * 0.006f;
        orbit.pitch += d.y * 0.006f;
        orbit.pitch = std::clamp(orbit.pitch, 0.08f, 1.45f);
    }
    float wheel = GetMouseWheelMove();
    if (wheel != 0 && !uiMouse) orbit.dist = std::clamp(orbit.dist * std::pow(0.88f, wheel), 5.0f, 45.0f);
    orbit.Update();

    // Hover / grab
    Ray ray = GetScreenToWorldRay(GetMousePosition(), orbit.cam);
    hovered = -1;
    if (world.GrabbedBody() < 0 && !uiMouse && !paused) {
        float t = 0;
        int particle = -1;
        int hit = world.Raycast(ray, &t, &particle);
        hovered = hit;
        if (hit >= 0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector3 p = ray.position + Norm(ray.direction) * t;
            grabPlaneDist = Dot(p - orbit.cam.position, orbit.Forward());
            world.BeginGrab(hit, particle, p);
            grabHistory.clear();
            grabHistory.push_back({GetTime(), p});
            audio::PlaySquelch(0.3f);
        }
    }
    if (world.GrabbedBody() >= 0) UpdateGrab();
}

void App::UpdateGrab() {
    Ray ray = GetScreenToWorldRay(GetMousePosition(), orbit.cam);
    Vector3 fwd = orbit.Forward();
    Vector3 planePoint = orbit.cam.position + fwd * grabPlaneDist;
    Vector3 dir = Norm(ray.direction);
    float denom = Dot(dir, fwd);
    Vector3 target = grabHistory.empty() ? planePoint : grabHistory.back().second;
    if (std::fabs(denom) > 1e-4f) target = ray.position + dir * (Dot(planePoint - ray.position, fwd) / denom);

    // Keep the target in a sane region and above the platform surface
    const float L = world.params.platformHalf;
    target.y = std::clamp(target.y, -8.0f, 20.0f);
    target.x = std::clamp(target.x, -L - 10, L + 10);
    target.z = std::clamp(target.z, -L - 10, L + 10);
    if (std::fabs(target.x) < L && std::fabs(target.z) < L) target.y = std::max(target.y, world.params.platformTop + 0.25f);

    world.SetGrabTarget(target);
    double now = GetTime();
    grabHistory.push_back({now, target});
    while (grabHistory.size() > 2 && now - grabHistory.front().first > 0.09) grabHistory.pop_front();

    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || paused) {
        Vector3 vel = {0, 0, 0};
        double span = grabHistory.back().first - grabHistory.front().first;
        if (span > 1e-3) vel = (grabHistory.back().second - grabHistory.front().second) * (float)(1.0 / span);
        world.EndGrab(vel, kMaxThrowSpeed);
        grabHistory.clear();
    }
}

void App::Simulate(float frameDt) {
    if (paused) return;
    const float dt = std::min(frameDt, 1.0f / 30.0f) * timeScale;
    world.params.softness = softness;
    const int substeps = std::clamp((int)std::ceil(dt / (1.0f / 300.0f)), 2, 16);
    double t0 = GetTime();
    world.Step(dt, substeps);
    physicsMs = physicsMs * 0.9f + (float)(GetTime() - t0) * 1000.0f * 0.1f;
    simTime += dt;

    // Remove jellies that fell off the platform
    for (int i = (int)world.bodies.size() - 1; i >= 0; i--) {
        if (world.bodies[i].center.y < kKillY) {
            renderer.DestroyBodyMesh(world.bodies[i]);
            world.RemoveBody(i);
        }
    }

    // Sounds
    for (Body& b : world.bodies) {
        if (b.soundCooldown > 0) continue;
        if (b.impactGround > 2.0f) {
            audio::PlaySplat(std::clamp((b.impactGround - 2.0f) / 9.0f, 0.12f, 1.0f) * std::min(1.0f, b.scale));
            b.soundCooldown = 0.25f;
        } else if (b.impactJelly > 1.6f) {
            audio::PlaySquelch(std::clamp((b.impactJelly - 1.6f) / 8.0f, 0.1f, 1.0f));
            b.soundCooldown = 0.3f;
        }
    }

    spawnTimer += frameDt;
    if (spawnTimer >= kSpawnInterval) {
        spawnTimer -= kSpawnInterval;
        Spawn();
    }

    dayPhase += dt / kDayLength;
    dayPhase -= std::floor(dayPhase);
}

void App::DrawUI() {
    char buf[128];
    const Color text = {245, 245, 250, 255};
    const Color dim = {245, 245, 250, 170};

    // FPS counter (always visible)
    std::snprintf(buf, sizeof buf, "FPS %d  ·  %.1f мс  ·  фізика %.1f мс", GetFPS(), GetFrameTime() * 1000.0f, physicsMs);
    Vector2 m = ui::Measure(buf, 17);
    ui::Label(buf, Vector2{GetScreenWidth() - m.x - 14, 10}, 17, text);

    if (!showPanel) {
        ui::Label("H — показати панель", Vector2{14, 10}, 16, dim);
        return;
    }

    const float x = 14, w = 300;
    ui::Panel(Rectangle{x - 4, 8, w + 28, 356});
    float y = 18;
    ui::Label("Желейки", Vector2{x + 10, y}, 28, text);
    y += 42;

    if (ui::Button(Rectangle{x + 10, y, 120, 34}, paused ? "Продовжити" : "Пауза", paused)) paused = !paused;
    if (ui::Button(Rectangle{x + 140, y, 170, 34}, "Скинути сцену")) Reset();
    y += 50;

    std::snprintf(buf, sizeof buf, "%.1f×", timeScale);
    ui::Slider(Rectangle{x + 10, y, w, 44}, "Швидкість часу", buf, &timeScale, 0.1f, 2.5f);
    y += 52;
    std::snprintf(buf, sizeof buf, "%.1f м/с²", world.params.gravity);
    ui::Slider(Rectangle{x + 10, y, w, 44}, "Гравітація", buf, &world.params.gravity, 0.0f, 30.0f);
    y += 52;
    std::snprintf(buf, sizeof buf, "%d%%", (int)std::round(softness * 100));
    ui::Slider(Rectangle{x + 10, y, w, 44}, "М’якість желе", buf, &softness, 0.0f, 1.0f);
    y += 56;

    std::snprintf(buf, sizeof buf, "Тіні: %s", ShadowQualityName(renderer.GetShadowQuality()));
    if (ui::Button(Rectangle{x + 10, y, 190, 32}, buf))
        renderer.SetShadowQuality((renderer.GetShadowQuality() + 1) % SHADOW_QUALITY_COUNT);
    if (ui::Button(Rectangle{x + 210, y, 100, 32}, "+ Желейка")) Spawn();
    y += 44;

    std::snprintf(buf, sizeof buf, "Желейок: %d    нова через %d с", (int)world.bodies.size(),
                  (int)std::ceil(kSpawnInterval - spawnTimer));
    ui::Label(buf, Vector2{x + 10, y}, 17, dim);

    const char* help1 = "ЛКМ — схопити й кинути  ·  ПКМ — камера  ·  Коліщатко — зум  ·  Пробіл — желейка (Shift — 10)";
    const char* help2 = "P — пауза  ·  R — скинути  ·  H — панель  ·  M — звук  ·  V — верт. синхр.  ·  F11 — повний екран";
    ui::Label(help1, Vector2{14, GetScreenHeight() - 50.0f}, 16, dim);
    ui::Label(help2, Vector2{14, GetScreenHeight() - 28.0f}, 16, dim);
}

}  // namespace

int main(int argc, char** argv) {
    bool fullscreen = false;
    for (int i = 1; i < argc; i++)
        if (std::strcmp(argv[i], "--fullscreen") == 0) fullscreen = true;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(1280, 720, "Желейки");
    SetExitKey(KEY_NULL);
    SetWindowMinSize(640, 400);
    if (fullscreen) ToggleBorderlessWindowed();

    static App app;
    BuildShapeTemplates(app.templates);
    eyes::Init(app.templates);
    if (!app.renderer.Init(GetScreenWidth(), GetScreenHeight())) {
        CloseWindow();
        return 1;
    }
    ui::Init();
    audio::Init();
    SetRandomSeed((unsigned)(GetTime() * 1e6) ^ 0x5eed);
    app.orbit.Update();
    app.Reset();

    int lastW = GetScreenWidth(), lastH = GetScreenHeight();
    while (!WindowShouldClose()) {
        const float frameDt = GetFrameTime();
        if (GetScreenWidth() != lastW || GetScreenHeight() != lastH) {
            lastW = GetScreenWidth();
            lastH = GetScreenHeight();
            if (lastW > 0 && lastH > 0) app.renderer.Resize(lastW, lastH);
        }

        ui::BeginFrame();
        app.HandleInput();
        app.Simulate(frameDt);
        eyes::Update(app.world, frameDt);
        audio::Update(frameDt);
        app.renderer.UpdateBodyMeshes(app.world);
        SkyState sky = ComputeSky(app.dayPhase);

        BeginDrawing();
        app.renderer.Render(app.orbit.cam, app.world, sky, app.hovered, app.world.GrabbedBody(), app.simTime);
        app.DrawUI();
        EndDrawing();
    }

    for (Body& b : app.world.bodies) app.renderer.DestroyBodyMesh(b);
    audio::Shutdown();
    eyes::Shutdown();
    ui::Shutdown();
    app.renderer.Shutdown();
    CloseWindow();
    return 0;
}
