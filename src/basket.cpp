#include "basket.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "audio.h"
#include "ui.h"

namespace basket {
namespace {

// Basket geometry in local units (multiplied by the pop-in animation scale)
const float kH = 0.85f;      // wall height
const float kRb = 0.92f;     // wall mid-surface radius at the bottom
const float kRt = 1.12f;     // wall mid-surface radius at the top (flared)
const float kT = 0.09f;      // wall thickness
const float kRim = 0.075f;   // rim tube radius
const float kFloor = 0.05f;  // floor thickness
const float kRoundTime = 60.0f;
const float kScoreDelay = 0.4f;   // seconds a jelly must stay inside
const float kThrowWindow = 2.9f;  // release -> score (2.5 s + the 0.4 s wait)
const int kMaxInside = 6;
const char* kRecordFile = "jellies_record.txt";

struct Track {
    bool seen = false;
    bool entered = false;  // crossed the rim plane downwards inside the opening
    bool scored = false;
    bool hasPrev = false;
    float prevY = 0;
    float insideTime = 0;
    float releaseTime = -100;  // sim time when the player let go of it
    Vector3 releasePos = {0, 0, 0};
    float ejectAt = -1;  // sim time to pop a scored jelly out of the basket
};

struct Popup {
    Vector3 pos;
    std::string text, detail;
    float age = 0;
};

struct State {
    bool enabled = false;
    bool roundOver = false;
    bool newRecord = false;
    int score = 0;
    int record = 0;
    float timeLeft = kRoundTime;
    float roundTime = 0;
    float clock = 0;  // simulation time while the mode is on
    Vector3 pos = {2.5f, 0, 3.0f};  // base centre on the platform (y unused)
    Vector3 nextPos = {2.5f, 0, 3.0f};
    float scale = 0;  // pop-in animation: 0 = hidden, 1 = full size
    int anim = 0;     // 0 = idle, 1 = shrinking (moving), 2 = growing
    float animT = 0;
    bool pendingMove = false;
    float crowdCooldown = 0;
    float emptyTime = 0;  // how long the basket has been empty
    int lastGrabbedId = -1;
    std::unordered_map<int, Track> tracks;
    std::vector<Popup> popups;
    Camera3D cam = {};
    bool hasCam = false;
    Mesh wall = {}, floor = {}, rim = {};
    bool meshes = false;
    float platformTop = 0, platformHalf = 7;
    World* world = nullptr;
} S;

float Clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
float Smooth(float t) {
    t = Clamp01(t);
    return t * t * (3 - 2 * t);
}

// --- collider -------------------------------------------------------------

void ProjectWithFriction(World& w, int i, Vector3 n, float pen, float mu) {
    Vector3& p = w.x[i];
    p += n * pen;
    Vector3 d = p - w.xprev[i];
    Vector3 t = d - n * Dot(d, n);
    float lt = Len(t);
    if (lt < 1e-9f) return;
    if (lt < mu * 1.3f * pen) p -= t;
    else p -= t * std::min(mu * pen / lt, 1.0f);
}

class BasketCollider : public Collider {
public:
    void Collide(World& w, float) override {
        if (!S.enabled || S.scale < 0.05f) return;
        const float s = S.scale;
        const Vector3 P = {S.pos.x, w.params.platformTop, S.pos.z};
        const float H = kH * s, rb = kRb * s, rt = kRt * s, ht = 0.5f * kT * s, rim = kRim * s, fl = kFloor * s;
        const float reach = rt + rim;
        // Wall mid-surface: segment (rb,0) -> (rt,H) in the (radius, height) half plane
        const float sx = rt - rb, sy = H, sl2 = sx * sx + sy * sy, sinv = 1.0f / std::sqrt(sl2);
        const int n = (int)w.x.size();
        for (int i = 0; i < n; i++) {
            const float pr = w.radius[i];
            Vector3 q = w.x[i] - P;
            if (q.y > H + rim + pr || q.y < -pr) continue;
            if (std::fabs(q.x) > reach + pr || std::fabs(q.z) > reach + pr) continue;
            float r = std::sqrt(q.x * q.x + q.z * q.z);
            if (r > reach + pr) continue;

            // Floor inside the basket
            if (r < rb - ht && q.y < fl + pr) {
                ProjectWithFriction(w, i, {0, 1, 0}, fl + pr - q.y, 0.6f);
                w.MarkGrounded(i);
                q = w.x[i] - P;
                r = std::sqrt(q.x * q.x + q.z * q.z);
            }
            const Vector3 er = r > 1e-6f ? Vector3{q.x / r, 0, q.z / r} : Vector3{1, 0, 0};

            // Where the particle was at the start of the substep: it must stay on
            // that side of the thin wall, so soft bodies drape over the wall
            // instead of tunnelling through it
            const Vector3 qp = w.xprev[i] - P;
            const float rp = std::sqrt(qp.x * qp.x + qp.z * qp.z);

            // Wall: a slanted shell of thickness kT (the rim covers its top end)
            {
                const float nx = sy * sinv, ny = -sx * sinv;  // outward normal of the shell
                float t = ((r - rb) * sx + q.y * sy) / sl2;
                if (t >= 0.0f && t <= 1.0f) {
                    float sd = (r - rb) * nx + q.y * ny;
                    float side = ((rp - rb) * nx + qp.y * ny) >= 0 ? 1.0f : -1.0f;
                    float pen = ht + pr - side * sd;
                    if (pen > 0) {
                        ProjectWithFriction(w, i, (er * nx + Vector3{0, ny, 0}) * side, pen, 0.4f);
                        q = w.x[i] - P;
                        r = std::sqrt(q.x * q.x + q.z * q.z);
                    }
                }
            }
            // Rim tube along the top edge
            {
                const float dx = r - rt, dy = q.y - H;
                const float d = std::sqrt(dx * dx + dy * dy);
                const float minD = rim + pr;
                if (d < minD) {
                    float nxr = 0, nyr = 1;
                    const float px = rp - rt, py = qp.y - H;
                    if (d > 1e-6f && dx * px + dy * py >= 0) {
                        nxr = dx / d;
                        nyr = dy / d;
                    } else {
                        // crossed the tube's centre this substep: push back the way it came
                        float pl = std::sqrt(px * px + py * py);
                        if (pl > 1e-6f) {
                            nxr = px / pl;
                            nyr = py / pl;
                        }
                    }
                    const float pen = minD - (dx * nxr + dy * nyr);
                    ProjectWithFriction(w, i, er * nxr + Vector3{0, nyr, 0}, pen, 0.3f);
                    if (nyr > 0.6f) w.MarkGrounded(i);
                }
            }
        }
    }
} gCollider;

// --- meshes ---------------------------------------------------------------

Mesh MakeMesh(const std::vector<Vector3>& P, const std::vector<Vector3>& N, const std::vector<unsigned short>& I) {
    Mesh m = {};
    m.vertexCount = (int)P.size();
    m.triangleCount = (int)I.size() / 3;
    m.vertices = (float*)RL_MALLOC(P.size() * 3 * sizeof(float));
    m.normals = (float*)RL_MALLOC(P.size() * 3 * sizeof(float));
    m.texcoords = (float*)RL_CALLOC(P.size() * 2, sizeof(float));
    m.indices = (unsigned short*)RL_MALLOC(I.size() * sizeof(unsigned short));
    for (size_t i = 0; i < P.size(); i++) {
        m.vertices[3 * i] = P[i].x;
        m.vertices[3 * i + 1] = P[i].y;
        m.vertices[3 * i + 2] = P[i].z;
        m.normals[3 * i] = N[i].x;
        m.normals[3 * i + 1] = N[i].y;
        m.normals[3 * i + 2] = N[i].z;
    }
    for (size_t i = 0; i < I.size(); i++) m.indices[i] = I[i];
    UploadMesh(&m, false);
    return m;
}

// Parametric grid surface; normals by finite differences, turned to face
// `facing(p)`, triangles wound to agree with them.
template <class F, class G>
void Grid(std::vector<Vector3>& P, std::vector<Vector3>& N, std::vector<unsigned short>& I, int nu, int nv, F f,
          G facing) {
    const int base = (int)P.size();
    const float e = 1e-3f;
    for (int i = 0; i <= nu; i++)
        for (int j = 0; j <= nv; j++) {
            float u = i / (float)nu, v = j / (float)nv;
            Vector3 p = f(u, v);
            Vector3 du = f(std::min(u + e, 1.0f), v) - f(std::max(u - e, 0.0f), v);
            Vector3 dv = f(u, v + e) - f(u, v - e);
            Vector3 nrm = Cross(du, dv);
            Vector3 face = facing(p);
            if (Len(nrm) < 1e-9f) nrm = face;
            nrm = Norm(nrm);
            if (Dot(nrm, face) < 0) nrm = nrm * -1.0f;
            P.push_back(p);
            N.push_back(nrm);
        }
    auto tri = [&](int a, int b, int c) {
        Vector3 gn = Cross(P[b] - P[a], P[c] - P[a]);
        if (Dot(gn, N[a] + N[b] + N[c]) < 0) std::swap(b, c);
        I.push_back((unsigned short)a);
        I.push_back((unsigned short)b);
        I.push_back((unsigned short)c);
    };
    for (int i = 0; i < nu; i++)
        for (int j = 0; j < nv; j++) {
            int a = base + i * (nv + 1) + j, b = a + nv + 1;
            tri(a, b, b + 1);
            tri(a, b + 1, a + 1);
        }
}

// Woven relief: horizontal bands going over/under vertical stakes
float Weave(float h, float th) {
    const int rows = 9, stakes = 18;
    float k = std::floor(h * rows);
    float fu = h * rows - k;
    float band = std::sqrt(std::max(std::sin(PI * fu), 0.0f));
    float over = std::cos(th * stakes + k * PI);
    return 0.014f * band * (0.55f + 0.45f * over);
}

void BuildMeshes() {
    const float TAU = 2.0f * PI;
    {
        std::vector<Vector3> P, N;
        std::vector<unsigned short> I;
        for (float side : {1.0f, -1.0f}) {
            auto f = [side](float u, float v) {
                float th = v * 2.0f * PI;
                float y0 = side > 0 ? 0.0f : kFloor;
                float y = y0 + (kH - y0) * u;
                float h = y / kH;
                float r = kRb + (kRt - kRb) * h + side * (0.5f * kT + Weave(h, th));
                return Vector3{r * std::cos(th), y, r * std::sin(th)};
            };
            auto facing = [side](Vector3 p) { return Norm(Vector3{p.x, 0, p.z}) * side; };
            Grid(P, N, I, 40, 144, f, facing);
        }
        S.wall = MakeMesh(P, N, I);
    }
    {
        std::vector<Vector3> P, N;
        std::vector<unsigned short> I;
        auto f = [TAU](float u, float v) {
            float th = v * TAU;
            float r = (kRb - 0.5f * kT) * u;
            float k = std::floor(u * 7.0f);
            float fu = u * 7.0f - k;
            float bump = 0.006f * std::sqrt(std::max(std::sin(PI * fu), 0.0f)) * (0.6f + 0.4f * std::cos(th * 16 + k * PI));
            return Vector3{r * std::cos(th), kFloor + bump, r * std::sin(th)};
        };
        Grid(P, N, I, 14, 144, f, [](Vector3) { return Vector3{0, 1, 0}; });
        S.floor = MakeMesh(P, N, I);
    }
    {
        std::vector<Vector3> P, N;
        std::vector<unsigned short> I;
        auto center = [](float th) { return Vector3{kRt * std::cos(th), kH, kRt * std::sin(th)}; };
        auto f = [TAU, center](float u, float v) {
            float phi = u * TAU, th = v * TAU;
            Vector3 er = {std::cos(th), 0, std::sin(th)};
            float rr = kRim * (1.0f + 0.13f * std::cos(3.0f * phi + 52.0f * th));  // twisted rope
            return center(th) + (er * std::cos(phi) + Vector3{0, std::sin(phi), 0}) * rr;
        };
        auto facing = [](Vector3 p) {
            float th = std::atan2(p.z, p.x);
            return p - Vector3{kRt * std::cos(th), kH, kRt * std::sin(th)};
        };
        Grid(P, N, I, 12, 208, f, facing);
        S.rim = MakeMesh(P, N, I);
    }
    S.meshes = true;
}

// --- game logic -------------------------------------------------------------

void LoadRecord() {
    if (FILE* fp = std::fopen(kRecordFile, "r")) {
        int v = 0;
        if (std::fscanf(fp, "%d", &v) == 1 && v > 0) S.record = v;
        std::fclose(fp);
    }
}

void SaveRecord() {
    if (FILE* fp = std::fopen(kRecordFile, "w")) {
        std::fprintf(fp, "%d\n", S.record);
        std::fclose(fp);
    }
}

void StartRound() {
    S.score = 0;
    S.timeLeft = kRoundTime;
    S.roundTime = 0;
    S.roundOver = false;
    S.newRecord = false;
    S.popups.clear();
}

void EndRound() {
    S.roundOver = true;
    if (S.score > S.record) {
        S.record = S.score;
        S.newRecord = true;
        SaveRecord();
    }
}

bool InsideBasket(Vector3 c, float s, float* outR = nullptr, float* outY = nullptr) {
    Vector3 q = c - Vector3{S.pos.x, S.platformTop, S.pos.z};
    float r = std::sqrt(q.x * q.x + q.z * q.z);
    if (outR) *outR = r;
    if (outY) *outY = q.y;
    if (s < 0.5f || q.y <= 0 || q.y >= kH * s) return false;
    float inner = (kRb + (kRt - kRb) * Clamp01(q.y / (kH * s)) - 0.5f * kT) * s;
    return r < inner;
}

void SetBodyVelocity(World& w, const Body& b, Vector3 vel) {
    Vector3 vm = {0, 0, 0};
    for (int i = b.first; i < b.first + b.count; i++) vm += w.v[i];
    vm = vm * (1.0f / b.count);
    Vector3 dv = vel - vm;
    for (int i = b.first; i < b.first + b.count; i++) w.v[i] += dv;
}

void Eject(World& w, const Body& b) {
    // Hop out towards the middle of the platform
    Vector3 dir = {-S.pos.x, 0, -S.pos.z};
    if (Len(dir) < 0.5f) dir = {1, 0, 0};
    dir = Norm(dir);
    const float g = std::max(w.params.gravity, 2.0f);
    const float vy = std::sqrt(2.0f * g * 1.7f) + 0.3f;
    SetBodyVelocity(w, b, dir * 2.4f + Vector3{0, vy, 0});
    audio::PlaySquelch(0.35f);
}

void Score(const Body& b, Track& t) {
    int pts = 1;
    std::string detail;
    auto add = [&detail](const char* s) {
        if (!detail.empty()) detail += "  ·  ";
        detail += s;
    };
    if (b.scale < 0.8f) {
        pts += 1;
        add("маленька +1");
    }
    if (S.clock - t.releaseTime <= kThrowWindow) {
        pts *= 2;
        add("кидок ×2");
        Vector3 d = t.releasePos - S.pos;
        float dist = std::sqrt(d.x * d.x + d.z * d.z);
        if (dist >= 7.0f) {
            pts += 2;
            add("далекий кидок +2");
        } else if (dist >= 4.0f) {
            pts += 1;
            add("здалеку +1");
        }
    }
    S.score += pts;
    t.scored = true;
    t.ejectAt = S.clock + 0.8f;
    S.pendingMove = true;

    Popup p;
    p.pos = {S.pos.x, S.platformTop + kH + 0.5f, S.pos.z};
    char buf[32];
    std::snprintf(buf, sizeof buf, "+%d!", pts);
    p.text = buf;
    p.detail = detail;
    S.popups.push_back(p);
    audio::PlaySquelch(0.8f);
    audio::PlaySplat(0.5f);
}

Vector3 PickPosition() {
    const float lim = S.platformHalf - 1.8f;
    const Vector3 avoid[2] = {{-4, 0, 3}, {4, 0, -3}};
    Vector3 best = {-S.pos.x, 0, -S.pos.z};
    for (int attempt = 0; attempt < 60; attempt++) {
        Vector3 c = {RandF(-lim, lim), 0, RandF(-lim, lim)};
        if (Len(c) < 2.2f) continue;
        if (Len(c - avoid[0]) < 2.8f || Len(c - avoid[1]) < 2.8f) continue;
        Vector3 d = c - S.pos;
        if (std::sqrt(d.x * d.x + d.z * d.z) < 3.5f) continue;
        best = c;
        break;
    }
    return best;
}

void Animate(float dt) {
    if (S.anim == 1) {
        S.animT += dt;
        S.scale = 1.0f - Smooth(S.animT / 0.25f);
        if (S.animT >= 0.25f) {
            S.pos = S.nextPos;
            S.anim = 2;
            S.animT = 0;
            S.scale = 0;
        }
    } else if (S.anim == 2) {
        S.animT += dt;
        float k = Clamp01(S.animT / 0.4f);
        // ease-out with a little overshoot
        const float c1 = 1.70158f, c3 = c1 + 1.0f;
        S.scale = 1.0f + c3 * std::pow(k - 1.0f, 3.0f) + c1 * std::pow(k - 1.0f, 2.0f);
        if (k >= 1.0f) {
            S.scale = 1.0f;
            S.anim = 0;
        }
    }
}

const Body* FindBody(const World& w, int id) {
    for (const Body& b : w.bodies)
        if (b.id == id) return &b;
    return nullptr;
}

}  // namespace

void Init(World& world) {
    S.world = &world;
    S.platformTop = world.params.platformTop;
    S.platformHalf = world.params.platformHalf;
    if (!S.meshes) BuildMeshes();
    LoadRecord();
    world.colliders.push_back(&gCollider);
}

void Shutdown(World& world) {
    auto& c = world.colliders;
    c.erase(std::remove(c.begin(), c.end(), static_cast<Collider*>(&gCollider)), c.end());
    if (S.meshes) {
        UnloadMesh(S.wall);
        UnloadMesh(S.floor);
        UnloadMesh(S.rim);
        S.meshes = false;
    }
}

void SetEnabled(bool on) {
    if (on == S.enabled) return;
    S.enabled = on;
    S.tracks.clear();
    S.popups.clear();
    S.lastGrabbedId = -1;
    S.pendingMove = false;
    if (on) {
        StartRound();
        S.scale = 0;
        S.anim = 2;  // pop in
        S.animT = 0;
    } else {
        S.scale = 0;
        S.anim = 0;
    }
}

bool Enabled() { return S.enabled; }

void Reset() {
    S.tracks.clear();
    S.lastGrabbedId = -1;
    if (S.enabled) StartRound();
}

void SetCamera(const Camera3D& cam) {
    S.cam = cam;
    S.hasCam = true;
}

void Update(World& w, float dt) {
    S.platformTop = w.params.platformTop;
    S.platformHalf = w.params.platformHalf;
    if (!S.enabled) return;
    S.clock += dt;
    Animate(dt);
    for (Popup& p : S.popups) p.age += dt;
    S.popups.erase(std::remove_if(S.popups.begin(), S.popups.end(), [](const Popup& p) { return p.age > 1.8f; }),
                   S.popups.end());
    if (!S.roundOver) {
        S.roundTime += dt;
        S.timeLeft -= dt;
        if (S.timeLeft <= 0) {
            S.timeLeft = 0;
            EndRound();
        }
    }

    // Remember where and when the player let go of a jelly
    const int gi = w.GrabbedBody();
    const int gid = gi >= 0 ? w.bodies[gi].id : -1;
    if (S.lastGrabbedId >= 0 && S.lastGrabbedId != gid) {
        if (const Body* b = FindBody(w, S.lastGrabbedId)) {
            Track& t = S.tracks[b->id];
            t.releaseTime = S.clock;
            t.releasePos = b->center;
        }
    }
    S.lastGrabbedId = gid;

    for (auto& kv : S.tracks) kv.second.seen = false;
    const float s = S.scale;
    const float rimY = kH * s;
    const float openR = (kRt - 0.5f * kT) * s;
    std::vector<int> inside;
    bool incoming = false;
    for (int bi = 0; bi < (int)w.bodies.size(); bi++) {
        const Body& b = w.bodies[bi];
        Track& t = S.tracks[b.id];
        t.seen = true;
        float r = 0, y = 0;
        const bool in = InsideBasket(b.center, s, &r, &y);
        if (t.hasPrev && t.prevY > rimY && y <= rimY && r < openR && S.anim == 0) t.entered = true;
        if (y > rimY) t.entered = false;
        t.prevY = y;
        t.hasPrev = true;
        if (y > rimY && y < rimY + 1.5f && r < openR + 0.3f) incoming = true;

        if (in) {
            inside.push_back(bi);
            t.insideTime += dt;
        } else {
            t.insideTime = 0;
        }
        if (in && t.entered && !t.scored && bi != gi && t.insideTime >= kScoreDelay && !S.roundOver && S.anim == 0)
            Score(b, t);
        if (t.scored && t.ejectAt > 0 && S.clock >= t.ejectAt) {
            if (in) Eject(w, b);
            t.ejectAt = -1;
        }
    }
    for (auto it = S.tracks.begin(); it != S.tracks.end();) {
        if (!it->second.seen) it = S.tracks.erase(it);
        else ++it;
    }

    // Too crowded: pop everything out (once, then give them time to leave)
    S.crowdCooldown = std::max(0.0f, S.crowdCooldown - dt);
    if ((int)inside.size() > kMaxInside && S.crowdCooldown <= 0) {
        for (int bi : inside) Eject(w, w.bodies[bi]);
        S.crowdCooldown = 1.5f;
    }

    // Hop to a new place once the basket has been empty for a moment and no
    // thrown jelly is still on its way
    bool flying = false;
    for (const auto& kv : S.tracks)
        if (S.clock - kv.second.releaseTime < 2.0f) flying = true;
    S.emptyTime = (inside.empty() && !incoming) ? S.emptyTime + dt : 0.0f;
    if (S.pendingMove && S.anim == 0 && S.emptyTime > 0.6f && !flying) {
        S.nextPos = PickPosition();
        S.anim = 1;
        S.animT = 0;
        S.pendingMove = false;
    }
}

void AppendSceneObjects(std::vector<SceneObject>& out) {
    if (!S.enabled || !S.meshes || S.scale < 0.01f) return;
    const float s = S.scale;
    const Matrix xf = MatrixMultiply(MatrixScale(s, s, s), MatrixTranslate(S.pos.x, S.platformTop, S.pos.z));
    SceneObject wall;
    wall.mesh = &S.wall;
    wall.transform = xf;
    wall.color = {0.86f, 0.64f, 0.36f};
    wall.gloss = 0.12f;
    out.push_back(wall);
    SceneObject floor = wall;
    floor.mesh = &S.floor;
    floor.color = {0.74f, 0.54f, 0.30f};
    out.push_back(floor);
    SceneObject rim = wall;
    rim.mesh = &S.rim;
    rim.color = {0.56f, 0.35f, 0.17f};
    rim.gloss = 0.25f;
    out.push_back(rim);
}

void DrawHUD() {
    if (!S.enabled) return;
    const Color white = {245, 245, 250, 255};
    const Color gold = {255, 196, 70, 255};
    const Color dim = {245, 245, 250, 180};
    const Color warn = {255, 110, 90, 255};
    const float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();

    // Top bar: «Кошик · Рахунок: 12 · Час: 0:37 · Рекорд: 25»
    char score[48], timeText[48], rec[48];
    const int secs = (int)std::ceil(S.timeLeft);
    std::snprintf(score, sizeof score, "  ·  Рахунок: %d", S.score);
    std::snprintf(timeText, sizeof timeText, "  ·  Час: %d:%02d", secs / 60, secs % 60);
    std::snprintf(rec, sizeof rec, "  ·  Рекорд: %d", S.record);
    const float fs = 22;
    const char* parts[4] = {"Кошик", score, timeText, rec};
    const Color cols[4] = {gold, white, (secs <= 10 && !S.roundOver) ? warn : white, dim};
    float total = 0;
    for (const char* p : parts) total += ui::Measure(p, fs).x;
    Rectangle bar = {sw * 0.5f - total * 0.5f - 18, 10, total + 36, 40};
    ui::Panel(bar);
    float x = bar.x + 18;
    for (int i = 0; i < 4; i++) {
        ui::Label(parts[i], Vector2{x, bar.y + 9}, fs, cols[i]);
        x += ui::Measure(parts[i], fs).x;
    }
    if (!S.roundOver && S.roundTime < 5.0f) {
        const char* hint = "Закидайте желейки в кошик! Бонуси: кидок ×2, здалеку, маленькі желейки";
        Vector2 m = ui::Measure(hint, 17);
        // keep clear of the control panel on the left
        ui::Label(hint, Vector2{std::max(sw * 0.5f - m.x * 0.5f, 352.0f), bar.y + bar.height + 8}, 17, dim);
    }

    // Score pop-ups above the basket
    if (S.hasCam) {
        const Vector3 fwd = Norm(S.cam.target - S.cam.position);
        for (const Popup& p : S.popups) {
            Vector3 wp = p.pos + Vector3{0, p.age * 0.7f, 0};
            if (Dot(wp - S.cam.position, fwd) <= 0.1f) continue;
            Vector2 sp = GetWorldToScreen(wp, S.cam);
            float a = 1.0f - Clamp01((p.age - 1.0f) / 0.8f);
            float pop = 1.0f + 0.35f * (1.0f - Clamp01(p.age / 0.18f));
            Color c = gold;
            c.a = (unsigned char)(255 * a);
            const float big = 44 * pop;
            Vector2 m = ui::Measure(p.text.c_str(), big);
            ui::Label(p.text.c_str(), Vector2{sp.x - m.x * 0.5f, sp.y - m.y * 0.5f}, big, c);
            if (!p.detail.empty()) {
                Color d = white;
                d.a = (unsigned char)(230 * a);
                Vector2 dm = ui::Measure(p.detail.c_str(), 17);
                ui::Label(p.detail.c_str(), Vector2{sp.x - dm.x * 0.5f, sp.y + m.y * 0.5f + 2}, 17, d);
            }
        }
    }

    // Round over
    if (S.roundOver) {
        const float w = 400, h = 210;
        Rectangle r = {sw * 0.5f - w * 0.5f, sh * 0.5f - h * 0.5f, w, h};
        ui::Panel(r);
        auto center = [&](const char* text, float y, float size, Color c) {
            Vector2 m = ui::Measure(text, size);
            ui::Label(text, Vector2{r.x + (w - m.x) * 0.5f, y}, size, c);
        };
        char buf[64];
        center("Час вийшов!", r.y + 18, 34, gold);
        std::snprintf(buf, sizeof buf, "Рахунок: %d", S.score);
        center(buf, r.y + 64, 28, white);
        if (S.newRecord && S.score > 0) {
            center("Новий рекорд!", r.y + 100, 22, gold);
        } else {
            std::snprintf(buf, sizeof buf, "Рекорд: %d", S.record);
            center(buf, r.y + 100, 22, dim);
        }
        if (ui::Button(Rectangle{r.x + 40, r.y + 140, 150, 36}, "Ще раз", true)) StartRound();
        if (ui::Button(Rectangle{r.x + 210, r.y + 140, 150, 36}, "Вийти з гри")) SetEnabled(false);
        center("R — ще раз (нова сцена)  ·  G — вийти", r.y + 184, 15, dim);
    }
}

}  // namespace basket
