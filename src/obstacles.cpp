#include "obstacles.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace obstacles {
namespace {

// --- layout (platform top is y = 0 by default) ------------------------------

// Ramp: a wedge rising along `dir` from its low edge to a vertical back face
const float kRampX = -4.0f, kRampZ = 3.0f;
const Vector3 kRampDir = {-1, 0, 0};  // uphill direction (towards the platform edge)
const float kRampLen = 3.2f, kRampWidth = 2.0f, kRampHeight = 1.2f;
const float kRampMuS = 0.2f, kRampMuK = 0.12f;  // slippery, so jellies slide down

// Trampoline: padded ring on a skirt, with an elastic membrane inside
const float kTrampX = 4.0f, kTrampZ = -3.0f;
const float kFrameRadius = 1.3f;  // torus major radius
const float kTube = 0.09f;        // torus minor radius
const float kFrameHeight = 0.5f;  // torus centre above the platform
const float kMembraneRadius = kFrameRadius - kTube * 0.9f;
const float kMembraneDrop = 0.02f;  // membrane rest height below the torus centre
const float kMaxDip = 0.38f;        // deepest the membrane can be pushed
const float kMembraneHz = 7.0f;     // stiffness of the soft membrane contact
// Leaving the membrane, a body is topped up to gain * landing speed (clamped);
// after a few boosted bounces in a row it only gets the natural rebound and
// settles down
const float kBounceMin = 6.0f, kBounceMax = 11.0f, kBounceGain = 1.25f;
const float kBounceThreshold = 3.5f;  // slower landings just rest on the membrane
const int kBoostedBounces = 3;

const float kBandHeight = 0.2f;  // padded band under the ring; legs below it
const int kLegs = 6;
const float kLegRadius = 0.05f;

const int kRings = 14, kSegments = 48;

// --- mesh helper ---------------------------------------------------------------

struct MeshBuilder {
    std::vector<Vector3> p, n;
    std::vector<unsigned short> idx;

    int V(Vector3 pos, Vector3 nrm) {
        p.push_back(pos);
        n.push_back(nrm);
        return (int)p.size() - 1;
    }
    // Triangle facing `outward` (counter-clockwise when seen from that side)
    void T(int a, int b, int c, Vector3 outward) {
        if (Dot(Cross(p[b] - p[a], p[c] - p[a]), outward) < 0) std::swap(b, c);
        idx.push_back((unsigned short)a);
        idx.push_back((unsigned short)b);
        idx.push_back((unsigned short)c);
    }
    void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 nrm) {
        int i0 = V(a, nrm), i1 = V(b, nrm), i2 = V(c, nrm), i3 = V(d, nrm);
        T(i0, i1, i2, nrm);
        T(i0, i2, i3, nrm);
    }
    Mesh Build(bool dynamic) const {
        Mesh m = {};
        m.vertexCount = (int)p.size();
        m.triangleCount = (int)idx.size() / 3;
        m.vertices = (float*)RL_MALLOC(p.size() * 3 * sizeof(float));
        m.normals = (float*)RL_MALLOC(n.size() * 3 * sizeof(float));
        m.indices = (unsigned short*)RL_MALLOC(idx.size() * sizeof(unsigned short));
        for (size_t i = 0; i < p.size(); i++) {
            m.vertices[3 * i + 0] = p[i].x;
            m.vertices[3 * i + 1] = p[i].y;
            m.vertices[3 * i + 2] = p[i].z;
            m.normals[3 * i + 0] = n[i].x;
            m.normals[3 * i + 1] = n[i].y;
            m.normals[3 * i + 2] = n[i].z;
        }
        std::copy(idx.begin(), idx.end(), m.indices);
        UploadMesh(&m, dynamic);
        return m;
    }
};

// --- state ---------------------------------------------------------------------

struct BounceState {
    bool inContact = false;
    float impact = 0;  // fastest downward speed since the contact started
    float sinceContact = 0;
    int streak = 0;    // boosted bounces without leaving the trampoline
};

// Contact of one body with the membrane during the latest substep (for visuals)
struct MembraneContact {
    float cx = 0, cz = 0;  // contact centroid
    float depth = 0;       // how far the body pushes the membrane below rest
    float spread = 0;      // contact radius
    bool fresh = false;
};

// Animated dip of the membrane (follows a contact, wobbles after release)
struct Dip {
    float cx = 0, cz = 0, spread = 0.3f;
    float depth = 0, vel = 0;
    bool active = false;
};

struct Obstacles;

struct ObstacleCollider : Collider {
    Obstacles* o = nullptr;
    void Collide(World& world, float dt) override;
};

struct Obstacles {
    bool enabled = false;
    bool ready = false;
    float top = 0;
    Vector3 rampC{}, rampPerp{};
    Vector3 trampC{};  // centre on the platform surface
    float frameY = 0, membraneY = 0;

    Mesh rampMesh = {}, frameMesh = {}, skirtMesh = {}, legsMesh = {}, membraneMesh = {};
    std::vector<Vector3> memRest;  // membrane vertex rest positions
    std::vector<Vector3> memNormals;

    ObstacleCollider collider;
    std::unordered_map<int, BounceState> bounce;     // by Body::id
    std::unordered_map<int, MembraneContact> touch;  // by Body::id
    std::unordered_map<int, Dip> dips;               // by Body::id

    // per-substep scratch, indexed by body index
    std::vector<int> cnt;
    std::vector<float> minY, sumX, sumZ, maxR;

    void BuildMeshes();
    void CollideRamp(World& w, int i);
    bool CollideTrampoline(World& w, int i, float alpha);
    void UpdateMembraneMesh();
};

Obstacles gO;

// --- meshes --------------------------------------------------------------------

void Obstacles::BuildMeshes() {
    // Ramp wedge (world coordinates)
    {
        const Vector3 up = {0, 1, 0};
        auto P = [&](float u, float y, float w) { return rampC + kRampDir * u + up * y + rampPerp * w; };
        const float hl = kRampLen * 0.5f, hw = kRampWidth * 0.5f;
        Vector3 lowA = P(-hl, 0, -hw), lowB = P(-hl, 0, hw);
        Vector3 topA = P(hl, kRampHeight, -hw), topB = P(hl, kRampHeight, hw);
        Vector3 botA = P(hl, 0, -hw), botB = P(hl, 0, hw);
        Vector3 slopeN = Norm(kRampDir * (-kRampHeight / kRampLen) + up);
        MeshBuilder mb;
        mb.Quad(lowA, lowB, topB, topA, slopeN);
        mb.Quad(botA, botB, topB, topA, kRampDir);
        {
            int a = mb.V(lowB, rampPerp), b = mb.V(botB, rampPerp), c = mb.V(topB, rampPerp);
            mb.T(a, b, c, rampPerp);
            Vector3 np = rampPerp * -1.0f;
            int d = mb.V(lowA, np), e = mb.V(botA, np), f = mb.V(topA, np);
            mb.T(d, e, f, np);
        }
        rampMesh = mb.Build(false);
    }

    // Trampoline frame: padded torus
    {
        MeshBuilder mb;
        const int U = 72, Vn = 14;
        for (int i = 0; i <= U; i++) {
            float u = 2 * PI * i / U;
            for (int j = 0; j <= Vn; j++) {
                float v = 2 * PI * j / Vn;
                Vector3 nrm = {std::cos(v) * std::cos(u), std::sin(v), std::cos(v) * std::sin(u)};
                Vector3 c = {trampC.x + kFrameRadius * std::cos(u), frameY, trampC.z + kFrameRadius * std::sin(u)};
                mb.V(c + nrm * kTube, nrm);
            }
        }
        for (int i = 0; i < U; i++)
            for (int j = 0; j < Vn; j++) {
                int a = i * (Vn + 1) + j, b = (i + 1) * (Vn + 1) + j, c = (i + 1) * (Vn + 1) + j + 1, d = i * (Vn + 1) + j + 1;
                Vector3 out = mb.n[a] + mb.n[c];
                mb.T(a, b, c, out);
                mb.T(a, c, d, out);
            }
        frameMesh = mb.Build(false);
    }

    // Padded band under the ring (the solid collision drum below it is not drawn)
    {
        MeshBuilder mb;
        const int U = 72;
        for (int i = 0; i <= U; i++) {
            float u = 2 * PI * i / U;
            Vector3 nrm = {std::cos(u), 0, std::sin(u)};
            Vector3 base = {trampC.x + kFrameRadius * nrm.x, frameY - kBandHeight, trampC.z + kFrameRadius * nrm.z};
            mb.V(base, nrm);
            mb.V(base + Vector3{0, kBandHeight, 0}, nrm);
        }
        for (int i = 0; i < U; i++) {
            int a = 2 * i, b = 2 * i + 1, c = 2 * i + 3, d = 2 * i + 2;
            mb.T(a, b, c, mb.n[a]);
            mb.T(a, c, d, mb.n[a]);
        }
        skirtMesh = mb.Build(false);
    }

    // Legs
    {
        MeshBuilder mb;
        const int S = 12;
        for (int l = 0; l < kLegs; l++) {
            float a = 2 * PI * (l + 0.5f) / kLegs;
            Vector3 c = {trampC.x + (kFrameRadius - 0.06f) * std::cos(a), top, trampC.z + (kFrameRadius - 0.06f) * std::sin(a)};
            int first = (int)mb.p.size();
            for (int i = 0; i <= S; i++) {
                float u = 2 * PI * i / S;
                Vector3 nrm = {std::cos(u), 0, std::sin(u)};
                mb.V(c + nrm * kLegRadius, nrm);
                mb.V(c + nrm * kLegRadius + Vector3{0, frameY - kBandHeight - top + 0.02f, 0}, nrm);
            }
            for (int i = 0; i < S; i++) {
                int q = first + 2 * i;
                mb.T(q, q + 1, q + 3, mb.n[q]);
                mb.T(q, q + 3, q + 2, mb.n[q]);
            }
        }
        legsMesh = mb.Build(false);
    }

    // Membrane: radial grid, updated every frame
    {
        MeshBuilder mb;
        const Vector3 up = {0, 1, 0};
        mb.V({trampC.x, membraneY, trampC.z}, up);
        for (int k = 1; k <= kRings; k++) {
            float rr = kMembraneRadius * k / kRings;
            for (int j = 0; j < kSegments; j++) {
                float a = 2 * PI * j / kSegments;
                mb.V({trampC.x + rr * std::cos(a), membraneY, trampC.z + rr * std::sin(a)}, up);
            }
        }
        auto ring = [](int k, int j) { return 1 + (k - 1) * kSegments + (j % kSegments); };
        for (int j = 0; j < kSegments; j++) mb.T(0, ring(1, j), ring(1, j + 1), up);
        for (int k = 1; k < kRings; k++)
            for (int j = 0; j < kSegments; j++) {
                int a = ring(k, j), b = ring(k + 1, j), c = ring(k + 1, j + 1), d = ring(k, j + 1);
                mb.T(a, b, c, up);
                mb.T(a, c, d, up);
            }
        memRest = mb.p;
        memNormals = mb.n;
        membraneMesh = mb.Build(true);
    }
}

void Obstacles::UpdateMembraneMesh() {
    float* vtx = membraneMesh.vertices;
    const int nv = membraneMesh.vertexCount;
    for (int i = 0; i < nv; i++) {
        Vector3 p = memRest[i];
        float rho = std::sqrt((p.x - trampC.x) * (p.x - trampC.x) + (p.z - trampC.z) * (p.z - trampC.z));
        float edge = std::max(0.0f, 1.0f - (rho / kMembraneRadius) * (rho / kMembraneRadius));
        float y = membraneY;
        for (auto& [id, d] : dips) {
            float dc = std::sqrt((d.cx - trampC.x) * (d.cx - trampC.x) + (d.cz - trampC.z) * (d.cz - trampC.z));
            float edgeC = std::max(0.3f, 1.0f - (dc / kMembraneRadius) * (dc / kMembraneRadius));
            float sigma = 0.22f + 0.65f * d.spread;
            float dd = (p.x - d.cx) * (p.x - d.cx) + (p.z - d.cz) * (p.z - d.cz);
            y -= d.depth * std::exp(-dd / (2 * sigma * sigma)) * edge / edgeC;
        }
        y = std::max(y, membraneY - kMaxDip);
        vtx[3 * i + 0] = p.x;
        vtx[3 * i + 1] = y;
        vtx[3 * i + 2] = p.z;
    }
    // Smooth normals from the triangles
    std::fill(memNormals.begin(), memNormals.end(), Vector3{0, 0, 0});
    const unsigned short* ind = membraneMesh.indices;
    for (int t = 0; t < membraneMesh.triangleCount; t++) {
        int a = ind[3 * t], b = ind[3 * t + 1], c = ind[3 * t + 2];
        Vector3 pa = {vtx[3 * a], vtx[3 * a + 1], vtx[3 * a + 2]};
        Vector3 pb = {vtx[3 * b], vtx[3 * b + 1], vtx[3 * b + 2]};
        Vector3 pc = {vtx[3 * c], vtx[3 * c + 1], vtx[3 * c + 2]};
        Vector3 fn = Cross(pb - pa, pc - pa);
        memNormals[a] += fn;
        memNormals[b] += fn;
        memNormals[c] += fn;
    }
    for (int i = 0; i < nv; i++) {
        Vector3 nn = Norm(memNormals[i]);
        if (Len2(nn) < 0.5f) nn = {0, 1, 0};
        membraneMesh.normals[3 * i + 0] = nn.x;
        membraneMesh.normals[3 * i + 1] = nn.y;
        membraneMesh.normals[3 * i + 2] = nn.z;
    }
    UpdateMeshBuffer(membraneMesh, 0, membraneMesh.vertices, nv * 3 * sizeof(float), 0);
    UpdateMeshBuffer(membraneMesh, 2, membraneMesh.normals, nv * 3 * sizeof(float), 0);
}

// --- collision -----------------------------------------------------------------

void Obstacles::CollideRamp(World& w, int i) {
    const float r = w.radius[i];
    Vector3& p = w.x[i];
    Vector3 q = p - rampC;
    const float u = Dot(q, kRampDir), y = q.y, s = Dot(q, rampPerp);
    const float hl = kRampLen * 0.5f, hw = kRampWidth * 0.5f;
    if (u < -hl - r || u > hl + r || std::fabs(s) > hw + r || y > kRampHeight + r || y < -r) return;

    // Signed distances of the sphere centre to the wedge faces (local u, y, s)
    const float invLen = 1.0f / std::sqrt(kRampHeight * kRampHeight + kRampLen * kRampLen);
    const float nu = -kRampHeight * invLen, ny = kRampLen * invLen;  // slope normal in (u, y)
    float dSlope = nu * (u + hl) + ny * y;
    float dBack = u - hl;
    float dSideP = s - hw, dSideN = -s - hw;
    float dBottom = -y;
    float best = dSlope;
    int face = 0;
    if (dBack > best) best = dBack, face = 1;
    if (dSideP > best) best = dSideP, face = 2;
    if (dSideN > best) best = dSideN, face = 3;
    if (dBottom > best) best = dBottom, face = 4;
    if (best >= r) return;  // outside the wedge (expanded by r)

    const Vector3 up = {0, 1, 0};
    Vector3 n;
    switch (face) {
        case 0: n = kRampDir * nu + up * ny; break;
        case 1: n = kRampDir; break;
        case 2: n = rampPerp; break;
        case 3: n = rampPerp * -1.0f; break;
        default: n = up * -1.0f; break;
    }
    const float pen = r - best;
    p += n * pen;
    if (face == 0) {
        // PBD friction on the slanted surface
        Vector3 d = p - w.xprev[i];
        Vector3 t = d - n * Dot(d, n);
        float lt = Len(t);
        if (lt < kRampMuS * pen) {
            p -= t;
        } else if (lt > 1e-9f) {
            p -= t * std::min(kRampMuK * pen / lt, 1.0f);
        }
        w.MarkGrounded(i);
    }
}

// Returns true when the particle touches the membrane
bool Obstacles::CollideTrampoline(World& w, int i, float alpha) {
    const float r = w.radius[i];
    Vector3& p = w.x[i];
    const float qx = p.x - trampC.x, qz = p.z - trampC.z;
    const float rho2 = qx * qx + qz * qz;
    const float outer = kFrameRadius + kTube + r + 0.02f;
    if (rho2 > outer * outer || p.y > frameY + kTube + r) return false;
    float rho = std::sqrt(rho2);

    // Padded ring (torus)
    if (rho > 1e-5f) {
        Vector3 c = {trampC.x + qx / rho * kFrameRadius, frameY, trampC.z + qz / rho * kFrameRadius};
        Vector3 d = p - c;
        float dist = Len(d);
        if (dist < kTube + r && dist > 1e-6f) {
            p = c + d * ((kTube + r) / dist);
            w.MarkGrounded(i);
        }
    }

    // Skirt wall: keep particles on the side they came from
    const float px = w.xprev[i].x - trampC.x, pz = w.xprev[i].z - trampC.z;
    const bool outside = px * px + pz * pz >= kFrameRadius * kFrameRadius;
    rho = std::sqrt((p.x - trampC.x) * (p.x - trampC.x) + (p.z - trampC.z) * (p.z - trampC.z));
    if (p.y < frameY && rho > 1e-5f) {
        float lim = outside ? kFrameRadius + 0.02f + r : kMembraneRadius - r * 0.5f;
        if (outside ? rho < lim : (rho > lim && p.y < membraneY)) {
            float k = lim / rho;
            p.x = trampC.x + (p.x - trampC.x) * k;
            p.z = trampC.z + (p.z - trampC.z) * k;
            rho = lim;
        }
    }
    if (outside || rho >= kMembraneRadius) return false;

    // Membrane: soft spring contact above a hard floor
    float pen = membraneY + r - p.y;
    if (pen <= 0) return false;
    const float stiff = 1.0f / std::max(0.15f, 1.0f - 0.85f * (rho / kMembraneRadius) * (rho / kMembraneRadius));
    p.y += pen * std::min(1.0f, alpha * stiff);
    const float floorY = membraneY - kMaxDip + r;
    if (p.y < floorY) p.y = floorY;
    // Rubber grip
    p.x += (w.xprev[i].x - p.x) * 0.12f;
    p.z += (w.xprev[i].z - p.z) * 0.12f;
    w.MarkGrounded(i);
    return true;
}

void ObstacleCollider::Collide(World& w, float dt) {
    Obstacles& O = *o;
    if (!O.enabled || !O.ready) return;
    const int nb = (int)w.bodies.size();
    O.cnt.assign(nb, 0);
    O.minY.assign(nb, 1e9f);
    O.sumX.assign(nb, 0);
    O.sumZ.assign(nb, 0);
    O.maxR.assign(nb, 0);

    float alpha = 2.0f * PI * kMembraneHz * dt;
    alpha *= alpha;

    const int n = (int)w.x.size();
    for (int i = 0; i < n; i++) {
        O.CollideRamp(w, i);
        if (O.CollideTrampoline(w, i, alpha)) {
            int b = w.owner[i];
            O.cnt[b]++;
            O.minY[b] = std::min(O.minY[b], w.x[i].y - w.radius[i]);
            O.sumX[b] += w.x[i].x;
            O.sumZ[b] += w.x[i].z;
        }
    }

    // Bodies on the membrane: remember the landing speed; when a body leaves
    // the membrane moving up, top its speed up (restitution > 1, capped)
    for (auto& [id, c] : O.touch) c.fresh = false;
    const int grabbed = w.GrabbedBody();
    auto meanVy = [&](const Body& body) {
        float vy = 0;
        for (int i = body.first; i < body.first + body.count; i++) vy += (w.x[i].y - w.xprev[i].y);
        return vy / (body.count * dt);
    };
    for (int b = 0; b < nb; b++) {
        const Body& body = w.bodies[b];
        if (O.cnt[b] == 0) {
            auto it = O.bounce.find(body.id);
            if (it == O.bounce.end()) continue;
            BounceState& st = it->second;
            st.sinceContact += dt;
            if (st.inContact) {
                st.inContact = false;
                float vy = meanVy(body);
                if (st.impact > kBounceThreshold && vy > 0 && b != grabbed && st.streak < kBoostedBounces) {
                    float target = std::clamp(kBounceGain * st.impact, kBounceMin, kBounceMax);
                    if (target > vy) {
                        float delta = target - vy;
                        for (int i = body.first; i < body.first + body.count; i++) w.xprev[i].y -= delta * dt;
                    }
                    st.streak++;
                }
            }
            continue;
        }
        BounceState& st = O.bounce[body.id];
        float vy = meanVy(body);
        if (!st.inContact) {
            st.inContact = true;
            st.impact = 0;
            if (st.sinceContact > 2.5f) st.streak = 0;
        }
        st.sinceContact = 0;
        st.impact = std::max(st.impact, -vy);

        MembraneContact& mc = O.touch[body.id];
        mc.cx = O.sumX[b] / O.cnt[b];
        mc.cz = O.sumZ[b] / O.cnt[b];
        mc.depth = std::clamp(O.membraneY - O.minY[b], 0.0f, kMaxDip);
        mc.spread = std::min(0.9f, std::sqrt((float)O.cnt[b]) * body.particleRadius * 1.2f);
        mc.fresh = true;
    }
    for (auto it = O.bounce.begin(); it != O.bounce.end();) {
        if (it->second.sinceContact > 3.0f) it = O.bounce.erase(it);
        else ++it;
    }
}

}  // namespace

void Init(World& world) {
    Obstacles& O = gO;
    O.top = world.params.platformTop;
    O.rampC = {kRampX, O.top, kRampZ};
    O.rampPerp = Norm(Cross(Vector3{0, 1, 0}, kRampDir));
    O.trampC = {kTrampX, O.top, kTrampZ};
    O.frameY = O.top + kFrameHeight;
    O.membraneY = O.frameY - kMembraneDrop;
    O.BuildMeshes();
    O.collider.o = &O;
    world.colliders.push_back(&O.collider);
    O.ready = true;
}

void Shutdown(World& world) {
    Obstacles& O = gO;
    auto& cs = world.colliders;
    cs.erase(std::remove(cs.begin(), cs.end(), (Collider*)&O.collider), cs.end());
    if (!O.ready) return;
    UnloadMesh(O.rampMesh);
    UnloadMesh(O.frameMesh);
    UnloadMesh(O.skirtMesh);
    UnloadMesh(O.legsMesh);
    UnloadMesh(O.membraneMesh);
    O.ready = false;
}

void SetEnabled(bool on) {
    gO.enabled = on;
    if (!on) {
        gO.bounce.clear();
        gO.touch.clear();
        gO.dips.clear();
    }
}

bool Enabled() { return gO.enabled; }

void Update(const World& world, float dt) {
    Obstacles& O = gO;
    if (!O.enabled || !O.ready || dt <= 0) return;

    // Contacts drive the dips; released dips wobble back with a damped spring
    for (auto& [id, c] : O.touch) {
        Dip& d = O.dips[id];
        if (c.fresh) {
            if (!d.active) d.vel = 0;
            float prev = d.depth;
            d.depth = c.depth;
            d.vel = (d.depth - prev) / dt;
            d.cx = c.cx;
            d.cz = c.cz;
            d.spread = c.spread;
            d.active = true;
        } else {
            d.active = false;
        }
    }
    O.touch.clear();
    const float k = 700.0f, damp = 9.0f;
    for (auto it = O.dips.begin(); it != O.dips.end();) {
        Dip& d = it->second;
        if (!d.active) {
            d.vel += (-k * d.depth - damp * d.vel) * dt;
            d.depth += d.vel * dt;
            if (std::fabs(d.depth) < 0.002f && std::fabs(d.vel) < 0.02f) {
                it = O.dips.erase(it);
                continue;
            }
        }
        ++it;
    }
    // Keep the touch map only for bodies still in contact this frame
    (void)world;
    O.UpdateMembraneMesh();
}

void AppendSceneObjects(std::vector<SceneObject>& out) {
    Obstacles& O = gO;
    if (!O.enabled || !O.ready) return;
    SceneObject ramp;
    ramp.mesh = &O.rampMesh;
    ramp.color = {1.0f, 0.70f, 0.22f};
    ramp.pattern = 2;
    ramp.gloss = 0.6f;
    out.push_back(ramp);

    SceneObject legs;
    legs.mesh = &O.legsMesh;
    legs.color = {0.55f, 0.58f, 0.62f};
    legs.gloss = 0.7f;
    out.push_back(legs);

    SceneObject skirt;
    skirt.mesh = &O.skirtMesh;
    skirt.color = {0.14f, 0.30f, 0.62f};
    skirt.pattern = 2;
    skirt.gloss = 0.3f;
    skirt.doubleSided = true;
    out.push_back(skirt);

    SceneObject frame;
    frame.mesh = &O.frameMesh;
    frame.color = {0.10f, 0.62f, 0.85f};
    frame.gloss = 0.8f;
    out.push_back(frame);

    SceneObject membrane;
    membrane.mesh = &O.membraneMesh;
    membrane.color = {0.10f, 0.12f, 0.24f};
    membrane.gloss = 0.55f;
    membrane.doubleSided = true;
    out.push_back(membrane);
}

}  // namespace obstacles
