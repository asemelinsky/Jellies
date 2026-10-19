#include "eyes.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "glsl.h"

namespace eyes {
namespace {

const float kBlinkDuration = 0.16f;
const float kFrameSpan = 0.05f;  // rest-space distance of the tangent helper points

// A point bound to the particles exactly like a render mesh vertex
struct SkinPoint {
    int idx[4];
    float w[4];
    Vector3 offset;
};

struct Anchor {
    SkinPoint p0, pu, pr;  // eye centre on the surface, plus "up" and "right" helpers
    Vector3 restNormal;
    float radius;
};

struct EyeState {
    float nextBlink = 3.0f;
    float blinkT = -1.0f;
    int blinksLeft = 0;
    float squint = 0.0f;
    float surprise = 0.0f;
    bool hasPrev = false;
    Vector3 prevCenter = {0, 0, 0}, prevVel = {0, 0, 0};
    Vector3 pupil[2] = {}, pupilVel[2] = {};
    unsigned rng = 1;
    bool seen = false;
};

struct Face {
    std::vector<Anchor> eyes;
    bool hasMouth = false;
    Anchor mouth;
};

std::vector<Face> gFaces;  // per shape kind
std::vector<float> gSpacing;
std::unordered_map<int, EyeState> gState;
Shader gShader = {};
Mesh gSphere = {};
Material gMat = {};
bool gReady = false;
int gLocLightDir = -1, gLocLightColor = -1, gLocAmbient = -1, gLocViewPos = -1, gLocEmissive = -1;

const char* kVS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 fragPos;
out vec3 fragNormal;
void main() {
    fragPos = (matModel * vec4(vertexPosition, 1.0)).xyz;
    fragNormal = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char* kFS = R"(#version 330
in vec3 fragPos;
in vec3 fragNormal;
uniform vec4 colDiffuse;
uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 ambient;
uniform vec3 viewPos;
uniform float emissive;
out vec4 finalColor;
void main() {
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(lightDir);
    vec3 V = normalize(viewPos - fragPos);
    float ndl = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, normalize(L + V)), 0.0), 70.0);
    vec3 c = colDiffuse.rgb * (ambient + lightColor * ndl * 0.85) + lightColor * spec * 0.6;
    c = max(c, colDiffuse.rgb * 0.22);  // stay readable at night
    c = mix(c, colDiffuse.rgb, emissive);
    finalColor = vec4(c, 1.0);
}
)";

float Rand01(unsigned& s) {
    s = s * 1664525u + 1013904223u;
    return (s >> 8) * (1.0f / 16777216.0f);
}

SkinPoint MakeSkin(const ShapeTemplate& st, Vector3 p) {
    SkinPoint sp;
    float bestD[4] = {1e9f, 1e9f, 1e9f, 1e9f};
    int best[4] = {0, 0, 0, 0};
    for (int i = 0; i < (int)st.rest.size(); i++) {
        float d = Len2(st.rest[i] - p);
        if (d >= bestD[3]) continue;
        int s = 3;
        while (s > 0 && bestD[s - 1] > d) {
            bestD[s] = bestD[s - 1];
            best[s] = best[s - 1];
            s--;
        }
        bestD[s] = d;
        best[s] = i;
    }
    const float soft = 0.5f * st.spacing;
    float wsum = 0;
    for (int k = 0; k < 4; k++) {
        sp.w[k] = 1.0f / (bestD[k] + soft * soft);
        wsum += sp.w[k];
    }
    Vector3 blend = {0, 0, 0};
    for (int k = 0; k < 4; k++) {
        sp.w[k] /= wsum;
        sp.idx[k] = best[k];
        blend += st.rest[best[k]] * sp.w[k];
    }
    sp.offset = p - blend;
    return sp;
}

Vector3 Eval(const World& w, const Body& b, const SkinPoint& s) {
    Vector3 p = b.rot * (s.offset * b.scale);
    for (int k = 0; k < 4; k++) p += w.x[b.first + s.idx[k]] * s.w[k];
    return p;
}

std::vector<Vector3> RestNormals(const ShapeTemplate& st) {
    std::vector<Vector3> n(st.meshRest.size(), Vector3{0, 0, 0});
    for (size_t t = 0; t + 2 < st.indices.size(); t += 3) {
        int a = st.indices[t], b = st.indices[t + 1], c = st.indices[t + 2];
        Vector3 fn = Cross(st.meshRest[b] - st.meshRest[a], st.meshRest[c] - st.meshRest[a]);
        n[a] += fn;
        n[b] += fn;
        n[c] += fn;
    }
    for (auto& v : n) v = Norm(v);
    return n;
}

int NearestVertex(const ShapeTemplate& st, Vector3 p) {
    int best = 0;
    float bd = 1e9f;
    for (int i = 0; i < (int)st.meshRest.size(); i++) {
        float d = Len2(st.meshRest[i] - p);
        if (d < bd) {
            bd = d;
            best = i;
        }
    }
    return best;
}

Anchor MakeAnchor(const ShapeTemplate& st, const std::vector<Vector3>& normals, int vertex, float radius) {
    Vector3 p = st.meshRest[vertex];
    Vector3 n = normals[vertex];
    Vector3 up = {0, 1, 0};
    Vector3 u = up - n * Dot(up, n);
    if (Len2(u) < 1e-6f) u = Vector3{0, 0, 1} - n * n.z;
    u = Norm(u);
    Vector3 r = Cross(u, n);
    Anchor a;
    a.p0 = MakeSkin(st, p);
    a.pu = MakeSkin(st, p + u * kFrameSpan);
    a.pr = MakeSkin(st, p + r * kFrameSpan);
    a.restNormal = n;
    a.radius = radius;
    return a;
}

int FrontVertex(const ShapeTemplate& st, float tx, float ty) {
    // Front-most (max z) mesh vertex near (tx, ty)
    int best = -1;
    for (float tol = 0.025f; best < 0 && tol < 0.2f; tol *= 1.6f) {
        float bestZ = -1e9f;
        for (int i = 0; i < (int)st.meshRest.size(); i++) {
            const Vector3& v = st.meshRest[i];
            if (std::fabs(v.x - tx) < tol && std::fabs(v.y - ty) < tol && v.z > bestZ) {
                bestZ = v.z;
                best = i;
            }
        }
    }
    return best >= 0 ? best : NearestVertex(st, Vector3{tx, ty, 0.3f});
}

void BuildBearFace(const ShapeTemplate& st, Face& face) {
    // The template is centred on its particle centroid; recover the vertical
    // offset from the known extents of the bear field (ears top ~0.545,
    // feet bottom ~-0.48) to place the eyes on the front of the head.
    float minY = 1e9f, maxY = -1e9f;
    for (const Vector3& v : st.meshRest) {
        minY = std::min(minY, v.y);
        maxY = std::max(maxY, v.y);
    }
    const float cy = 0.5f * ((0.545f - maxY) + (-0.48f - minY));
    std::vector<Vector3> normals = RestNormals(st);
    for (float side : {-1.0f, 1.0f})
        face.eyes.push_back(MakeAnchor(st, normals, FrontVertex(st, 0.09f * side, 0.305f - cy), 0.062f));
    face.mouth = MakeAnchor(st, normals, FrontVertex(st, 0.0f, 0.185f - cy), 1.0f);
    face.hasMouth = true;
}

void BuildWormFace(const ShapeTemplate& st, Face& face) {
    // Head = the round -x end; eyes sit on its upper front
    int tip = 0;
    for (int i = 1; i < (int)st.meshRest.size(); i++)
        if (st.meshRest[i].x < st.meshRest[tip].x) tip = i;
    const Vector3 head = st.meshRest[tip] + Vector3{0.19f, 0, 0};
    std::vector<Vector3> normals = RestNormals(st);
    for (float side : {-1.0f, 1.0f}) {
        Vector3 dir = Norm(Vector3{-0.5f, 0.68f, 0.4f * side});
        int v = NearestVertex(st, head + dir * 0.32f);
        face.eyes.push_back(MakeAnchor(st, normals, v, 0.056f));
    }
    face.mouth = MakeAnchor(st, normals, NearestVertex(st, head + Norm(Vector3{-0.97f, 0.1f, 0}) * 0.32f), 1.0f);
    face.hasMouth = true;
}

// Deformed tangent frame at an anchor; returns false when degenerate
bool AnchorFrame(const World& world, const Body& b, const Anchor& an, Vector3& p0, Vector3& T, Vector3& U, Vector3& N,
                 float& stretch) {
    p0 = Eval(world, b, an.p0);
    Vector3 du = Eval(world, b, an.pu) - p0;
    Vector3 dr = Eval(world, b, an.pr) - p0;
    N = Norm(Cross(dr, du));
    Vector3 restN = b.rot * an.restNormal;
    if (Len2(N) < 0.5f) N = Norm(restN);
    if (Dot(N, restN) < 0) N = N * -1.0f;
    U = Norm(du - N * Dot(du, N));
    if (Len2(U) < 0.5f) return false;
    T = Cross(U, N);
    // Features stretch a little with the jelly around them
    stretch = std::clamp(std::sqrt(Len(du) * Len(dr)) / (kFrameSpan * b.scale), 0.75f, 1.3f);
    return true;
}

void SetUniforms(Vector3 lightDir, Vector3 lightColor, Vector3 ambient, Vector3 viewPos) {
    SetShaderValue(gShader, gLocLightDir, &lightDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(gShader, gLocLightColor, &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(gShader, gLocAmbient, &ambient, SHADER_UNIFORM_VEC3);
    SetShaderValue(gShader, gLocViewPos, &viewPos, SHADER_UNIFORM_VEC3);
}

void DrawEllipsoid(Vector3 c, Vector3 T, Vector3 U, Vector3 N, Vector3 s, Color col, float emissive) {
    Matrix m = {T.x * s.x, U.x * s.y, N.x * s.z, c.x,  //
                T.y * s.x, U.y * s.y, N.y * s.z, c.y,  //
                T.z * s.x, U.z * s.y, N.z * s.z, c.z,  //
                0, 0, 0, 1};
    SetShaderValue(gShader, gLocEmissive, &emissive, SHADER_UNIFORM_FLOAT);
    gMat.maps[MATERIAL_MAP_DIFFUSE].color = col;
    DrawMesh(gSphere, gMat, m);
}

}  // namespace

void Init(const std::vector<ShapeTemplate>& templates) {
    gFaces.assign(templates.size(), Face{});
    for (size_t s = 0; s < templates.size(); s++) {
        if ((int)s == SHAPE_BEAR) BuildBearFace(templates[s], gFaces[s]);
        if ((int)s == SHAPE_WORM) BuildWormFace(templates[s], gFaces[s]);
    }
    gShader = LoadShaderFromMemory(PortGLSL(kVS).c_str(), PortGLSL(kFS).c_str());
    gLocLightDir = GetShaderLocation(gShader, "lightDir");
    gLocLightColor = GetShaderLocation(gShader, "lightColor");
    gLocAmbient = GetShaderLocation(gShader, "ambient");
    gLocViewPos = GetShaderLocation(gShader, "viewPos");
    gLocEmissive = GetShaderLocation(gShader, "emissive");
    gSphere = GenMeshSphere(1.0f, 14, 18);
    gMat = LoadMaterialDefault();
    gMat.shader = gShader;
    gReady = gShader.id > 0;
}

void Shutdown() {
    if (!gReady) return;
    UnloadMaterial(gMat);  // also unloads the shader
    UnloadMesh(gSphere);
    gState.clear();
    gReady = false;
}

void ForceBlink(int bodyId) {
    auto it = gState.find(bodyId);
    if (it != gState.end() && it->second.blinkT < 0) it->second.blinkT = 0;
}

void Update(const World& world, float dt) {
    if (dt <= 0) return;
    dt = std::min(dt, 0.05f);
    for (auto& kv : gState) kv.second.seen = false;

    const int grabbed = world.GrabbedBody();
    const float K = 480.0f, C = 8.0f, G = 7.0f;  // pupil spring, damping, inertia gain
    for (int bi = 0; bi < (int)world.bodies.size(); bi++) {
        const Body& b = world.bodies[bi];
        if (b.shape >= (int)gFaces.size() || gFaces[b.shape].eyes.empty()) continue;
        auto [it, fresh] = gState.try_emplace(b.id);
        EyeState& s = it->second;
        if (fresh) {
            s.rng = 0x9E3779B9u ^ (unsigned)b.id * 2654435761u;
            s.nextBlink = 0.8f + 3.0f * Rand01(s.rng);
        }
        s.seen = true;

        // Blinking, sometimes a double blink
        s.nextBlink -= dt;
        if (s.nextBlink <= 0 && s.blinkT < 0) {
            s.blinkT = 0;
            s.blinksLeft = Rand01(s.rng) < 0.2f ? 1 : 0;
            s.nextBlink = 2.0f + 4.0f * Rand01(s.rng);
        }
        if (s.blinkT >= 0) {
            s.blinkT += dt;
            if (s.blinkT > kBlinkDuration) {
                if (s.blinksLeft > 0) {
                    s.blinksLeft--;
                    s.blinkT = 0;
                } else {
                    s.blinkT = -1;
                }
            }
        }

        // Squint after a hard landing, wide eyes while being carried
        if (b.impactGround > 4.0f) s.squint = std::max(s.squint, std::min(1.0f, 0.5f + (b.impactGround - 4.0f) / 6.0f));
        s.squint = std::max(0.0f, s.squint - dt * 1.5f);
        const float surpriseTarget = bi == grabbed ? 1.0f : 0.0f;
        s.surprise += (surpriseTarget - s.surprise) * std::min(1.0f, dt * 8.0f);

        // Pupils behave like heavy beads: they sink against the felt
        // (proper) acceleration and wobble back on a damped spring
        Vector3 vel = s.hasPrev ? (b.center - s.prevCenter) * (1.0f / dt) : Vector3{0, 0, 0};
        Vector3 acc = s.hasPrev ? (vel - s.prevVel) * (1.0f / dt) : Vector3{0, 0, 0};
        float al = Len(acc);
        if (al > 200.0f) acc = acc * (200.0f / al);
        s.prevCenter = b.center;
        s.prevVel = vel;
        s.hasPrev = true;
        Vector3 felt = acc + Vector3{0, world.params.gravity, 0};

        const std::vector<Anchor>& anchors = gFaces[b.shape].eyes;
        for (size_t e = 0; e < anchors.size() && e < 2; e++) {
            Vector3 n = Norm(b.rot * anchors[e].restNormal);
            Vector3 ft = felt - n * Dot(felt, n);
            Vector3& o = s.pupil[e];
            Vector3& ov = s.pupilVel[e];
            Vector3 a = o * -K - ov * C - ft * G;
            ov += a * dt;
            o += ov * dt;
            o = o - n * Dot(o, n);
            float ol = Len(o);
            if (ol > 0.5f) {
                o = o * (0.5f / ol);
                Vector3 outward = o * (1.0f / 0.5f);
                float vo = Dot(ov, outward);
                if (vo > 0) ov = ov - outward * vo;
            }
        }
    }
    for (auto it = gState.begin(); it != gState.end();) {
        if (!it->second.seen) it = gState.erase(it);
        else ++it;
    }
}

void Draw(const World& world, const Camera3D& cam, Vector3 lightDir, Vector3 lightColor, Vector3 ambient) {
    if (!gReady) return;
    SetUniforms(Norm(lightDir), lightColor, ambient, cam.position);
    const Color sclera = {248, 247, 242, 255};
    const Color pupilCol = {16, 12, 20, 255};
    const Color shine = {255, 255, 255, 255};

    for (const Body& b : world.bodies) {
        if (b.shape >= (int)gFaces.size() || gFaces[b.shape].eyes.empty()) continue;
        const Face& face = gFaces[b.shape];
        EyeState st;
        auto it = gState.find(b.id);
        if (it != gState.end()) st = it->second;

        float closed = 0;
        if (st.blinkT >= 0) {
            float k = st.blinkT / kBlinkDuration;
            closed = k < 0.4f ? k / 0.4f : 1.0f - (k - 0.4f) / 0.6f;
        }
        closed = std::clamp(std::max(closed, st.squint * 0.65f), 0.0f, 1.0f);
        const float surprise = st.surprise;

        for (size_t e = 0; e < face.eyes.size() && e < 2; e++) {
            const Anchor& an = face.eyes[e];
            Vector3 p0, T, U, N;
            float stretch;
            if (!AnchorFrame(world, b, an, p0, T, U, N, stretch)) continue;
            float r = an.radius * b.scale * stretch * (1.0f + 0.18f * surprise);
            Vector3 center = p0 + N * (r * 0.28f);

            // Sclera, squashed from the top while blinking
            float sy = r * std::max(0.1f, 1.0f - 0.9f * closed);
            float sz = r * 0.62f;
            Vector3 cs = center - U * ((r - sy) * 0.55f);
            DrawEllipsoid(cs, T, U, N, Vector3{r, sy, sz}, sclera, 0.0f);

            // Pupil: looks a bit towards the camera plus the inertial wobble
            Vector3 toCam = Norm(cam.position - center);
            Vector3 look = (toCam - N * Dot(toCam, N)) * 0.3f + st.pupil[e];
            float ll = Len(look);
            if (ll > 0.5f) look = look * (0.5f / ll);
            float ox = Dot(look, T), oy = Dot(look, U);
            float rp = r * (0.56f - 0.2f * surprise);
            float zf = std::sqrt(std::max(0.0f, 1.0f - ox * ox - oy * oy));  // stay on the sclera surface
            Vector3 pc = cs + T * (ox * r) + U * (oy * sy) + N * (sz * zf - rp * 0.12f);
            Vector3 ps = {rp * (1.0f + 0.5f * closed), std::max(rp * 0.12f, rp * (1.0f - 0.92f * closed)), rp * 0.42f};
            DrawEllipsoid(pc, T, U, N, ps, pupilCol, 0.0f);

            // Catch-light
            if (closed < 0.55f) {
                float hr = rp * 0.3f;
                Vector3 hc = pc + T * (-0.38f * rp) + U * (0.4f * rp * (1.0f - closed)) + N * (rp * 0.33f);
                DrawEllipsoid(hc, T, U, N, Vector3{hr, hr, hr * 0.6f}, shine, 1.0f);
            }
        }

        // Mouth: a small smile, or a round "o" while being carried
        Vector3 p0, T, U, N;
        float stretch;
        if (face.hasMouth && AnchorFrame(world, b, face.mouth, p0, T, U, N, stretch)) {
            const float s = b.scale * stretch;
            if (surprise > 0.5f) {
                float mr = 0.022f * s * surprise;
                DrawEllipsoid(p0 + N * (0.004f * s), T, U, N, Vector3{mr * 0.85f, mr, mr * 0.45f}, pupilCol, 0.0f);
            } else {
                const float w = 0.032f * s, curve = 0.014f * s, br = 0.0085f * s;
                for (int i = -3; i <= 3; i++) {
                    float t = i / 3.0f;
                    Vector3 c = p0 + T * (t * w) + U * (curve * (t * t - 0.5f)) + N * (0.003f * s);
                    DrawEllipsoid(c, T, U, N, Vector3{br * 1.3f, br, br * 0.5f}, pupilCol, 0.0f);
                }
            }
        }
    }
}

}  // namespace eyes
