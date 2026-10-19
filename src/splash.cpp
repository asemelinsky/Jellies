#include "splash.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>
#include "rlgl.h"

namespace splash {
namespace {

const int kMaxDroplets = 300;
const int kMaxStains = 400;

struct Droplet {
    Vector3 p, v, color;
    float r;
};

struct Stain {
    Vector3 p, color;
    float radius, stretch, angle;
    float age, life, alpha;
};

struct BodyTrack {
    Vector3 lastTrail = {0, 0, 0};
    bool hasTrail = false;
    float cooldown = 0;
    bool seen = false;
};

std::vector<Droplet> gDrops;
std::vector<Stain> gStains;
std::unordered_map<int, BodyTrack> gTracks;
float gPlatformTop = 0, gPlatformHalf = 7;

Shader gStainShader = {}, gDropShader = {};
int gStainLoc[6], gDropLoc[4];

const char* kStainVS = R"(#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
out vec3 fragPos;
out vec2 fragUV;
out vec4 fragColor;
void main() {
    fragPos = vertexPosition;
    fragUV = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
})";

// pass 0: multiplicative tint (wet darkening), pass 1: additive gloss
const char* kStainFS = R"(#version 330
in vec3 fragPos;
in vec2 fragUV;
in vec4 fragColor;
uniform vec3 camPos;
uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 ambient;
uniform float platformHalf;
uniform int pass;
out vec4 finalColor;

float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y);
}

void main() {
    if (abs(fragPos.x) > platformHalf || abs(fragPos.z) > platformHalf) discard;
    // Wobbly outline from world-space noise
    float wob = noise(fragPos.xz * 5.0) * 0.7 + noise(fragPos.xz * 11.0) * 0.3;
    float d = length(fragUV) / (1.0 - 0.3 * wob);
    if (d > 1.0) discard;
    float shape = smoothstep(1.0, 0.72, d) * fragColor.a;
    float ring = smoothstep(0.6, 0.9, d) * (1.0 - smoothstep(0.9, 1.0, d));

    if (pass == 0) {
        vec3 base = fragColor.rgb;
        float lum = dot(base, vec3(0.3, 0.59, 0.11));
        vec3 wet = clamp(mix(vec3(lum), base, 1.4), 0.0, 1.0) * 0.62 + 0.26;
        wet *= 1.0 - 0.14 * ring;  // darker rim, like a drying drop
        finalColor = vec4(mix(vec3(1.0), wet, shape), 1.0);
    } else {
        // Thin dome: normal tilts towards the rim
        vec2 g = fragUV * (0.45 + 0.4 * d);
        vec3 N = normalize(vec3(g.x, 1.0, g.y));
        vec3 V = normalize(camPos - fragPos);
        vec3 L = normalize(lightDir);
        vec3 H = normalize(L + V);
        float nh = max(dot(N, H), 0.0);
        float spec = pow(nh, 120.0) * 2.2 + pow(nh, 24.0) * 0.18;
        float fres = pow(1.0 - max(dot(N, V), 0.0), 5.0);
        vec3 sheen = ambient * 0.07 * (1.0 - d) + fragColor.rgb * lightColor * 0.05;  // thin glossy film
        vec3 col = lightColor * spec * step(0.0, L.y) + ambient * fres * 0.6 + sheen;
        finalColor = vec4(col, shape);
    }
})";

const char* kDropVS = R"(#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
out vec3 fragPos;
out vec2 fragUV;
out vec3 fragAxis;
out vec4 fragColor;
void main() {
    fragPos = vertexPosition;
    fragUV = vertexTexCoord;
    fragAxis = vertexNormal;
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
})";

// Sphere impostor on a camera-facing quad, stretched along the velocity
const char* kDropFS = R"(#version 330
in vec3 fragPos;
in vec2 fragUV;
in vec3 fragAxis;
in vec4 fragColor;
uniform vec3 camPos;
uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 ambient;
out vec4 finalColor;
void main() {
    float r2 = dot(fragUV, fragUV);
    if (r2 > 1.0) discard;
    vec3 V = normalize(camPos - fragPos);
    vec3 a1 = normalize(fragAxis);
    vec3 a2 = normalize(cross(V, a1));
    vec3 N = normalize(a1 * fragUV.x + a2 * fragUV.y + V * sqrt(1.0 - r2));
    vec3 L = normalize(lightDir);
    float ndl = dot(N, L);
    vec3 base = fragColor.rgb;
    vec3 col = base * (ambient * 0.9 + lightColor * (max(ndl, 0.0) * 0.5 + 0.25));
    // Light shining through the droplet
    col += base * lightColor * pow(max(dot(V, -L), 0.0), 3.0) * 0.8;
    float spec = pow(max(dot(N, normalize(L + V)), 0.0), 80.0);
    col += lightColor * spec * 1.8;
    float fres = pow(1.0 - max(dot(N, V), 0.0), 3.0);
    col += ambient * fres * 0.5;
    float a = mix(0.72, 0.95, fres) * smoothstep(1.0, 0.85, r2) * fragColor.a;
    finalColor = vec4(col, a);
})";

void AddStain(Vector3 p, Vector3 color, float radius, float life, float alpha) {
    if ((int)gStains.size() >= kMaxStains) gStains.erase(gStains.begin());
    Stain s;
    s.p = {p.x, gPlatformTop, p.z};
    s.color = color;
    s.radius = radius;
    s.stretch = RandF(1.0f, 1.35f);
    s.angle = RandF(0.0f, 2.0f * PI);
    s.age = 0;
    s.life = life * RandF(0.85f, 1.15f);
    s.alpha = alpha;
    gStains.push_back(s);
}

void AddDroplet(Vector3 p, Vector3 v, Vector3 color, float r) {
    if ((int)gDrops.size() >= kMaxDroplets) return;
    gDrops.push_back({p, v, color, r});
}

bool OverPlatform(Vector3 p) { return std::fabs(p.x) < gPlatformHalf && std::fabs(p.z) < gPlatformHalf; }

void Burst(Vector3 origin, Vector3 color, float speed, float scale, int count, float upBias) {
    for (int i = 0; i < count; i++) {
        float a = RandF(0.0f, 2.0f * PI);
        float out = RandF(1.2f, 3.2f) * (0.6f + speed * 0.06f);
        Vector3 v = {std::cos(a) * out, RandF(2.0f, 5.0f) * upBias * (0.7f + speed * 0.04f), std::sin(a) * out};
        Vector3 p = origin + Vector3{std::cos(a), 0.0f, std::sin(a)} * (0.12f * scale) + Vector3{0, 0.05f, 0};
        AddDroplet(p, v, color, RandF(0.03f, 0.07f) * (0.7f + 0.3f * scale));
    }
}

}  // namespace

void Init() {
    gStainShader = LoadShaderFromMemory(kStainVS, kStainFS);
    gStainLoc[0] = GetShaderLocation(gStainShader, "camPos");
    gStainLoc[1] = GetShaderLocation(gStainShader, "lightDir");
    gStainLoc[2] = GetShaderLocation(gStainShader, "lightColor");
    gStainLoc[3] = GetShaderLocation(gStainShader, "ambient");
    gStainLoc[4] = GetShaderLocation(gStainShader, "platformHalf");
    gStainLoc[5] = GetShaderLocation(gStainShader, "pass");
    gDropShader = LoadShaderFromMemory(kDropVS, kDropFS);
    gDropLoc[0] = GetShaderLocation(gDropShader, "camPos");
    gDropLoc[1] = GetShaderLocation(gDropShader, "lightDir");
    gDropLoc[2] = GetShaderLocation(gDropShader, "lightColor");
    gDropLoc[3] = GetShaderLocation(gDropShader, "ambient");
}

void Shutdown() {
    UnloadShader(gStainShader);
    UnloadShader(gDropShader);
    gStainShader = {};
    gDropShader = {};
    Clear();
}

void Clear() {
    gDrops.clear();
    gStains.clear();
    gTracks.clear();
}

void Update(const World& world, float dt) {
    if (dt <= 0) return;
    gPlatformTop = world.params.platformTop;
    gPlatformHalf = world.params.platformHalf;
    const float top = gPlatformTop;

    for (auto& kv : gTracks) kv.second.seen = false;

    for (const Body& b : world.bodies) {
        BodyTrack& tr = gTracks[b.id];
        tr.seen = true;
        tr.cooldown = std::max(0.0f, tr.cooldown - dt);

        // Contact patch: the lowest particles of the body
        float minY = 1e9f;
        for (int i = b.first; i < b.first + b.count; i++) minY = std::min(minY, world.x[i].y);
        const float r = b.particleRadius;
        Vector3 p = {0, 0, 0}, vel = {0, 0, 0};
        int n = 0;
        for (int i = b.first; i < b.first + b.count; i++) {
            if (world.x[i].y < minY + 1.2f * r) {
                p += world.x[i];
                vel += world.v[i];
                n++;
            }
        }
        p = p * (1.0f / n);
        vel = vel * (1.0f / n);
        float footprint = 0;
        for (int i = b.first; i < b.first + b.count; i++)
            if (world.x[i].y < minY + 1.2f * r) {
                Vector3 d = world.x[i] - p;
                footprint = std::max(footprint, std::sqrt(d.x * d.x + d.z * d.z));
            }
        const bool grounded = minY <= top + r * 1.3f && OverPlatform(p);

        // Ground impact: puddle + droplets
        if (grounded && tr.cooldown <= 0 && b.impactGround > 4.0f) {
            const float s = b.impactGround;
            AddStain(p, b.color, std::min((footprint + r) * 0.95f + 0.03f * s * b.scale, 0.8f * b.scale), 6.0f,
                     std::min(0.22f + 0.02f * s, 0.42f));
            if (s > 4.0f) {
                int count = std::clamp((int)((s - 3.0f) * 1.4f * b.scale), 2, 12);
                Burst(p, b.color, s, b.scale, count, 1.0f);
            }
            tr.cooldown = 0.25f;
        }
        // Hard jelly-jelly hit: a few droplets from the body's top half
        if (tr.cooldown <= 0 && b.impactJelly > 5.0f) {
            const float R = world.Template(b).boundRadius * b.scale;
            int count = std::clamp((int)((b.impactJelly - 4.0f) * 0.8f), 1, 6);
            Vector3 o = b.center + Vector3{RandF(-0.3f, 0.3f), 0.7f, RandF(-0.3f, 0.3f)} * R;
            Burst(o, b.color, b.impactJelly, b.scale, count, 0.8f);
            tr.cooldown = 0.25f;
        }

        // Wet trail while sliding
        float hs = std::sqrt(vel.x * vel.x + vel.z * vel.z);
        if (grounded && hs > 0.6f) {
            Vector3 flat = {p.x, top, p.z};
            if (!tr.hasTrail) {
                tr.lastTrail = flat;
                tr.hasTrail = true;
            } else if (Len(flat - tr.lastTrail) >= 0.15f) {
                AddStain(flat, b.color, RandF(0.1f, 0.16f) * b.scale + footprint * 0.3f, 5.0f, 0.2f);
                tr.lastTrail = flat;
            }
        } else if (!grounded) {
            tr.hasTrail = false;
        } else {
            tr.lastTrail = {p.x, top, p.z};
        }
    }
    for (auto it = gTracks.begin(); it != gTracks.end();) {
        if (!it->second.seen) it = gTracks.erase(it);
        else ++it;
    }

    // Droplets: ballistic, leave a spot where they land
    const float g = world.params.gravity;
    for (size_t i = 0; i < gDrops.size();) {
        Droplet& d = gDrops[i];
        Vector3 prev = d.p;
        d.v.y -= g * dt;
        d.v = d.v * std::exp(-0.3f * dt);
        d.p += d.v * dt;
        bool dead = false;
        if (prev.y - d.r >= top - 0.05f && d.p.y - d.r <= top && OverPlatform(d.p)) {
            AddStain(d.p, d.color, d.r * RandF(1.8f, 2.6f), 5.0f, 0.35f);
            dead = true;
        } else if (d.p.y < -30.0f) {
            dead = true;
        }
        if (dead) {
            d = gDrops.back();
            gDrops.pop_back();
        } else {
            i++;
        }
    }

    // Age stains; a fresh stain spreads out quickly
    for (Stain& s : gStains) s.age += dt;
    gStains.erase(std::remove_if(gStains.begin(), gStains.end(), [](const Stain& s) { return s.age >= s.life; }),
                  gStains.end());
}

void DrawStains(const Camera3D& cam, Vector3 lightDir, Vector3 lightColor, Vector3 ambient) {
    if (gStains.empty() || gStainShader.id == 0) return;
    SetShaderValue(gStainShader, gStainLoc[0], &cam.position, SHADER_UNIFORM_VEC3);
    SetShaderValue(gStainShader, gStainLoc[1], &lightDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(gStainShader, gStainLoc[2], &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(gStainShader, gStainLoc[3], &ambient, SHADER_UNIFORM_VEC3);
    SetShaderValue(gStainShader, gStainLoc[4], &gPlatformHalf, SHADER_UNIFORM_FLOAT);

    rlDrawRenderBatchActive();
    rlDisableDepthTest();  // depth writes are off too while the test is disabled
    rlDisableBackfaceCulling();
    for (int pass = 0; pass < 2; pass++) {
        SetShaderValue(gStainShader, gStainLoc[5], &pass, SHADER_UNIFORM_INT);
        BeginBlendMode(pass == 0 ? BLEND_MULTIPLIED : BLEND_ADDITIVE);
        BeginShaderMode(gStainShader);
        rlBegin(RL_QUADS);
        for (size_t i = 0; i < gStains.size(); i++) {
            const Stain& s = gStains[i];
            float t = s.age / s.life;
            float fade = s.alpha * (1.0f - std::pow(t, 2.0f)) * std::min(1.0f, s.age * 12.0f + 0.3f);
            float grow = 0.65f + 0.35f * std::min(1.0f, s.age * 7.0f);
            float rx = s.radius * grow * s.stretch, rz = s.radius * grow;
            Vector3 ax = {std::cos(s.angle) * rx, 0, std::sin(s.angle) * rx};
            Vector3 az = {-std::sin(s.angle) * rz, 0, std::cos(s.angle) * rz};
            Vector3 c = {s.p.x, s.p.y + 0.002f + 0.00001f * i, s.p.z};
            rlColor4ub((unsigned char)(s.color.x * 255), (unsigned char)(s.color.y * 255), (unsigned char)(s.color.z * 255),
                       (unsigned char)(std::clamp(fade, 0.0f, 1.0f) * 255));
            const float uv[4][2] = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}};
            for (auto& q : uv) {
                Vector3 v = c + ax * q[0] + az * q[1];
                rlTexCoord2f(q[0], q[1]);
                rlVertex3f(v.x, v.y, v.z);
            }
        }
        rlEnd();
        EndShaderMode();
        EndBlendMode();
    }
    rlEnableBackfaceCulling();
    rlEnableDepthTest();
}

void DrawDroplets(const Camera3D& cam, Vector3 lightDir, Vector3 lightColor, Vector3 ambient) {
    if (gDrops.empty() || gDropShader.id == 0) return;
    SetShaderValue(gDropShader, gDropLoc[0], &cam.position, SHADER_UNIFORM_VEC3);
    SetShaderValue(gDropShader, gDropLoc[1], &lightDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(gDropShader, gDropLoc[2], &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(gDropShader, gDropLoc[3], &ambient, SHADER_UNIFORM_VEC3);

    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    rlDisableBackfaceCulling();
    BeginShaderMode(gDropShader);
    rlBegin(RL_QUADS);
    for (const Droplet& d : gDrops) {
        Vector3 toCam = Norm(cam.position - d.p);
        // Stretch along the screen-projected velocity
        Vector3 vp = d.v - toCam * Dot(d.v, toCam);
        float sp = Len(vp);
        Vector3 a1 = sp > 0.3f ? vp * (1.0f / sp) : Norm(Cross(Vector3{0, 1, 0}, toCam));
        if (Len2(a1) < 1e-6f) a1 = {1, 0, 0};
        Vector3 a2 = Norm(Cross(toCam, a1));
        float len1 = d.r * (1.0f + std::min(sp * 0.06f, 1.2f));
        Vector3 e1 = a1 * len1, e2 = a2 * d.r;
        rlColor4ub((unsigned char)(d.color.x * 255), (unsigned char)(d.color.y * 255), (unsigned char)(d.color.z * 255), 255);
        rlNormal3f(a1.x, a1.y, a1.z);
        const float uv[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
        for (auto& q : uv) {
            Vector3 v = d.p + e1 * q[0] + e2 * q[1];
            rlTexCoord2f(q[0], q[1]);
            rlVertex3f(v.x, v.y, v.z);
        }
    }
    rlEnd();
    EndShaderMode();
    rlEnableBackfaceCulling();
    rlEnableDepthMask();
}

}  // namespace splash
