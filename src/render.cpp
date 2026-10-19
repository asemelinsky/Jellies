#include "render.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rlgl.h"
#if defined(PLATFORM_WEB)
#include <GLES3/gl3.h>
#endif
#include "eyes.h"
#include "glsl.h"
#include "splash.h"

namespace {

// OpenGL enums for rlgl's custom blend helpers
constexpr int kGlZero = 0;
constexpr int kGlOne = 1;
constexpr int kGlSrcAlpha = 0x0302;
constexpr int kGlOneMinusSrcAlpha = 0x0303;
constexpr int kGlDstColor = 0x0306;
constexpr int kGlFuncAdd = 0x8006;
constexpr int kGlMax = 0x8008;

constexpr float kLightHalfExtent = 12.0f;  // half size of the shadow frustum (m)
constexpr float kLightDistance = 40.0f;
constexpr float kLightNear = 1.0f;
constexpr float kLightFar = 80.0f;
constexpr float kSeaLevel = -28.0f;

// --- shaders -----------------------------------------------------------------

const char* kLitVS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 fragPos;
out vec3 fragNormal;
void main() {
    fragPos = vec3(matModel * vec4(vertexPosition, 1.0));
    fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char* kDepthVS = R"(#version 330
in vec3 vertexPosition;
uniform mat4 mvp;
void main() { gl_Position = mvp * vec4(vertexPosition, 1.0); }
)";

const char* kDepthFS = R"(#version 330
out vec4 finalColor;
void main() { finalColor = vec4(1.0); }
)";

// Jelly colour shadow: rgb = transmittance (multiplied), a = caustic focus
// (max-blended). Faces looking straight at the light focus the most light.
const char* kTintFS = R"(#version 330
in vec3 fragPos;
in vec3 fragNormal;
uniform vec3 tint;
uniform vec3 lightDir;
uniform float causticGain;
out vec4 finalColor;
void main() {
    float ndl = clamp(dot(normalize(fragNormal), lightDir), 0.0, 1.0);
    finalColor = vec4(tint, pow(ndl, 3.0) * causticGain);
}
)";

// Lighting helpers shared by every lit surface
const char* kCommonGLSL = R"(
uniform sampler2D shadowOpaque;
uniform sampler2D shadowJellyDepth;
uniform sampler2D shadowJellyColor;
uniform mat4 lightVP;
uniform float shadowTexel;
uniform float shadowRadius;
uniform int shadowTaps;
uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 ambientSky;
uniform vec3 ambientGround;
uniform vec3 zenithColor;
uniform vec3 horizonColor;
uniform vec3 camPos;

const vec2 kPoisson[16] = vec2[16](
    vec2(-0.94201624, -0.39906216), vec2(0.94558609, -0.76890725),
    vec2(-0.09418410, -0.92938870), vec2(0.34495938, 0.29387760),
    vec2(-0.91588581, 0.45771432), vec2(-0.81544232, -0.87912464),
    vec2(-0.38277543, 0.27676845), vec2(0.97484398, 0.75648379),
    vec2(0.44323325, -0.97511554), vec2(0.53742981, -0.47373420),
    vec2(-0.26496911, -0.41893023), vec2(0.79197514, 0.19090188),
    vec2(-0.24188840, 0.99706507), vec2(-0.81409955, 0.91437590),
    vec2(0.19984126, 0.78641367), vec2(0.14383161, -0.14100790));

// Set by ShadowLight: caustic focus of the jellies in front of the light and
// the light-space position (for the caustic pattern)
float gCaustic = 0.0;
vec2 gLightUV = vec2(0.0);

// Light reaching worldPos: rgb transmittance through jellies, already
// multiplied by the opaque visibility. opaqueVis = opaque-only visibility.
vec3 ShadowLight(vec3 worldPos, vec3 N, out float opaqueVis) {
    float ndl = clamp(dot(N, lightDir), 0.0, 1.0);
    vec3 p = worldPos + N * (0.015 + 0.05 * (1.0 - ndl));
    vec4 lp = lightVP * vec4(p, 1.0);
    vec3 c = lp.xyz / lp.w * 0.5 + 0.5;
    opaqueVis = 1.0;
    gCaustic = 0.0;
    gLightUV = c.xy;
    if (c.x <= 0.002 || c.x >= 0.998 || c.y <= 0.002 || c.y >= 0.998 || c.z >= 1.0) return vec3(1.0);
    float a = 6.2831853 * fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    mat2 rot = mat2(cos(a), sin(a), -sin(a), cos(a));
    float vis = 0.0, caus = 0.0;
    vec3 tint = vec3(0.0);
    for (int i = 0; i < 16; i++) {
        if (i >= shadowTaps) break;
        vec2 o;
        if (shadowTaps == 4) o = vec2(float(i % 2) - 0.5, float(i / 2) - 0.5);
        else if (shadowTaps == 9) o = vec2(float(i % 3) - 1.0, float(i / 3) - 1.0);
        else o = rot * kPoisson[i];
        vec2 uv = c.xy + o * shadowRadius * shadowTexel;
        float lit = (c.z - 0.0006 > texture(shadowOpaque, uv).r) ? 0.0 : 1.0;
        bool behind = c.z - 0.0009 > texture(shadowJellyDepth, uv).r;
        vec4 jc = behind ? texture(shadowJellyColor, uv) : vec4(1.0, 1.0, 1.0, 0.0);
        vis += lit;
        tint += jc.rgb * lit;
        caus += jc.a * lit;
    }
    float n = float(min(shadowTaps, 16));
    opaqueVis = vis / n;
    gCaustic = caus / n;
    return tint / n;
}

vec3 Ambient(vec3 N) { return mix(ambientGround, ambientSky, N.y * 0.5 + 0.5); }
// Soft highlight roll-off so bright noon light does not clip to white
vec3 Tone(vec3 c) { return vec3(1.0) - exp(-c * 1.45); }

// Animated web of bright caustic lines (after Dave Hoskins' "Tileable Water Caustic")
float CausticPattern(vec2 p, float t) {
    vec2 i = p;
    float c = 1.0;
    const float inten = 0.005;
    for (int n = 0; n < 4; n++) {
        float tt = t * (1.0 - (3.5 / float(n + 1)));
        i = p + vec2(cos(tt - i.x) + sin(tt + i.y), sin(tt - i.y) + cos(tt + i.x));
        c += 1.0 / length(vec2(p.x / (sin(i.x + tt) / inten), p.y / (cos(i.y + tt) / inten)));
    }
    c /= 4.0;
    c = 1.17 - pow(c, 1.4);
    return clamp(pow(abs(c), 8.0), 0.0, 2.0);
}

// Coloured light focused by the jellies in front of the light
vec3 Caustics(vec3 transmit, float ndl, float strength, float time) {
    if (gCaustic < 0.002 || strength <= 0.0) return vec3(0.0);
    vec3 hue = transmit / max(max(transmit.r, transmit.g), max(transmit.b, 1e-3));
    hue = pow(hue, vec3(1.6));
    float pat = CausticPattern(gLightUV * 125.0, time * 0.9);
    float focus = gCaustic * gCaustic;  // concentrate towards the centre of the pool
    return lightColor * hue * (focus * 0.9 + gCaustic * pat * 2.4) * ndl * strength;
}
vec3 SkyColor(vec3 d) { return mix(horizonColor, zenithColor, pow(clamp(d.y, 0.0, 1.0), 0.55)); }
)";

const char* kSkyFS = R"(
in vec3 fragPos;
in vec3 fragNormal;
out vec4 finalColor;
uniform vec3 sunDir;
uniform float night;
uniform float time;

float Hash(vec3 p) {
    p = fract(p * 0.3183099 + 0.1);
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

void main() {
    vec3 d = normalize(fragPos - camPos);
    float h = d.y;
    vec3 col = SkyColor(d);

    float sunUp = smoothstep(-0.12, 0.04, sunDir.y);
    vec3 sunTint = mix(vec3(1.0, 0.55, 0.25), vec3(1.0, 0.93, 0.8), smoothstep(0.0, 0.45, sunDir.y));
    float s = max(dot(d, sunDir), 0.0);
    col += sunTint * (pow(s, 8.0) * 0.3 + pow(s, 90.0) * 0.55) * sunUp;
    col = mix(col, vec3(1.0, 0.97, 0.9) * 1.25, smoothstep(0.99935, 0.99965, s) * sunUp);

    vec3 md = -sunDir;
    float m = max(dot(d, md), 0.0);
    float moonUp = smoothstep(-0.05, 0.08, md.y) * night;
    col += vec3(0.35, 0.42, 0.6) * pow(m, 40.0) * 0.25 * moonUp;
    col = mix(col, vec3(0.92, 0.94, 1.0), smoothstep(0.99955, 0.9998, m) * moonUp);

    if (night > 0.01 && h > 0.0) {
        vec3 sp = d * 180.0;
        vec3 cell = floor(sp);
        float r = Hash(cell);
        if (r > 0.992) {
            vec3 f = fract(sp) - 0.5;
            float star = smoothstep(0.3, 0.0, length(f));
            float tw = 0.65 + 0.35 * sin(time * 3.0 + r * 120.0);
            col += vec3(star * tw) * night * smoothstep(0.0, 0.25, h);
        }
    }
    finalColor = vec4(col, 1.0);
}
)";

const char* kPlatformFS = R"(
in vec3 fragPos;
in vec3 fragNormal;
out vec4 finalColor;
uniform float platformHalf;
uniform float causticStrength;
uniform float time;

void main() {
    vec3 N = normalize(fragNormal);
    vec3 base;
    if (N.y > 0.5) {
        vec2 t = fragPos.xz;
        float checker = mod(floor(t.x) + floor(t.y), 2.0);
        base = mix(vec3(0.97, 0.90, 0.78), vec3(0.88, 0.71, 0.50), checker);
        vec2 f = abs(fract(t) - 0.5);
        float grout = smoothstep(0.465, 0.49, max(f.x, f.y));
        base *= 1.0 - 0.16 * grout;
        float edge = max(abs(fragPos.x), abs(fragPos.z));
        base = mix(base, vec3(0.66, 0.45, 0.30), smoothstep(platformHalf - 0.32, platformHalf - 0.28, edge));
    } else {
        float stripe = smoothstep(0.42, 0.5, abs(fract(fragPos.y * 2.5) - 0.5));
        base = vec3(0.60, 0.40, 0.26) * (1.0 - 0.12 * stripe);
        if (N.y < -0.5) base *= 0.7;
    }
    float ov;
    vec3 sh = ShadowLight(fragPos, N, ov);
    float ndl = max(dot(N, lightDir), 0.0);
    vec3 V = normalize(camPos - fragPos);
    vec3 H = normalize(lightDir + V);
    float spec = pow(max(dot(N, H), 0.0), 48.0) * 0.12;
    vec3 col = base * (Ambient(N) * 0.85 + (lightColor * ndl * sh + Caustics(sh, ndl, causticStrength, time)) * 0.9) +
               lightColor * sh * spec;
    finalColor = vec4(Tone(col), 1.0);
}
)";

const char* kSeaFS = R"(
in vec3 fragPos;
in vec3 fragNormal;
out vec4 finalColor;
uniform float time;

void main() {
    vec3 V = normalize(camPos - fragPos);
    vec2 p = fragPos.xz;
    vec2 g = vec2(0.0);
    vec2 dirs[5] = vec2[5](vec2(0.94, 0.34), vec2(-0.41, 0.91), vec2(0.62, -0.78), vec2(-0.87, -0.49), vec2(0.17, 0.98));
    float freq = 0.11, amp = 0.06, speed = 0.8;
    for (int i = 0; i < 5; i++) {
        float ph = dot(dirs[i], p) * freq + time * speed + float(i) * 1.7;
        g += dirs[i] * cos(ph) * amp;
        freq *= 1.83; amp *= 0.62; speed *= 1.27;
    }
    float far = clamp(length(fragPos - camPos) / 300.0, 0.0, 1.0);
    vec3 N = normalize(vec3(g.x * (1.0 - far * 0.7), 1.0, g.y * (1.0 - far * 0.7)));
    float ndv = max(dot(N, V), 0.0);
    float F = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);
    vec3 R = reflect(-V, N);
    R.y = abs(R.y);
    float ov;
    vec3 sh = ShadowLight(fragPos, vec3(0.0, 1.0, 0.0), ov);
    vec3 deep = vec3(0.03, 0.20, 0.27) * (ambientSky * 0.9 + lightColor * sh * max(dot(N, lightDir), 0.0) * 0.6);
    vec3 col = mix(deep, SkyColor(R), F);
    vec3 H = normalize(lightDir + V);
    col += lightColor * sh * pow(max(dot(N, H), 0.0), 220.0) * 1.6;
    float dist = length(fragPos - camPos);
    float fog = 1.0 - exp(-dist * 0.0035);
    col = mix(Tone(col), horizonColor, fog);
    finalColor = vec4(col, 1.0);
}
)";

const char* kJellyFS = R"(
in vec3 fragPos;
in vec3 fragNormal;
out vec4 finalColor;
uniform sampler2D sceneColor;
uniform mat4 viewProj;
uniform vec3 jellyColor;
uniform float jellySize;
uniform float highlight;

void main() {
    vec3 N = normalize(fragNormal);
    vec3 V = normalize(camPos - fragPos);
    float ndv = dot(N, V);
    if (ndv < 0.0) {
        // deformation can tilt a normal away from the viewer; bend it back
        N = normalize(N - V * (ndv * 1.05));
        ndv = dot(N, V);
    }
    ndv = clamp(ndv, 0.0, 1.0);
    vec3 L = lightDir;
    float F = 0.04 + 0.96 * pow(1.0 - ndv, 5.0);

    // Screen-space refraction of the opaque scene behind the jelly
    vec3 rd = refract(-V, N, 1.0 / 1.35);
    vec4 clip = viewProj * vec4(fragPos + rd * (0.6 * jellySize + 0.4), 1.0);
    vec2 uv = clamp(clip.xy / clip.w * 0.5 + 0.5, vec2(0.001), vec2(0.999));
    vec3 bg = texture(sceneColor, uv).rgb;
    float thick = jellySize * (0.25 + 0.75 * ndv);
    vec3 absorb = (1.0 - jellyColor) * 2.2 + 0.08;
    vec3 refr = bg * exp(-absorb * thick);

    float ov;
    vec3 sh = ShadowLight(fragPos, N, ov);
    float ndl = dot(N, L);
    float wrap = clamp((ndl + 0.4) / 1.4, 0.0, 1.0);
    vec3 body = jellyColor * (Ambient(N) * 0.55 + lightColor * sh * wrap * 0.55);

    // Fake subsurface scattering: glows when the light is behind the jelly
    vec3 Ht = normalize(L + N * 0.35);
    float back = pow(clamp(dot(V, -Ht), 0.0, 1.0), 4.0);
    float lightLum = dot(lightColor, vec3(0.333));
    vec3 glow = jellyColor * (lightColor * ov * back * 1.7 + jellyColor * lightLum * 0.06);

    vec3 H = normalize(L + V);
    float nh = max(dot(N, H), 0.0);
    vec3 spec = lightColor * sh * (pow(nh, 160.0) * 2.4 + pow(nh, 24.0) * 0.08);

    vec3 R = reflect(-V, N);
    vec3 col = refr * 0.7 + body * 0.75 + jellyColor * (0.05 + 0.07 * (1.0 - ndv));
    col = mix(col, SkyColor(R), F * 0.85) + glow + spec;
    col += jellyColor * highlight * (0.25 + 0.75 * pow(1.0 - ndv, 2.0)) * 0.55;
    finalColor = vec4(col, mix(0.8, 0.95, F));
}
)";

// --- helpers -----------------------------------------------------------------

struct ShaderBox {
    Shader sh = {};
    Material mat = {};
    std::unordered_map<std::string, int> locs;

    bool Load(const char* vs, const std::string& fs) {
        const std::string v = PortGLSL(vs), f = PortGLSL(fs.c_str());
        sh = LoadShaderFromMemory(v.c_str(), f.c_str());
        if (!IsShaderValid(sh)) return false;
        mat = LoadMaterialDefault();
        mat.shader = sh;
        return true;
    }
    void Unload() {
        if (sh.id > 0) UnloadShader(sh);
        if (mat.maps) RL_FREE(mat.maps);
        sh = {};
        mat = {};
        locs.clear();
    }
    int L(const char* name) {
        auto it = locs.find(name);
        if (it != locs.end()) return it->second;
        int l = GetShaderLocation(sh, name);
        locs.emplace(name, l);
        return l;
    }
    // Route a material map slot (1..4) to a sampler uniform
    void BindSlot(int slot, const char* sampler) { sh.locs[SHADER_LOC_MAP_DIFFUSE + slot] = L(sampler); }
    void SetTexture(int slot, unsigned id) { mat.maps[slot].texture.id = id; }
    void F(const char* n, float v) { SetShaderValue(sh, L(n), &v, SHADER_UNIFORM_FLOAT); }
    void I(const char* n, int v) { SetShaderValue(sh, L(n), &v, SHADER_UNIFORM_INT); }
    void V2(const char* n, Vector2 v) { SetShaderValue(sh, L(n), &v, SHADER_UNIFORM_VEC2); }
    void V3(const char* n, Vector3 v) { SetShaderValue(sh, L(n), &v, SHADER_UNIFORM_VEC3); }
    void M(const char* n, Matrix m) { SetShaderValueMatrix(sh, L(n), m); }
};

std::string WithCommon(const char* body) { return std::string("#version 330\n") + kCommonGLSL + body; }

// WebGL 2 only accepts sized depth formats, while raylib requests an unsized
// one there, so the browser build creates its depth textures itself
unsigned LoadDepthTexture(int w, int h) {
#if defined(PLATFORM_WEB)
    GLuint id = 0;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return id;
#else
    return rlLoadTextureDepth(w, h, false);
#endif
}

unsigned LoadColorTexture(int w, int h, bool linear) {
    unsigned id = rlLoadTexture(nullptr, w, h, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, 1);
    int filter = linear ? RL_TEXTURE_FILTER_LINEAR : RL_TEXTURE_FILTER_NEAREST;
    rlTextureParameters(id, RL_TEXTURE_MIN_FILTER, filter);
    rlTextureParameters(id, RL_TEXTURE_MAG_FILTER, filter);
    rlTextureParameters(id, RL_TEXTURE_WRAP_S, RL_TEXTURE_WRAP_CLAMP);
    rlTextureParameters(id, RL_TEXTURE_WRAP_T, RL_TEXTURE_WRAP_CLAMP);
    return id;
}

RenderTexture2D MakeTarget(unsigned color, unsigned depth, int w, int h) {
    RenderTexture2D rt = {};
    rt.id = rlLoadFramebuffer();
    rt.texture = {color, w, h, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8};
    rt.depth = {depth, w, h, 1, 19};
    rlFramebufferAttach(rt.id, color, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    rlFramebufferAttach(rt.id, depth, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(rt.id)) TraceLog(LOG_WARNING, "JELLY: framebuffer %u incomplete", rt.id);
    return rt;
}

Vector3 Mix3(Vector3 a, Vector3 b, float t) { return a + (b - a) * t; }

float Smooth(float e0, float e1, float x) {
    float t = std::min(std::max((x - e0) / (e1 - e0), 0.0f), 1.0f);
    return t * t * (3 - 2 * t);
}

void SkinBody(const World& w, const Body& b, const ShapeTemplate& st, Mesh& m) {
    const PhysicsParams& P = w.params;
    const bool flatten = b.center.y > P.platformTop;
    const float floorY = P.platformTop + 0.003f;
    float* vtx = m.vertices;
    for (int i = 0; i < m.vertexCount; i++) {
        const auto& id = st.skinIdx[i];
        const auto& wt = st.skinW[i];
        Vector3 p = w.x[b.first + id[0]] * wt[0] + w.x[b.first + id[1]] * wt[1] + w.x[b.first + id[2]] * wt[2] +
                    w.x[b.first + id[3]] * wt[3] + b.rot * (st.skinOffset[i] * b.scale);
        if (flatten && p.y < floorY && std::fabs(p.x) < P.platformHalf && std::fabs(p.z) < P.platformHalf) p.y = floorY;
        vtx[3 * i + 0] = p.x;
        vtx[3 * i + 1] = p.y;
        vtx[3 * i + 2] = p.z;
    }
    float* nrm = m.normals;
    std::memset(nrm, 0, sizeof(float) * 3 * m.vertexCount);
    const unsigned short* idx = m.indices;
    for (int t = 0; t < m.triangleCount; t++) {
        int a = idx[3 * t], c1 = idx[3 * t + 1], c2 = idx[3 * t + 2];
        Vector3 pa = {vtx[3 * a], vtx[3 * a + 1], vtx[3 * a + 2]};
        Vector3 pb = {vtx[3 * c1], vtx[3 * c1 + 1], vtx[3 * c1 + 2]};
        Vector3 pc = {vtx[3 * c2], vtx[3 * c2 + 1], vtx[3 * c2 + 2]};
        Vector3 n = Cross(pb - pa, pc - pa);
        for (int k : {a, c1, c2}) {
            nrm[3 * k] += n.x;
            nrm[3 * k + 1] += n.y;
            nrm[3 * k + 2] += n.z;
        }
    }
    for (int i = 0; i < m.vertexCount; i++) {
        Vector3 n = Norm({nrm[3 * i], nrm[3 * i + 1], nrm[3 * i + 2]});
        nrm[3 * i] = n.x;
        nrm[3 * i + 1] = n.y;
        nrm[3 * i + 2] = n.z;
    }
}

}  // namespace

const char* ShadowQualityName(int q) {
    switch (q) {
        case SHADOW_LOW: return "Низька";
        case SHADOW_MEDIUM: return "Середня";
        case SHADOW_HIGH: return "Висока";
        default: return "Ультра";
    }
}

SkyState ComputeSky(float dayPhase) {
    float ph = dayPhase - std::floor(dayPhase);
    // Day takes 75% of the cycle, night the rest
    float ang = ph < 0.75f ? PI * ph / 0.75f : PI + PI * (ph - 0.75f) / 0.25f;
    const float tilt = 30.0f * DEG2RAD;  // max sun elevation = 60 degrees
    Vector3 east = {1, 0, 0};
    Vector3 arc = {0, std::cos(tilt), -std::sin(tilt)};
    SkyState s;
    s.sunDir = Norm(east * std::cos(ang) + arc * std::sin(ang));
    const float e = s.sunDir.y;

    const float day = Smooth(-0.12f, 0.2f, e);
    const float sunset = (1.0f - Smooth(0.0f, 0.35f, e)) * Smooth(-0.22f, 0.0f, e);
    s.night = 1.0f - Smooth(-0.25f, 0.05f, e);

    s.zenith = Mix3({0.015f, 0.025f, 0.08f}, {0.20f, 0.44f, 0.86f}, day);
    s.zenith = Mix3(s.zenith, {0.28f, 0.26f, 0.52f}, sunset * 0.35f);
    s.horizon = Mix3({0.05f, 0.07f, 0.16f}, {0.70f, 0.83f, 0.95f}, day);
    s.horizon = Mix3(s.horizon, {1.0f, 0.56f, 0.32f}, sunset * 0.75f);

    const float sunI = Smooth(-0.02f, 0.12f, e);
    const float moonI = Smooth(-0.02f, 0.12f, -e) * 0.35f;
    Vector3 sunCol = Mix3({1.0f, 0.52f, 0.24f}, {1.0f, 0.95f, 0.86f}, Smooth(0.04f, 0.45f, e)) * 1.1f;
    if (e >= 0) {
        s.lightDir = s.sunDir;
        s.lightColor = sunCol * sunI;
    } else {
        s.lightDir = s.sunDir * -1.0f;
        s.lightColor = Vector3{0.55f, 0.65f, 0.95f} * moonI;
    }

    s.ambientSky = Mix3({0.10f, 0.12f, 0.22f}, {0.42f, 0.50f, 0.62f}, day);
    s.ambientSky = Mix3(s.ambientSky, {0.55f, 0.42f, 0.40f}, sunset * 0.3f);
    s.ambientGround = Mix3({0.05f, 0.05f, 0.08f}, {0.30f, 0.27f, 0.22f}, day);
    return s;
}

struct Renderer::Impl {
    int width = 0, height = 0;
    unsigned sceneDepth = 0, colorA = 0, colorB = 0;
    RenderTexture2D rtA = {}, rtB = {};

    int shadowSize = 0, taps = 16;
    float softness = 0.04f;  // PCF radius in metres
    float caustics = 1.0f;   // 0 disables coloured caustics
    unsigned shOpaqueDepth = 0, shOpaqueColor = 0, shJellyDepth = 0, shJellyColor = 0;
    RenderTexture2D rtShOpaque = {}, rtShJelly = {};

    Mesh platform = {}, sky = {}, sea = {};
    float platHalf = -1, platTop = 0, platBottom = 0;

    ShaderBox depth, tint, skyS, platformS, seaS, jelly;

    void CreateSceneTargets(int w, int h) {
        DestroySceneTargets();
        width = w;
        height = h;
        sceneDepth = LoadDepthTexture(w, h);
        colorA = LoadColorTexture(w, h, true);
        colorB = LoadColorTexture(w, h, true);
        rtA = MakeTarget(colorA, sceneDepth, w, h);
        rtB = MakeTarget(colorB, sceneDepth, w, h);
    }
    void DestroySceneTargets() {
        if (rtA.id) rlUnloadFramebuffer(rtA.id);
        if (rtB.id) rlUnloadFramebuffer(rtB.id);
        if (colorA) rlUnloadTexture(colorA);
        if (colorB) rlUnloadTexture(colorB);
        if (sceneDepth) rlUnloadTexture(sceneDepth);
        rtA = rtB = {};
        colorA = colorB = sceneDepth = 0;
    }

    void CreateShadowTargets(int quality) {
        DestroyShadowTargets();
        switch (quality) {
            case SHADOW_LOW: shadowSize = 1024; taps = 4; softness = 0.035f; caustics = 0.0f; break;
            case SHADOW_MEDIUM: shadowSize = 2048; taps = 9; softness = 0.03f; caustics = 1.0f; break;
            case SHADOW_HIGH: shadowSize = 2048; taps = 16; softness = 0.045f; caustics = 1.0f; break;
            default: shadowSize = 4096; taps = 16; softness = 0.05f; caustics = 1.0f; break;
        }
        const int s = shadowSize;
        shOpaqueDepth = LoadDepthTexture(s, s);
        shOpaqueColor = LoadColorTexture(s, s, false);
        shJellyDepth = LoadDepthTexture(s, s);
        shJellyColor = LoadColorTexture(s, s, true);
        rtShOpaque = MakeTarget(shOpaqueColor, shOpaqueDepth, s, s);
        rtShJelly = MakeTarget(shJellyColor, shJellyDepth, s, s);
    }
    void DestroyShadowTargets() {
        if (rtShOpaque.id) rlUnloadFramebuffer(rtShOpaque.id);
        if (rtShJelly.id) rlUnloadFramebuffer(rtShJelly.id);
        for (unsigned* t : {&shOpaqueDepth, &shOpaqueColor, &shJellyDepth, &shJellyColor}) {
            if (*t) rlUnloadTexture(*t);
            *t = 0;
        }
        rtShOpaque = rtShJelly = {};
    }

    void EnsurePlatform(const PhysicsParams& P) {
        if (platHalf == P.platformHalf && platTop == P.platformTop && platBottom == P.platformBottom) return;
        if (platform.vaoId) UnloadMesh(platform);
        platHalf = P.platformHalf;
        platTop = P.platformTop;
        platBottom = P.platformBottom;
        platform = GenMeshCube(2 * platHalf, platTop - platBottom, 2 * platHalf);
    }
    Matrix PlatformTransform() const { return MatrixTranslate(0, 0.5f * (platTop + platBottom), 0); }

    // Uniforms shared by every lit shader
    void SetCommon(ShaderBox& s, const SkyState& sky, Vector3 camPos, Matrix lightVP) {
        s.M("lightVP", lightVP);
        s.F("shadowTexel", 1.0f / shadowSize);
        s.F("shadowRadius", softness / (2.0f * kLightHalfExtent / shadowSize));
        s.I("shadowTaps", taps);
        s.V3("lightDir", sky.lightDir);
        s.V3("lightColor", sky.lightColor);
        s.V3("ambientSky", sky.ambientSky);
        s.V3("ambientGround", sky.ambientGround);
        s.V3("zenithColor", sky.zenith);
        s.V3("horizonColor", sky.horizon);
        s.V3("camPos", camPos);
        s.SetTexture(1, shOpaqueDepth);
        s.SetTexture(2, shJellyDepth);
        s.SetTexture(3, shJellyColor);
    }
};

bool Renderer::Init(int width, int height) {
    impl_ = new Impl();
    Impl& R = *impl_;
    bool ok = true;
    ok &= R.depth.Load(kDepthVS, kDepthFS);
    ok &= R.tint.Load(kLitVS, kTintFS);
    ok &= R.skyS.Load(kLitVS, WithCommon(kSkyFS));
    ok &= R.platformS.Load(kLitVS, WithCommon(kPlatformFS));
    ok &= R.seaS.Load(kLitVS, WithCommon(kSeaFS));
    ok &= R.jelly.Load(kLitVS, WithCommon(kJellyFS));
    if (!ok) return false;
    for (ShaderBox* s : {&R.skyS, &R.platformS, &R.seaS, &R.jelly}) {
        s->BindSlot(1, "shadowOpaque");
        s->BindSlot(2, "shadowJellyDepth");
        s->BindSlot(3, "shadowJellyColor");
    }
    R.jelly.BindSlot(4, "sceneColor");

    R.sky = GenMeshSphere(1.0f, 24, 32);
    R.sea = GenMeshPlane(1600.0f, 1600.0f, 1, 1);
    R.CreateSceneTargets(width, height);
    R.CreateShadowTargets(shadowQuality_);
    return true;
}

void Renderer::Shutdown() {
    if (!impl_) return;
    Impl& R = *impl_;
    R.DestroySceneTargets();
    R.DestroyShadowTargets();
    if (R.platform.vaoId) UnloadMesh(R.platform);
    UnloadMesh(R.sky);
    UnloadMesh(R.sea);
    for (ShaderBox* s : {&R.depth, &R.tint, &R.skyS, &R.platformS, &R.seaS, &R.jelly}) s->Unload();
    delete impl_;
    impl_ = nullptr;
}

void Renderer::Resize(int width, int height) {
    if (!impl_ || width <= 0 || height <= 0) return;
    if (width == impl_->width && height == impl_->height) return;
    impl_->CreateSceneTargets(width, height);
}

void Renderer::SetShadowQuality(int q) {
    q = std::min(std::max(q, 0), SHADOW_QUALITY_COUNT - 1);
    if (q == shadowQuality_ && impl_ && impl_->shadowSize) return;
    shadowQuality_ = q;
    if (impl_) impl_->CreateShadowTargets(q);
}

void Renderer::CreateBodyMesh(Body& b, const ShapeTemplate& st) {
    if (b.meshReady) return;
    Mesh m = {};
    m.vertexCount = (int)st.meshRest.size();
    m.triangleCount = (int)st.indices.size() / 3;
    m.vertices = (float*)RL_CALLOC(m.vertexCount * 3, sizeof(float));
    m.normals = (float*)RL_CALLOC(m.vertexCount * 3, sizeof(float));
    m.indices = (unsigned short*)RL_MALLOC(st.indices.size() * sizeof(unsigned short));
    std::memcpy(m.indices, st.indices.data(), st.indices.size() * sizeof(unsigned short));
    // Rest pose placed at the body's current transform until the first skinning
    for (int i = 0; i < m.vertexCount; i++) {
        Vector3 p = b.center + b.rot * (st.meshRest[i] * b.scale);
        Vector3 n = b.rot * Norm(st.meshRest[i]);
        std::memcpy(&m.vertices[3 * i], &p, sizeof(Vector3));
        std::memcpy(&m.normals[3 * i], &n, sizeof(Vector3));
    }
    UploadMesh(&m, true);
    b.mesh = m;
    b.meshReady = true;
}

void Renderer::DestroyBodyMesh(Body& b) {
    if (!b.meshReady) return;
    UnloadMesh(b.mesh);
    b.mesh = {};
    b.meshReady = false;
}

void Renderer::UpdateBodyMeshes(World& world) {
    for (Body& b : world.bodies) {
        const ShapeTemplate& st = world.Template(b);
        if (!b.meshReady) CreateBodyMesh(b, st);
        SkinBody(world, b, st, b.mesh);
        UpdateMeshBuffer(b.mesh, 0, b.mesh.vertices, b.mesh.vertexCount * 3 * sizeof(float), 0);
        UpdateMeshBuffer(b.mesh, 2, b.mesh.normals, b.mesh.vertexCount * 3 * sizeof(float), 0);
    }
}

void Renderer::Render(const Camera3D& cam, const World& world, const SkyState& sky, int hovered, int grabbed, float time) {
    if (!impl_) return;
    Impl& R = *impl_;
    const int sw = GetScreenWidth(), sh = GetScreenHeight();
    if (sw > 0 && sh > 0 && (sw != R.width || sh != R.height)) R.CreateSceneTargets(sw, sh);
    R.EnsurePlatform(world.params);
    const Matrix platformXf = R.PlatformTransform();

    // --- light matrices ---
    const Vector3 L = sky.lightDir;
    const Vector3 center = {0, 1.0f, 0};
    const Vector3 up = std::fabs(L.y) > 0.99f ? Vector3{0, 0, 1} : Vector3{0, 1, 0};
    const Matrix lightView = MatrixLookAt(center + L * kLightDistance, center, up);
    const Matrix lightProj = MatrixOrtho(-kLightHalfExtent, kLightHalfExtent, -kLightHalfExtent, kLightHalfExtent, kLightNear, kLightFar);
    const Matrix lightVP = MatrixMultiply(lightView, lightProj);
    const Camera3D dummy = {{0, 10, 10}, {0, 0, 0}, {0, 1, 0}, 45.0f, CAMERA_PERSPECTIVE};

    auto beginLight = [&](RenderTexture2D rt) {
        BeginTextureMode(rt);
        ClearBackground({255, 255, 255, 0});
        BeginMode3D(dummy);
        rlSetMatrixProjection(lightProj);
        rlSetMatrixModelview(lightView);
    };

    // --- shadow pass: opaque platform (back faces -> no acne on the top) ---
    beginLight(R.rtShOpaque);
    rlSetCullFace(RL_CULL_FACE_FRONT);
    DrawMesh(R.platform, R.depth.mat, platformXf);
    rlSetCullFace(RL_CULL_FACE_BACK);
    EndMode3D();
    EndTextureMode();

    // --- shadow pass: jellies (nearest depth, then multiplied colour) ---
    beginLight(R.rtShJelly);
    rlColorMask(false, false, false, false);
    for (const Body& b : world.bodies)
        if (b.meshReady) DrawMesh(b.mesh, R.depth.mat, MatrixIdentity());
    rlColorMask(true, true, true, true);
    rlDisableDepthTest();
    rlDisableDepthMask();
    rlSetBlendFactorsSeparate(kGlDstColor, kGlZero, kGlOne, kGlOne, kGlFuncAdd, kGlMax);
    BeginBlendMode(BLEND_CUSTOM_SEPARATE);
    R.tint.V3("lightDir", L);
    R.tint.F("causticGain", R.caustics);
    for (const Body& b : world.bodies) {
        if (!b.meshReady) continue;
        R.tint.V3("tint", Mix3(b.color, {1, 1, 1}, 0.25f) * 0.6f);
        DrawMesh(b.mesh, R.tint.mat, MatrixIdentity());
    }
    EndBlendMode();
    rlEnableDepthMask();
    rlEnableDepthTest();
    EndMode3D();
    EndTextureMode();

    // --- opaque scene into A ---
    BeginTextureMode(R.rtA);
    ClearBackground({(unsigned char)(sky.horizon.x * 255), (unsigned char)(sky.horizon.y * 255), (unsigned char)(sky.horizon.z * 255), 255});
    BeginMode3D(cam);
    const Matrix view = rlGetMatrixModelview();
    const Matrix proj = rlGetMatrixProjection();
    const Matrix viewProj = MatrixMultiply(view, proj);

    R.SetCommon(R.skyS, sky, cam.position, lightVP);
    R.skyS.V3("sunDir", sky.sunDir);
    R.skyS.F("night", sky.night);
    R.skyS.F("time", time);
    rlDisableDepthTest();
    rlDisableBackfaceCulling();
    DrawMesh(R.sky, R.skyS.mat, MatrixMultiply(MatrixScale(400, 400, 400), MatrixTranslate(cam.position.x, cam.position.y, cam.position.z)));
    rlEnableBackfaceCulling();
    rlEnableDepthTest();

    R.SetCommon(R.seaS, sky, cam.position, lightVP);
    R.seaS.F("time", time);
    DrawMesh(R.sea, R.seaS.mat, MatrixTranslate(cam.position.x, kSeaLevel, cam.position.z));

    R.SetCommon(R.platformS, sky, cam.position, lightVP);
    R.platformS.F("platformHalf", R.platHalf);
    R.platformS.F("causticStrength", R.caustics);
    R.platformS.F("time", time);
    DrawMesh(R.platform, R.platformS.mat, platformXf);
    splash::DrawStains(cam, sky.lightDir, sky.lightColor, sky.ambientSky);
    EndMode3D();
    EndTextureMode();

    // --- B = A + jellies (refraction samples A) ---
    BeginTextureMode(R.rtB);
    rlDisableDepthTest();
    DrawTexturePro(R.rtA.texture, {0, 0, (float)R.width, -(float)R.height}, {0, 0, (float)R.width, (float)R.height}, {0, 0}, 0, WHITE);
    rlDrawRenderBatchActive();
    BeginMode3D(cam);

    std::vector<std::pair<float, int>> order;
    order.reserve(world.bodies.size());
    for (int i = 0; i < (int)world.bodies.size(); i++)
        if (world.bodies[i].meshReady) order.push_back({Len2(world.bodies[i].center - cam.position), i});
    std::sort(order.begin(), order.end(), [](auto& a, auto& b) { return a.first > b.first; });

    R.SetCommon(R.jelly, sky, cam.position, lightVP);
    R.jelly.SetTexture(4, R.colorA);
    R.jelly.M("viewProj", viewProj);
    rlSetBlendFactorsSeparate(kGlSrcAlpha, kGlOneMinusSrcAlpha, kGlOne, kGlOne, kGlFuncAdd, kGlFuncAdd);
    BeginBlendMode(BLEND_CUSTOM_SEPARATE);
    for (auto& [d, i] : order) {
        const Body& b = world.bodies[i];
        R.jelly.V3("jellyColor", b.color);
        R.jelly.F("jellySize", world.Template(b).boundRadius * b.scale);
        R.jelly.F("highlight", i == grabbed ? 1.0f : (i == hovered ? 0.5f : 0.0f));
        DrawMesh(b.mesh, R.jelly.mat, MatrixIdentity());
    }
    EndBlendMode();
    eyes::Draw(world, cam, sky.lightDir, sky.lightColor, (sky.ambientSky + sky.ambientGround) * 0.5f);
    splash::DrawDroplets(cam, sky.lightDir, sky.lightColor, sky.ambientSky);
    EndMode3D();
    EndTextureMode();

    // --- present ---
    DrawTexturePro(R.rtB.texture, {0, 0, (float)R.width, -(float)R.height}, {0, 0, (float)sw, (float)sh}, {0, 0}, 0, WHITE);
}
