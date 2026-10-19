#include "shapes.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace {

using Field = std::function<float(Vector3)>;

float Clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

float SMin(float a, float b, float k) {
    float h = Clamp01(0.5f + 0.5f * (b - a) / k);
    return b + (a - b) * h - k * h * (1.0f - h);
}

float SdSphere(Vector3 p, Vector3 c, float r) { return Len(p - c) - r; }

float SdEllipsoid(Vector3 p, Vector3 c, Vector3 r) {
    Vector3 q = p - c;
    float k0 = Len({q.x / r.x, q.y / r.y, q.z / r.z});
    float k1 = Len({q.x / (r.x * r.x), q.y / (r.y * r.y), q.z / (r.z * r.z)});
    return k1 > 1e-6f ? k0 * (k0 - 1.0f) / k1 : -std::min(r.x, std::min(r.y, r.z));
}

float SdRoundBox(Vector3 p, Vector3 b, float r) {
    Vector3 q = {std::fabs(p.x) - b.x + r, std::fabs(p.y) - b.y + r, std::fabs(p.z) - b.z + r};
    Vector3 m = {std::max(q.x, 0.0f), std::max(q.y, 0.0f), std::max(q.z, 0.0f)};
    return Len(m) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f) - r;
}

// Rounded extrusion of a 2D distance along z with half-thickness t
float Extrude(float d2, float z, float t, float round) {
    float wx = d2 + round, wy = std::fabs(z) - (t - round);
    float outside = std::sqrt(std::max(wx, 0.0f) * std::max(wx, 0.0f) + std::max(wy, 0.0f) * std::max(wy, 0.0f));
    return std::min(std::max(wx, wy), 0.0f) + outside - round;
}

float Dot2(Vector2 v) { return v.x * v.x + v.y * v.y; }

// Inigo Quilez' 2D heart, tip at the origin, ~1.1 tall
float SdHeart2(Vector2 p) {
    p.x = std::fabs(p.x);
    if (p.y + p.x > 1.0f) return std::sqrt(Dot2({p.x - 0.25f, p.y - 0.75f})) - std::sqrt(2.0f) / 4.0f;
    float m = std::max(p.x + p.y, 0.0f) * 0.5f;
    float d = std::min(Dot2({p.x, p.y - 1.0f}), Dot2({p.x - m, p.y - m}));
    return std::sqrt(d) * (p.x - p.y > 0 ? 1.0f : -1.0f);
}

// Inigo Quilez' 5-point star
float SdStar5(Vector2 p, float r, float rf) {
    const Vector2 k1 = {0.809016994375f, -0.587785252292f};
    const Vector2 k2 = {-k1.x, k1.y};
    p.x = std::fabs(p.x);
    float d1 = std::max(k1.x * p.x + k1.y * p.y, 0.0f);
    p = {p.x - 2 * d1 * k1.x, p.y - 2 * d1 * k1.y};
    float d2 = std::max(k2.x * p.x + k2.y * p.y, 0.0f);
    p = {p.x - 2 * d2 * k2.x, p.y - 2 * d2 * k2.y};
    p.x = std::fabs(p.x);
    p.y -= r;
    Vector2 ba = {rf * -k1.y, rf * k1.x - 1.0f};
    float h = std::min(std::max((p.x * ba.x + p.y * ba.y) / Dot2(ba), 0.0f), r);
    Vector2 q = {p.x - ba.x * h, p.y - ba.y * h};
    return std::sqrt(Dot2(q)) * ((p.y * ba.x - p.x * ba.y) > 0 ? 1.0f : -1.0f);
}

// Inigo Quilez' pyramid: unit square base at y=0, apex at y=h
float SdPyramid(Vector3 p, float h) {
    float m2 = h * h + 0.25f;
    float px = std::fabs(p.x), pz = std::fabs(p.z);
    if (pz > px) std::swap(px, pz);
    px -= 0.5f;
    pz -= 0.5f;
    Vector3 q = {pz, h * p.y - 0.5f * px, h * px + 0.5f * p.y};
    float s = std::max(-q.x, 0.0f);
    float t = Clamp01((q.y - 0.5f * pz) / (m2 + 0.25f));
    float a = m2 * (q.x + s) * (q.x + s) + q.y * q.y;
    float b = m2 * (q.x + 0.5f * t) * (q.x + 0.5f * t) + (q.y - m2 * t) * (q.y - m2 * t);
    float d2 = std::min(q.y, -q.x * m2 - q.y * 0.5f) > 0.0f ? 0.0f : std::min(a, b);
    return std::sqrt((d2 + q.z * q.z) / m2) * (std::max(q.z, -p.y) > 0 ? 1.0f : -1.0f);
}

Field MakeField(ShapeKind kind) {
    switch (kind) {
        case SHAPE_CUBE:
            return [](Vector3 p) { return SdRoundBox(p, {0.42f, 0.42f, 0.42f}, 0.12f); };
        case SHAPE_SPHERE:
            return [](Vector3 p) { return SdSphere(p, {0, 0, 0}, 0.5f); };
        case SHAPE_BEAR:
            return [](Vector3 p) {
                // Gummy bear, slightly flattened front-to-back
                const float zs = 0.78f;
                Vector3 q = {p.x, p.y, p.z / zs};
                float d = SdEllipsoid(q, {0, -0.1f, 0}, {0.26f, 0.29f, 0.24f});
                d = SMin(d, SdSphere(q, {0, 0.28f, 0}, 0.23f), 0.07f);
                d = SMin(d, SdSphere(q, {0, 0.22f, 0.17f}, 0.1f), 0.05f);
                d = SMin(d, SdSphere(q, {0.17f, 0.46f, 0}, 0.085f), 0.05f);
                d = SMin(d, SdSphere(q, {-0.17f, 0.46f, 0}, 0.085f), 0.05f);
                d = SMin(d, SdSphere(q, {0.26f, 0.0f, 0.05f}, 0.1f), 0.06f);
                d = SMin(d, SdSphere(q, {-0.26f, 0.0f, 0.05f}, 0.1f), 0.06f);
                d = SMin(d, SdEllipsoid(q, {0.15f, -0.37f, 0.05f}, {0.12f, 0.11f, 0.13f}), 0.06f);
                d = SMin(d, SdEllipsoid(q, {-0.15f, -0.37f, 0.05f}, {0.12f, 0.11f, 0.13f}), 0.06f);
                return d * zs;
            };
        case SHAPE_WORM:
            return [](Vector3 p) {
                // Chain of spheres along a gentle S-curve, with ridges
                float d = 1e9f;
                const int n = 14;
                for (int i = 0; i < n; i++) {
                    float t = i / float(n - 1);
                    float x = -0.62f + 1.24f * t;
                    Vector3 c = {x, 0.0f, 0.16f * std::sin(t * 2.0f * PI)};
                    float r = 0.17f + 0.03f * std::sin(t * PI) - (t > 0.92f ? 0.02f : 0.0f);
                    d = SMin(d, SdSphere(p, c, r), 0.08f);
                }
                float ridges = 0.008f * std::sin(p.x * 38.0f);
                return d + ridges;
            };
        case SHAPE_HEART:
            return [](Vector3 p) {
                const float k = 0.92f;
                float d2 = SdHeart2({p.x / k, p.y / k + 0.55f}) * k;
                return Extrude(d2, p.z, 0.22f, 0.13f);
            };
        case SHAPE_STAR:
            return [](Vector3 p) {
                float d2 = SdStar5({p.x, p.y + 0.04f}, 0.56f, 0.5f);
                return Extrude(d2, p.z, 0.2f, 0.1f);
            };
        case SHAPE_CYLINDER:
            return [](Vector3 p) {
                const float R = 0.42f, H = 0.42f, rb = 0.1f;
                float dx = std::sqrt(p.x * p.x + p.z * p.z) - (R - rb);
                float dy = std::fabs(p.y) - (H - rb);
                float out = std::sqrt(std::max(dx, 0.0f) * std::max(dx, 0.0f) + std::max(dy, 0.0f) * std::max(dy, 0.0f));
                return std::min(std::max(dx, dy), 0.0f) + out - rb;
            };
        case SHAPE_PYRAMID:
        default:
            return [](Vector3 p) {
                const float k = 0.95f, rr = 0.07f;
                Vector3 q = {p.x / k, (p.y + 0.3f) / k, p.z / k};
                return SdPyramid(q, 0.92f) * k - rr;
            };
    }
}

const char* kShapeNames[SHAPE_COUNT] = {"cube", "sphere", "bear", "worm", "heart", "star", "cylinder", "pyramid"};

// --- particles & clusters ----------------------------------------------------

struct LatticeBuild {
    std::vector<Vector3> pos;
    std::vector<std::array<int, 3>> ijk;
};

LatticeBuild SampleLattice(const Field& f, float h) {
    LatticeBuild lb;
    const int N = (int)std::ceil(0.8f / h);
    for (int i = -N; i <= N; i++)
        for (int j = -N; j <= N; j++)
            for (int k = -N; k <= N; k++) {
                Vector3 p = {i * h, j * h, k * h};
                if (f(p) < -0.25f * h) {
                    lb.pos.push_back(p);
                    lb.ijk.push_back({i, j, k});
                }
            }
    return lb;
}

void BuildClusters(ShapeTemplate& st, const LatticeBuild& lb) {
    const int n = (int)lb.pos.size();
    std::vector<int> covered(n, 0);
    auto addCluster = [&](int center) {
        Cluster c;
        const auto& a = lb.ijk[center];
        for (int m = 0; m < n; m++) {
            const auto& b = lb.ijk[m];
            if (std::abs(a[0] - b[0]) <= 1 && std::abs(a[1] - b[1]) <= 1 && std::abs(a[2] - b[2]) <= 1)
                c.members.push_back(m);
        }
        if (c.members.size() < 5) return;
        Vector3 sum = {0, 0, 0};
        for (int m : c.members) {
            sum += st.rest[m];
            covered[m] = 1;
        }
        c.restCenter = sum * (1.0f / c.members.size());
        st.clusters.push_back(std::move(c));
    };
    for (int i = 0; i < n; i++) {
        const auto& a = lb.ijk[i];
        if (((a[0] & 1) == 0) && ((a[1] & 1) == 0) && ((a[2] & 1) == 0)) addCluster(i);
    }
    for (int i = 0; i < n; i++)
        if (!covered[i]) addCluster(i);

    // Global cluster keeps the overall shape; it is always the last one
    Cluster g;
    for (int i = 0; i < n; i++) g.members.push_back(i);
    g.restCenter = {0, 0, 0};
    st.clusters.push_back(std::move(g));
}

// --- surface nets mesh -------------------------------------------------------

void BuildMesh(ShapeTemplate& st, const Field& f) {
    const float g = 0.042f;
    const float lo = -0.8f;
    const int n = (int)std::round(1.6f / g) + 1;
    auto nodeIdx = [n](int i, int j, int k) { return (i * n + j) * n + k; };
    auto cellIdx = [n](int i, int j, int k) { return (i * (n - 1) + j) * (n - 1) + k; };

    std::vector<float> F(n * n * n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++) F[nodeIdx(i, j, k)] = f({lo + i * g, lo + j * g, lo + k * g});

    std::vector<int> cellVert((n - 1) * (n - 1) * (n - 1), -1);
    std::vector<Vector3> verts;
    static const int corner[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
    static const int edges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (int i = 0; i < n - 1; i++)
        for (int j = 0; j < n - 1; j++)
            for (int k = 0; k < n - 1; k++) {
                float v[8];
                int mask = 0;
                for (int c = 0; c < 8; c++) {
                    v[c] = F[nodeIdx(i + corner[c][0], j + corner[c][1], k + corner[c][2])];
                    if (v[c] < 0) mask |= 1 << c;
                }
                if (mask == 0 || mask == 255) continue;
                Vector3 sum = {0, 0, 0};
                int cnt = 0;
                for (auto& e : edges) {
                    float a = v[e[0]], b = v[e[1]];
                    if ((a < 0) == (b < 0)) continue;
                    float t = a / (a - b);
                    Vector3 pa = {(float)corner[e[0]][0], (float)corner[e[0]][1], (float)corner[e[0]][2]};
                    Vector3 pb = {(float)corner[e[1]][0], (float)corner[e[1]][1], (float)corner[e[1]][2]};
                    sum += pa + (pb - pa) * t;
                    cnt++;
                }
                Vector3 local = sum * (1.0f / cnt);
                cellVert[cellIdx(i, j, k)] = (int)verts.size();
                verts.push_back({lo + (i + local.x) * g, lo + (j + local.y) * g, lo + (k + local.z) * g});
            }

    std::vector<int> tris;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++) {
                int base[3] = {i, j, k};
                bool in0 = F[nodeIdx(i, j, k)] < 0;
                for (int axis = 0; axis < 3; axis++) {
                    int nb[3] = {i, j, k};
                    nb[axis]++;
                    if (nb[axis] >= n) continue;
                    bool in1 = F[nodeIdx(nb[0], nb[1], nb[2])] < 0;
                    if (in0 == in1) continue;
                    int u = (axis + 1) % 3, w = (axis + 2) % 3;
                    if (base[u] < 1 || base[w] < 1 || base[u] >= n - 1 || base[w] >= n - 1) continue;
                    static const int du[4] = {-1, 0, 0, -1};
                    static const int dw[4] = {-1, -1, 0, 0};
                    int q[4];
                    bool ok = true;
                    for (int c = 0; c < 4; c++) {
                        int cc[3] = {base[0], base[1], base[2]};
                        cc[u] += du[c];
                        cc[w] += dw[c];
                        q[c] = cellVert[cellIdx(cc[0], cc[1], cc[2])];
                        if (q[c] < 0) ok = false;
                    }
                    if (!ok) continue;
                    Vector3 outward = {0, 0, 0};
                    float s = in0 ? 1.0f : -1.0f;
                    if (axis == 0) outward.x = s;
                    else if (axis == 1) outward.y = s;
                    else outward.z = s;
                    Vector3 nrm = Cross(verts[q[1]] - verts[q[0]], verts[q[2]] - verts[q[0]]);
                    if (Dot(nrm, outward) < 0) std::swap(q[1], q[3]);
                    tris.insert(tris.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
                }
            }

    // Pull vertices onto the implicit surface
    const float e = g * 0.1f;
    for (auto& p : verts) {
        for (int it = 0; it < 3; it++) {
            float d = f(p);
            Vector3 grad = {f({p.x + e, p.y, p.z}) - f({p.x - e, p.y, p.z}), f({p.x, p.y + e, p.z}) - f({p.x, p.y - e, p.z}),
                            f({p.x, p.y, p.z + e}) - f({p.x, p.y, p.z - e})};
            grad = grad * (1.0f / (2 * e));
            float gl2 = Len2(grad);
            if (gl2 < 1e-8f) break;
            Vector3 step = grad * (-d / gl2);
            float sl = Len(step);
            if (sl > g * 0.5f) step = step * (g * 0.5f / sl);
            p += step;
        }
    }

    st.meshRest = verts;
    st.indices.assign(tris.begin(), tris.end());
}

void BuildSkin(ShapeTemplate& st) {
    const size_t nv = st.meshRest.size();
    st.skinIdx.resize(nv);
    st.skinW.resize(nv);
    st.skinOffset.resize(nv);
    const float soft = 0.5f * st.spacing;
    for (size_t v = 0; v < nv; v++) {
        Vector3 p = st.meshRest[v];
        int best[4] = {0, 0, 0, 0};
        float bestD[4] = {1e9f, 1e9f, 1e9f, 1e9f};
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
        float wsum = 0;
        float w[4];
        for (int k = 0; k < 4; k++) {
            w[k] = 1.0f / (bestD[k] + soft * soft);
            wsum += w[k];
        }
        Vector3 blend = {0, 0, 0};
        for (int k = 0; k < 4; k++) {
            w[k] /= wsum;
            st.skinIdx[v][k] = best[k];
            st.skinW[v][k] = w[k];
            blend += st.rest[best[k]] * w[k];
        }
        st.skinOffset[v] = p - blend;
    }
}

}  // namespace

void BuildShapeTemplates(std::vector<ShapeTemplate>& out) {
    out.clear();
    out.resize(SHAPE_COUNT);
    for (int s = 0; s < SHAPE_COUNT; s++) {
        ShapeTemplate& st = out[s];
        st.name = kShapeNames[s];
        Field f = MakeField((ShapeKind)s);

        // Pick the coarsest lattice that still gives enough particles
        LatticeBuild lb;
        float h = 0.2f;
        for (; h > 0.085f; h -= 0.005f) {
            lb = SampleLattice(f, h);
            if (lb.pos.size() >= 90) break;
        }
        st.spacing = h;
        st.rest = lb.pos;

        BuildMesh(st, f);

        // Centre everything on the particle centroid
        Vector3 c = {0, 0, 0};
        for (auto& p : st.rest) c += p;
        c = c * (1.0f / st.rest.size());
        for (auto& p : st.rest) p -= c;
        for (auto& p : st.meshRest) p -= c;
        st.boundRadius = 0;
        for (auto& p : st.meshRest) st.boundRadius = std::max(st.boundRadius, Len(p));

        BuildClusters(st, lb);
        BuildSkin(st);
    }
}
