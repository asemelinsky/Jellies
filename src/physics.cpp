#include "physics.h"

#include <algorithm>
#include <cmath>

int World::AddBody(int shape, float scale, Vector3 color, Vector3 pos, Quat rot, Vector3 vel) {
    const ShapeTemplate& st = (*tpl_)[shape];
    Body b;
    b.id = nextId_++;
    b.shape = shape;
    b.scale = scale;
    b.color = color;
    b.first = (int)x.size();
    b.count = (int)st.rest.size();
    b.clusterRot.assign(st.clusters.size(), rot);
    b.rot = QToM3(rot);
    b.center = pos;
    b.particleRadius = 0.5f * st.spacing * scale;
    const float im = 1.0f / (scale * scale * scale);
    for (const Vector3& r : st.rest) {
        Vector3 p = pos + b.rot * (r * scale);
        x.push_back(p);
        xprev.push_back(p);
        v.push_back(vel);
        radius.push_back(b.particleRadius);
        invMass.push_back(im);
        owner.push_back((int)bodies.size());
    }
    bodies.push_back(std::move(b));
    return (int)bodies.size() - 1;
}

void World::RemoveBody(int index) {
    if (index < 0 || index >= (int)bodies.size()) return;
    if (grabBody_ == index) CancelGrab();
    else if (grabBody_ > index) grabBody_--;

    const int first = bodies[index].first, count = bodies[index].count;
    auto eraseRange = [&](auto& vec) { vec.erase(vec.begin() + first, vec.begin() + first + count); };
    eraseRange(x);
    eraseRange(xprev);
    eraseRange(v);
    eraseRange(radius);
    eraseRange(invMass);
    eraseRange(owner);
    bodies.erase(bodies.begin() + index);
    for (int b = index; b < (int)bodies.size(); b++) {
        bodies[b].first -= count;
        for (int i = bodies[b].first; i < bodies[b].first + bodies[b].count; i++) owner[i] = b;
    }
}

void World::Clear() {
    CancelGrab();
    bodies.clear();
    x.clear();
    xprev.clear();
    v.clear();
    radius.clear();
    invMass.clear();
    owner.clear();
}

void World::Step(float dt, int substeps) {
    for (Body& b : bodies) {
        b.impactGround = 0;
        b.impactJelly = 0;
        b.soundCooldown = std::max(0.0f, b.soundCooldown - dt);
    }
    if (bodies.empty() || dt <= 0) return;
    const float h = dt / substeps;
    for (int s = 0; s < substeps; s++) Substep(h);
}

void World::Substep(float dt) {
    const size_t n = x.size();
    goalSum_.resize(n);
    goalCnt_.resize(n);
    vnIn_.resize(n);
    grounded_.resize(n);

    const float drag = std::exp(-0.05f * dt);
    for (size_t i = 0; i < n; i++) {
        v[i].y -= params.gravity * dt;
        v[i] = v[i] * drag;
        vnIn_[i] = v[i].y;
        grounded_[i] = 0;
        xprev[i] = x[i];
        x[i] += v[i] * dt;
    }

    ApplyGrab();
    ShapeMatch(dt);
    CollideParticles();
    CollidePlatform();

    const float inv = 1.0f / dt;
    const float maxV = params.maxSpeed;
    for (size_t i = 0; i < n; i++) {
        Vector3 nv = (x[i] - xprev[i]) * inv;
        float sp = Len(nv);
        if (sp > maxV) nv = nv * (maxV / sp);
        v[i] = nv;
        if (grounded_[i]) {
            Body& b = bodies[owner[i]];
            b.impactGround = std::max(b.impactGround, -vnIn_[i]);
        }
    }
    DampBodies(dt);
}

void World::ApplyGrab() {
    if (grabBody_ < 0) return;
    const Body& b = bodies[grabBody_];
    const ShapeTemplate& st = (*tpl_)[b.shape];
    const Vector3 anchorTarget = grabTarget_ + grabOffset_;
    const Vector3 anchorRest = st.rest[grabAnchor_];
    for (auto& [idx, w] : grabParticles_) {
        Vector3 t = anchorTarget + b.rot * ((st.rest[idx] - anchorRest) * b.scale);
        int gi = b.first + idx;
        x[gi] += (t - x[gi]) * (0.35f * w);
    }
}

void World::ShapeMatch(float dt) {
    // Softness maps to the oscillation frequency of the jelly (Hz). PBD with a
    // relaxation factor alpha behaves like a spring with omega = sqrt(alpha)/dt.
    const float freq = 24.0f * std::pow(0.2f, params.softness);
    float alpha = (2.0f * PI * freq * dt);
    alpha = std::min(1.0f, alpha * alpha);

    for (Body& b : bodies) {
        const ShapeTemplate& st = (*tpl_)[b.shape];
        for (int i = b.first; i < b.first + b.count; i++) {
            goalSum_[i] = {0, 0, 0};
            goalCnt_[i] = 0;
        }
        const int nc = (int)st.clusters.size();
        for (int ci = 0; ci < nc; ci++) {
            const Cluster& cl = st.clusters[ci];
            Vector3 c = {0, 0, 0};
            for (int m : cl.members) c += x[b.first + m];
            c = c * (1.0f / cl.members.size());
            M3 A = M3::Zero();
            for (int m : cl.members) A.AddOuter(x[b.first + m] - c, st.rest[m] - cl.restCenter);
            Quat& q = b.clusterRot[ci];
            ExtractRotation(A, q, 3);
            M3 R = QToM3(q);
            for (int m : cl.members) {
                goalSum_[b.first + m] += c + R * ((st.rest[m] - cl.restCenter) * b.scale);
                goalCnt_[b.first + m] += 1.0f;
            }
            if (ci == nc - 1) {
                b.center = c;
                b.rot = R;
            }
        }
        for (int i = b.first; i < b.first + b.count; i++) {
            Vector3 goal = goalSum_[i] * (1.0f / goalCnt_[i]);
            x[i] += (goal - x[i]) * alpha;
        }
    }
}

void World::CollideParticles() {
    const int nb = (int)bodies.size();
    if (nb < 2) return;

    // Particle-centre bounding boxes per body
    boxMin_.resize(nb);
    boxMax_.resize(nb);
    for (int b = 0; b < nb; b++) {
        Vector3 lo = x[bodies[b].first], hi = lo;
        for (int i = bodies[b].first + 1; i < bodies[b].first + bodies[b].count; i++) {
            lo = {std::min(lo.x, x[i].x), std::min(lo.y, x[i].y), std::min(lo.z, x[i].z)};
            hi = {std::max(hi.x, x[i].x), std::max(hi.y, x[i].y), std::max(hi.z, x[i].z)};
        }
        float r = bodies[b].particleRadius;
        boxMin_[b] = lo - Vector3{r, r, r};
        boxMax_[b] = hi + Vector3{r, r, r};
    }

    // Broad phase: spatial hash grid over body boxes; bodies sharing a cell
    // and overlapping become candidate pairs
    const float cell = 1.5f, inv = 1.0f / cell;
    auto key = [](int ix, int iy, int iz) {
        return ((long long)(ix & 0x1FFFFF) << 42) | ((long long)(iy & 0x1FFFFF) << 21) | (long long)(iz & 0x1FFFFF);
    };
    cellEntries_.clear();
    for (int b = 0; b < nb; b++) {
        int x0 = (int)std::floor(boxMin_[b].x * inv), x1 = (int)std::floor(boxMax_[b].x * inv);
        int y0 = (int)std::floor(boxMin_[b].y * inv), y1 = (int)std::floor(boxMax_[b].y * inv);
        int z0 = (int)std::floor(boxMin_[b].z * inv), z1 = (int)std::floor(boxMax_[b].z * inv);
        if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 64) continue;  // exploded body, ignore
        for (int ix = x0; ix <= x1; ix++)
            for (int iy = y0; iy <= y1; iy++)
                for (int iz = z0; iz <= z1; iz++) cellEntries_.push_back({key(ix, iy, iz), b});
    }
    std::sort(cellEntries_.begin(), cellEntries_.end());
    pairs_.clear();
    for (size_t s = 0; s < cellEntries_.size();) {
        size_t e = s;
        while (e < cellEntries_.size() && cellEntries_[e].first == cellEntries_[s].first) e++;
        for (size_t i = s; i < e; i++)
            for (size_t j = i + 1; j < e; j++) {
                int a = cellEntries_[i].second, b = cellEntries_[j].second;
                if (boxMin_[a].x > boxMax_[b].x || boxMin_[b].x > boxMax_[a].x || boxMin_[a].y > boxMax_[b].y ||
                    boxMin_[b].y > boxMax_[a].y || boxMin_[a].z > boxMax_[b].z || boxMin_[b].z > boxMax_[a].z)
                    continue;
                pairs_.push_back({std::min(a, b), std::max(a, b)});
            }
        s = e;
    }
    std::sort(pairs_.begin(), pairs_.end());
    pairs_.erase(std::unique(pairs_.begin(), pairs_.end()), pairs_.end());

    for (auto& [a, b] : pairs_) CollideBodyPair(a, b);
}

void World::CollideBodyPair(int a, int b) {
    // Narrow phase: only particles inside the other body's (padded) box
    const Body& A = bodies[a];
    const Body& B = bodies[b];
    const float pad = A.particleRadius + B.particleRadius;
    auto inside = [](Vector3 p, Vector3 lo, Vector3 hi, float m) {
        return p.x > lo.x - m && p.x < hi.x + m && p.y > lo.y - m && p.y < hi.y + m && p.z > lo.z - m && p.z < hi.z + m;
    };
    listA_.clear();
    listB_.clear();
    for (int i = A.first; i < A.first + A.count; i++)
        if (inside(x[i], boxMin_[b], boxMax_[b], pad)) listA_.push_back(i);
    if (listA_.empty()) return;
    for (int j = B.first; j < B.first + B.count; j++)
        if (inside(x[j], boxMin_[a], boxMax_[a], pad)) listB_.push_back(j);

    const float mu = params.jellyFriction;
    const float rr = pad;
    const float wi = invMass[A.first], wj = invMass[B.first], ws = wi + wj;
    float impact = 0;
    for (int i : listA_)
        for (int j : listB_) {
            Vector3 d = x[i] - x[j];
            float d2 = Len2(d);
            if (d2 >= rr * rr || d2 < 1e-12f) continue;
            float dist = std::sqrt(d2);
            Vector3 nrm = d * (1.0f / dist);
            float pen = rr - dist;
            Vector3 corr = nrm * (pen / ws);
            x[i] += corr * wi;
            x[j] -= corr * wj;

            // Friction on the relative tangential displacement
            Vector3 dr = (x[i] - xprev[i]) - (x[j] - xprev[j]);
            Vector3 dt = dr - nrm * Dot(dr, nrm);
            float lt = Len(dt);
            if (lt > 1e-9f) {
                float f = std::min(mu * pen / lt, 1.0f);
                Vector3 fc = dt * (f / ws);
                x[i] -= fc * wi;
                x[j] += fc * wj;
            }
            impact = std::max(impact, -Dot(v[i] - v[j], nrm));
        }
    if (impact > 0) {
        bodies[a].impactJelly = std::max(bodies[a].impactJelly, impact);
        bodies[b].impactJelly = std::max(bodies[b].impactJelly, impact);
    }
}

void World::CollidePlatform() {
    const float L = params.platformHalf, top = params.platformTop, bottom = params.platformBottom;
    const float muS = params.friction * 1.3f, muK = params.friction;
    const int n = (int)x.size();
    for (int i = 0; i < n; i++) {
        const float r = radius[i];
        Vector3& p = x[i];
        if (std::fabs(p.x) >= L + r || std::fabs(p.z) >= L + r || p.y >= top + r || p.y <= bottom - r) continue;
        float dTop = top + r - p.y;
        float dBot = p.y - (bottom - r);
        float dXp = L + r - p.x, dXn = p.x + L + r;
        float dZp = L + r - p.z, dZn = p.z + L + r;
        float m = std::min({dTop, dBot, dXp, dXn, dZp, dZn});
        if (m == dTop) {
            p.y = top + r;
            grounded_[i] = 1;
            Vector3 d = p - xprev[i];
            Vector3 t = {d.x, 0, d.z};
            float lt = Len(t);
            if (lt < muS * dTop) {
                p.x = xprev[i].x;
                p.z = xprev[i].z;
            } else if (lt > 1e-9f) {
                float k = std::min(muK * dTop / lt, 1.0f);
                p.x -= t.x * k;
                p.z -= t.z * k;
            }
        } else if (m == dBot) {
            p.y = bottom - r;
        } else if (m == dXp) {
            p.x = L + r;
        } else if (m == dXn) {
            p.x = -L - r;
        } else if (m == dZp) {
            p.z = L + r;
        } else {
            p.z = -L - r;
        }
    }
}

void World::DampBodies(float dt) {
    // Damp only the deviation from rigid motion (Müller et al. 2007)
    const float k = 1.0f - std::exp(-params.internalDamping * dt);
    for (Body& b : bodies) {
        const int f = b.first, n = b.count;
        Vector3 xc = {0, 0, 0}, vc = {0, 0, 0};
        for (int i = f; i < f + n; i++) {
            xc += x[i];
            vc += v[i];
        }
        xc = xc * (1.0f / n);
        vc = vc * (1.0f / n);
        Vector3 L = {0, 0, 0};
        M3 I = M3::Zero();
        for (int i = f; i < f + n; i++) {
            Vector3 r = x[i] - xc;
            L += Cross(r, v[i] - vc);
            float r2 = Len2(r);
            I.c[0] += Vector3{r2, 0, 0};
            I.c[1] += Vector3{0, r2, 0};
            I.c[2] += Vector3{0, 0, r2};
            I.AddOuter(r * -1.0f, r);
        }
        bool ok = false;
        M3 Iinv = Inverse(I, &ok);
        Vector3 w = ok ? Iinv * L : Vector3{0, 0, 0};
        for (int i = f; i < f + n; i++) {
            Vector3 r = x[i] - xc;
            Vector3 target = vc + Cross(w, r);
            v[i] += (target - v[i]) * k;
        }
    }
}

int World::Raycast(Ray ray, float* hitDist, int* hitParticle) const {
    int best = -1;
    float bestT = 1e30f;
    int bestP = -1;
    const Vector3 o = ray.position, d = Norm(ray.direction);
    for (int bi = 0; bi < (int)bodies.size(); bi++) {
        const Body& b = bodies[bi];
        const float br = (*tpl_)[b.shape].boundRadius * b.scale * 1.4f;
        Vector3 oc = o - b.center;
        float bq = Dot(oc, d);
        float cq = Len2(oc) - br * br;
        if (cq > 0 && bq > 0) continue;
        if (bq * bq - cq < 0) continue;
        const float r = b.particleRadius * 1.7f;
        for (int i = b.first; i < b.first + b.count; i++) {
            Vector3 op = o - x[i];
            float pb = Dot(op, d);
            float pc = Len2(op) - r * r;
            float disc = pb * pb - pc;
            if (disc < 0) continue;
            float t = -pb - std::sqrt(disc);
            if (t < 0) continue;
            if (t < bestT) {
                bestT = t;
                best = bi;
                bestP = i - b.first;
            }
        }
    }
    if (best >= 0) {
        if (hitDist) *hitDist = bestT;
        if (hitParticle) *hitParticle = bestP;
    }
    return best;
}

void World::BeginGrab(int body, int particle, Vector3 grabPoint) {
    CancelGrab();
    if (body < 0 || body >= (int)bodies.size()) return;
    const Body& b = bodies[body];
    const ShapeTemplate& st = (*tpl_)[b.shape];
    grabBody_ = body;
    grabAnchor_ = particle;
    grabTarget_ = grabPoint;
    grabOffset_ = x[b.first + particle] - grabPoint;
    const float reach = st.spacing * 2.3f;
    for (int i = 0; i < b.count; i++) {
        float d = Len(st.rest[i] - st.rest[particle]);
        if (d <= reach) grabParticles_.push_back({i, 1.0f - 0.6f * (d / reach)});
    }
}

void World::EndGrab(Vector3 throwVelocity, float maxThrowSpeed) {
    if (grabBody_ < 0) return;
    float s = Len(throwVelocity);
    if (s > maxThrowSpeed) throwVelocity = throwVelocity * (maxThrowSpeed / s);
    Body& b = bodies[grabBody_];
    Vector3 vm = {0, 0, 0};
    for (int i = b.first; i < b.first + b.count; i++) vm += v[i];
    vm = vm * (1.0f / b.count);
    Vector3 dv = throwVelocity - vm;
    for (int i = b.first; i < b.first + b.count; i++) v[i] += dv;
    CancelGrab();
}
