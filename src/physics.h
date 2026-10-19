// Soft-body world: Position-Based Dynamics with lattice shape matching
// (overlapping clusters + one global cluster), sub-stepping, velocity
// clamping and a spatial hash for jelly-vs-jelly particle contacts.
#pragma once
#include <unordered_map>
#include <vector>
#include "shapes.h"

struct Body {
    int id = 0;
    int shape = 0;
    float scale = 1.0f;
    Vector3 color = {1, 1, 1};
    int first = 0, count = 0;           // particle range in the world arrays
    std::vector<Quat> clusterRot;       // warm-start rotations per cluster
    Vector3 center = {0, 0, 0};         // centroid of particles
    M3 rot = M3::Identity();            // rotation of the global cluster
    float particleRadius = 0.08f;
    // Contact events gathered during the last Step() (for sounds)
    float impactGround = 0, impactJelly = 0;
    float unstick = 0;  // strength of a sticky contact tearing apart
    float soundCooldown = 0;
    // Render data, owned by the renderer
    Mesh mesh = {};
    bool meshReady = false;
};

struct PhysicsParams {
    float gravity = 9.81f;
    float softness = 0.5f;      // 0 = firm, 1 = very soft
    float platformHalf = 7.0f;  // platform is [-half, half] in x and z
    float platformTop = 0.0f;
    float platformBottom = -1.2f;
    float maxSpeed = 22.0f;     // per-particle speed clamp
    float friction = 0.55f;
    float jellyFriction = 0.35f;
    float internalDamping = 1.6f;
    float stickiness = 0.5f;    // 0 = none, 1 = very sticky (fresh contacts only)
};

class World {
public:
    explicit World(const std::vector<ShapeTemplate>* templates) : tpl_(templates) {}

    int AddBody(int shape, float scale, Vector3 color, Vector3 pos, Quat rot, Vector3 vel);
    void RemoveBody(int index);  // caller releases the render mesh first
    void Clear();
    void Step(float dt, int substeps);

    // Nearest body hit by the ray; returns body index or -1
    int Raycast(Ray ray, float* hitDist, int* hitParticle) const;

    void BeginGrab(int body, int particle, Vector3 grabPoint);
    void SetGrabTarget(Vector3 grabPoint) { grabTarget_ = grabPoint; }
    void EndGrab(Vector3 throwVelocity, float maxThrowSpeed);
    void CancelGrab() { grabBody_ = -1; grabParticles_.clear(); }
    int GrabbedBody() const { return grabBody_; }

    const ShapeTemplate& Template(const Body& b) const { return (*tpl_)[b.shape]; }

    PhysicsParams params;
    std::vector<Body> bodies;
    // Particle state (structure of arrays)
    std::vector<Vector3> x, xprev, v;
    std::vector<float> radius, invMass;
    std::vector<int> owner;

private:
    void Substep(float dt);
    void ShapeMatch(float dt);
    void ApplyGrab();
    void CollideParticles();
    void CollidePlatform();
    void DampBodies(float dt);
    void UpdateSticky(float dt);

    const std::vector<ShapeTemplate>* tpl_;
    int nextId_ = 1;

    // Grab constraint
    int grabBody_ = -1;
    int grabAnchor_ = -1;
    Vector3 grabTarget_ = {0, 0, 0};
    Vector3 grabOffset_ = {0, 0, 0};
    std::vector<std::pair<int, float>> grabParticles_;

    // Scratch buffers
    std::vector<Vector3> goalSum_;
    std::vector<float> goalCnt_;
    std::vector<float> vnIn_;
    std::vector<unsigned char> grounded_;
    std::vector<Vector3> boxMin_, boxMax_;
    std::vector<std::pair<long long, int>> cellEntries_;
    std::vector<std::pair<int, int>> pairs_;
    std::vector<int> listA_, listB_;

    void CollideBodyPair(int a, int b);

    // Stickiness: contact age per body pair (key from body ids) and per body
    // with the platform (key with 0 as the second id). Adhesion is strong right
    // after contact and fades within ~2 s.
    struct StickyContact {
        float age = 0;          // seconds spent in contact
        float since = 0;        // seconds since the last contact
        bool touched = false;   // in contact during the current Step
        bool released = false;  // separation already reported
    };
    std::unordered_map<long long, StickyContact> sticky_;
    std::vector<float> groundStick_;
    std::vector<unsigned char> groundTouched_;
    std::vector<Vector3> bodyVel_;  // mean velocity per body at the start of the Step
    float subDt_ = 1.0f / 300.0f;
};
